/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the TX poll below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	IF-15 (subsystem 1, instructor-approved 2026-10-28): application tasks
 *	never call lwIP or MQTT. Subsystem 1 puts [topic ID][payload] messages
 *	into a TX message buffer; this file takes them with TMO_POL and
 *	publishes them. Messages arriving on the cmd / map topics are copied
 *	into an RX message buffer with TMO_POL before the callback returns.
 *	Both buffers are created and owned by subsystem 1, which passes their
 *	IDs in through mqtt_utk_set_buffers(); until then they are 0 and
 *	skipped. Message format and topics: app_program/subsystem1/s1_topics.h.
 *
 *	tk/tkernel.h is needed for the message-buffer calls. PROHIBIT_DEF_SIZE_T
 *	(tk/syslib.h) stops it typedef'ing size_t, leaving the toolchain's
 *	<stddef.h> definition, pulled in first, as the only one lwIP sees --
 *	the same approach as sys_arch.c.
 */

#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/s1_topics.h"

#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_TX_POLL_MS     20	/* how often TX is checked     */
#define MQTT_UTK_TX_PER_POLL    4	/* most messages per check     */
#define MQTT_UTK_QOS_CMD        1	/* cmd / map subscriptions     */

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Only the cmd and map topics are
 * subscribed; their payloads are at most S1_RX_MSG_MAX - 1 bytes. A longer
 * one is forwarded with an EMPTY payload, so subsystem 1 rejects and counts
 * it (FR8) instead of acting on a truncated message. */
#define MQTT_UTK_RX_UNKNOWN     0xFFu

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

/* IF-15 buffer IDs: written once by subsystem1_init(), read here */
static volatile ID	tx_mbf;
static volatile ID	rx_mbf;
static volatile int32_t	mqtt_up;

static u8_t		rx_topic_id = MQTT_UTK_RX_UNKNOWN;
static UB		rx_msg[S1_RX_MSG_MAX];	/* [topic ID][payload] */
static u16_t		rx_len;
static u8_t		rx_oversized;

/* A TX message taken out but refused by mqtt_publish() (output buffer
 * full) is kept here and tried again on the next poll, so it is not lost. */
static UB		tx_pending[S1_TX_MSG_MAX];
static INT		tx_pending_len;

/* Counters, readable in a debugger */
static UW		tx_published;
static UW		rx_dropped;

static void tx_poll_timeout(void *arg);

void mqtt_utk_set_buffers(int32_t tx, int32_t rx)
{
	tx_mbf = (ID)tx;
	rx_mbf = (ID)rx;
}

int32_t mqtt_utk_connected(void)
{
	return mqtt_up;
}

/* Topic ID -> topic name, QoS and retain flag for a TX message.
 * Returns NULL for an ID this file does not publish. */
static const char *tx_topic(UB id, u8_t *qos, u8_t *retain)
{
	*retain = 0;
	switch(id) {
	case S1_TOPIC_TELEM:	*qos = 0; return S1_TOPIC_NAME_TELEM;
	case S1_TOPIC_HB:	*qos = 0; return S1_TOPIC_NAME_HB;
	case S1_TOPIC_EVT:	*qos = 1; return S1_TOPIC_NAME_EVT;
	default:		*qos = 0; return NULL;
	}
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- note which one it is. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	const char	*cmd = S1_TOPIC_NAME_CMD;
	const char	*map = S1_TOPIC_NAME_MAP;
	u32_t		i;

	(void)arg;

	for(i = 0; cmd[i] != '\0' && topic[i] == cmd[i]; i++) {}
	if(cmd[i] == '\0' && topic[i] == '\0') {
		rx_topic_id = S1_TOPIC_CMD;
	} else {
		for(i = 0; map[i] != '\0' && topic[i] == map[i]; i++) {}
		rx_topic_id = (map[i] == '\0' && topic[i] == '\0')?
			      S1_TOPIC_MAP: MQTT_UTK_RX_UNKNOWN;
	}

	rx_msg[0] = rx_topic_id;
	rx_len = 1;
	rx_oversized = (tot_len > (S1_RX_MSG_MAX - 1u));
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;
	INT	msg_len;

	(void)arg;

	for(i = 0; i < len && !rx_oversized; i++) {
		if(rx_len < S1_RX_MSG_MAX) {
			rx_msg[rx_len++] = data[i];
		} else {
			rx_oversized = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) == 0) {
		return;
	}

	/* Copy into the RX buffer now: the data is gone after we return */
	if(rx_topic_id == MQTT_UTK_RX_UNKNOWN || rx_mbf <= 0) {
		rx_dropped++;
		return;
	}
	msg_len = rx_oversized? 1: (INT)rx_len;	/* oversized: topic only */
	if(tk_snd_mbf(rx_mbf, rx_msg, msg_len, TMO_POL) != E_OK) {
		rx_dropped++;
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

/* Publish one [topic ID][payload] message. ERR_OK if mqtt.c took it. */
static err_t tx_publish(const UB *msg, INT len)
{
	const char	*topic;
	u8_t		qos;
	u8_t		retain;

	topic = tx_topic(msg[0], &qos, &retain);
	if(topic == NULL || len < 2) {
		return ERR_OK;		/* not ours to publish: drop it */
	}
	return mqtt_publish(client, topic, &msg[1], (u16_t)(len - 1),
			    qos, retain, NULL, NULL);
}

/* Every MQTT_UTK_TX_POLL_MS: while connected, publish up to
 * MQTT_UTK_TX_PER_POLL messages from the TX buffer. While not connected
 * the buffer is left alone, so it fills and subsystem 1 keeps telemetry
 * in its outage buffer for replay. */
static void tx_poll_timeout(void *arg)
{
	INT	n;
	INT	len;

	(void)arg;

	if(tx_mbf > 0 && client != NULL && mqtt_client_is_connected(client)) {
		for(n = 0; n < MQTT_UTK_TX_PER_POLL; n++) {
			if(tx_pending_len == 0) {
				len = tk_rcv_mbf(tx_mbf, tx_pending, TMO_POL);
				if(len <= 0) {
					break;			/* TX empty */
				}
				tx_pending_len = len;
			}
			if(tx_publish(tx_pending, tx_pending_len) != ERR_OK) {
				break;		/* mqtt.c busy: retry next poll */
			}
			tx_pending_len = 0;
			tx_published++;
		}
	}

	sys_timeout(MQTT_UTK_TX_POLL_MS, tx_poll_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		mqtt_up = 1;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe here, on every connect, so the subscriptions come
		 * back after a reconnect. Callbacks must be registered before
		 * subscribing, or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, S1_TOPIC_NAME_CMD, MQTT_UTK_QOS_CMD,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe(cmd) failed, err=%d\n",
				  (int)err);
		}
		err = mqtt_subscribe(client, S1_TOPIC_NAME_MAP, MQTT_UTK_QOS_CMD,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe(map) failed, err=%d\n",
				  (int)err);
		}

		/* Exactly one TX poll chain: drop any left from a previous
		 * connection before arming a new one. Each would otherwise hold
		 * a MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(tx_poll_timeout, NULL);
		sys_timeout(MQTT_UTK_TX_POLL_MS, tx_poll_timeout, NULL);
	} else {
		mqtt_up = 0;

		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err == ERR_ISCONN) {
		/* An attempt is already under way (retry_timeout() cleared the
		 * guard while a connect started by the poll loop was still
		 * waiting on TCP). Keep the guard set and let that attempt
		 * finish: clearing it here made the poll loop call this again
		 * every few ms, printing this error each time. */
		return;
	}
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}