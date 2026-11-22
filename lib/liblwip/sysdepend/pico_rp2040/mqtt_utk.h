/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Plain C on purpose (no kernel types), so application code can include
 *	it without the size_t clash between tk/tkernel.h and lwIP.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h). Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/*
 * IF-15 (subsystem 1). Hand over the two message buffers, by kernel ID:
 *   tx_mbf  [topic ID][payload] messages to publish  (s1_topics.h)
 *   rx_mbf  [topic ID][payload] messages received on the cmd / map topics
 * Call from subsystem1_init() after creating both buffers. Until then
 * both IDs are 0 and this file leaves the buffers alone.
 */
void mqtt_utk_set_buffers(int32_t tx_mbf, int32_t rx_mbf);

/*
 * 1 while connected to the broker, else 0. One word written only by the
 * WiFi service task, so any task may read it.
 */
int32_t mqtt_utk_connected(void);

#endif /* MQTT_UTK_H */