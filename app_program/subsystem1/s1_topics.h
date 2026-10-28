/*
 * s1_topics.h
 * MQTT topics and message-buffer message format for IF-15
 * (S1 <-> mqtt_utk.c).
 *
 * Plain C, no kernel headers, mqtt_utk.c can include it too.
 *
 * Every message in mbf_mqtt_tx / mbf_mqtt_rx is:
 *   byte 0     topic ID (below)
 *   bytes 1..  payload: a packed frame (TX) or the raw MQTT payload (RX)
 */

#ifndef S1_TOPICS_H
#define S1_TOPICS_H

/* TX topic IDs (S1 -> broker).  0-2 equal the frame header's type field. */
#define S1_TOPIC_TELEM       (0U)
#define S1_TOPIC_HB          (1U)
#define S1_TOPIC_EVT         (2U)
#define S1_TOPIC_ONLINE      (3U)

/* RX topic IDs (broker -> S1) */
#define S1_TOPIC_CMD         (0x80U)
#define S1_TOPIC_MAP         (0x81U)

/* Topic names */
#define S1_TOPIC_NAME_TELEM  "is14/car/telem"
#define S1_TOPIC_NAME_HB     "is14/car/hb"
#define S1_TOPIC_NAME_EVT    "is14/car/evt"
#define S1_TOPIC_NAME_ONLINE "is14/car/online"
#define S1_TOPIC_NAME_CMD    "is14/car/cmd"
#define S1_TOPIC_NAME_MAP    "is14/car/map"

/* Largest message in each buffer, bytes (topic ID + payload) */
#define S1_TX_MSG_MAX        (1U + 15U)   /* + largest frame (telemetry) */
#define S1_RX_MSG_MAX        (1U + 3U)    /* + largest payload (map)     */

#endif /* S1_TOPICS_H */