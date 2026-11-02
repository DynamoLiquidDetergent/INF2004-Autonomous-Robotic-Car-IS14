/*
 * cmd_parse.h
 * Command parser for subsystem 1 (Network Gatekeeper).
 *
 * Checks each command that arrives over MQTT (IF-15 RX) and lets through
 * only PAUSE, RESUME and ABORT (FR7, FR8).  Anything else is rejected
 * and counted.
 *
 * Frame: exactly 1 byte (IF-15 RX)
 *        1 PAUSE, 2 RESUME, 3 ABORT.  Every other value is rejected.
 *
 * The values match IF-14 bits[1:0], so an accepted command can be
 * forwarded to S5 unchanged.
 *
 * Also checks the letter-to-movement map (map topic, IF-18), which the
 * instructor sends from the PC before the run.
 *
 * Map frame: exactly 3 bytes, read as one 24-bit value with byte 0 as
 * bits 0-7 (little-endian).  Four 5-bit letter codes, 0 = A .. 25 = Z:
 *   bits  0-4   FORWARD
 *   bits  5-9   U-TURN
 *   bits 10-14  LEFT
 *   bits 15-19  RIGHT
 *   bits 20-23  must be 0
 * Accepted only if every code is 25 or less, all four letters differ,
 * the spare bits are 0, and the car is in READY.
 *
 * An accepted map becomes one 32-bit word: bits 0-19 as above, bit 20 =
 * valid.  S1 stores it with a single write (IF-18, Change Requirement 1).
 * The movement order (0 FORWARD, 1 U-TURN, 2 LEFT, 3 RIGHT) is the same
 * as the telemetry "last action" field.
 *
 * Pure C: no kernel calls.  Each parser is a caller-owned cmd_parser_t.
 * Not thread-safe: only one task may use a given parser.
 */

#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stdbool.h>
#include <stdint.h>

#define CMD_FRAME_LEN  (1U)   /* bytes in a command frame (IF-15 RX) */

/* Map frame and stored map word (IF-18) */
#define CMD_MAP_FRAME_LEN    (3U)          /* bytes                      */
#define CMD_MAP_MOVES        (4U)          /* FORWARD, U-TURN, LEFT, RIGHT */
#define CMD_MAP_CODE_BITS    (5U)          /* bits per letter code       */
#define CMD_MAP_CODE_MASK    (0x1FUL)
#define CMD_MAP_LETTER_MAX   (25U)         /* 'Z'                        */
#define CMD_MAP_FIELDS_MASK  (0x000FFFFFUL) /* bits 0-19                  */
#define CMD_MAP_SPARE_MASK   (0x00F00000UL) /* bits 20-23 of the frame    */
#define CMD_MAP_VALID        (0x00100000UL) /* bit 20 of the stored word  */

#define CMD_MOVE_FORWARD     (0U)
#define CMD_MOVE_UTURN       (1U)
#define CMD_MOVE_LEFT        (2U)
#define CMD_MOVE_RIGHT       (3U)

/* MAP_ACK event reason (2 bits), meaningful only when the map is rejected */
#define CMD_MAP_REASON_FORMAT    (0U)   /* wrong length or spare bits set */
#define CMD_MAP_REASON_LETTER    (1U)   /* a code past 'Z'                */
#define CMD_MAP_REASON_DUPLICATE (2U)   /* the same letter used twice     */
#define CMD_MAP_REASON_STATE     (3U)   /* car not in READY               */

typedef enum
{
    CMD_NONE   = 0,
    CMD_PAUSE  = 1,
    CMD_RESUME = 2,
    CMD_ABORT  = 3
} cmd_code_t;

typedef enum
{
    CMD_OK = 0,
    CMD_ERR_NULL,   /* NULL pointer                      */
    CMD_ERR_LEN,    /* frame is not CMD_FRAME_LEN bytes  */
    CMD_ERR_CODE,       /* not PAUSE, RESUME or ABORT        */
    CMD_ERR_RESERVED,   /* map: a spare bit is set           */
    CMD_ERR_LETTER,     /* map: a letter code past 'Z'       */
    CMD_ERR_DUPLICATE,  /* map: the same letter used twice   */
    CMD_ERR_STATE       /* map: car not in READY             */
} cmd_result_t;

typedef struct
{
    uint16_t rejected;   /* frames rejected (saturates at 65535) */
} cmd_parser_t;

void         cmd_parse_init(cmd_parser_t *p_parser);
cmd_result_t cmd_parse(cmd_parser_t *p_parser,
                       const uint8_t *p_frame, uint32_t len,
                       cmd_code_t *p_cmd);
uint16_t     cmd_parse_rejected(const cmd_parser_t *p_parser);

cmd_result_t cmd_parse_map(cmd_parser_t *p_parser,
                           const uint8_t *p_frame, uint32_t len,
                           bool b_ready, uint32_t *p_map_word);
uint8_t      cmd_map_reason(cmd_result_t result);
bool         cmd_map_lookup(uint32_t map_word, uint8_t letter,
                            uint8_t *p_move);

#endif /* CMD_PARSE_H */