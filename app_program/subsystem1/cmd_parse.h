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
 * Pure C: no kernel calls.  Each parser is a caller-owned cmd_parser_t.
 * Not thread-safe: only one task may use a given parser.
 */

#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stdbool.h>
#include <stdint.h>

#define CMD_FRAME_LEN  (1U)   /* bytes in a command frame (IF-15 RX) */

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
    CMD_ERR_CODE    /* not PAUSE, RESUME or ABORT        */
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

#endif /* CMD_PARSE_H */