/*
 * cmd_parse.c
 * Validates 1-byte command frames (see cmd_parse.h for format).
 * Only PAUSE, RESUME & ABORT get through.
 */

#include <stddef.h>
#include "cmd_parse.h"

#define CMD_REJECTED_MAX  (0xFFFFU)

static cmd_result_t cmd_reject(cmd_parser_t *p_parser, cmd_result_t why);

/*
 * Start a parser with reject count of 0.
 */
void cmd_parse_init(cmd_parser_t *p_parser)
{
    if (p_parser != NULL)
    {
        p_parser->rejected = 0U;
    }
}

/*
 * Check one frame.  On CMD_OK, *p_cmd is PAUSE, RESUME or ABORT.
 * On any error, *p_cmd is CMD_NONE and the reject count goes up.
 */
cmd_result_t cmd_parse(cmd_parser_t *p_parser,
                       const uint8_t *p_frame, uint32_t len,
                       cmd_code_t *p_cmd)
{
    if (p_parser == NULL)
    {
        return CMD_ERR_NULL;
    }

    if (p_cmd == NULL)
    {
        return cmd_reject(p_parser, CMD_ERR_NULL);
    }

    *p_cmd = CMD_NONE;

    if (p_frame == NULL)
    {
        return cmd_reject(p_parser, CMD_ERR_NULL);
    }

    if (len != CMD_FRAME_LEN)
    {
        return cmd_reject(p_parser, CMD_ERR_LEN);
    }

    switch (p_frame[0])
    {
        case (uint8_t)CMD_PAUSE:
            *p_cmd = CMD_PAUSE;
            break;

        case (uint8_t)CMD_RESUME:
            *p_cmd = CMD_RESUME;
            break;

        case (uint8_t)CMD_ABORT:
            *p_cmd = CMD_ABORT;
            break;

        default:
            return cmd_reject(p_parser, CMD_ERR_CODE);
    }

    return CMD_OK;
}

/*
 * Number of frames rejected since init (saturates at 65535).
 */
uint16_t cmd_parse_rejected(const cmd_parser_t *p_parser)
{
    return (p_parser == NULL) ? 0U : p_parser->rejected;
}

/*
 * Count a rejected frame & pass the reason back.
 */
static cmd_result_t cmd_reject(cmd_parser_t *p_parser, cmd_result_t why)
{
    if (p_parser->rejected < CMD_REJECTED_MAX)
    {
        p_parser->rejected++;
    }
    return why;
}