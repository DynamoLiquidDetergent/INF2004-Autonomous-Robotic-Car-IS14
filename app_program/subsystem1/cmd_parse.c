/*
 * cmd_parse.c
 * Validates 1-byte command frames and 3-byte map frames (see cmd_parse.h
 * for formats).  Only PAUSE, RESUME & ABORT get through, and only a
 * complete, unambiguous map sent while READY.
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
 * Check one map frame (format in cmd_parse.h).  On CMD_OK, *p_map_word
 * holds the four letter codes plus the valid bit, ready to store.  On any
 * error, *p_map_word is 0 and the reject count goes up.  Checks run in
 * this order: length, spare bits, letters, duplicates, then READY.
 */
cmd_result_t cmd_parse_map(cmd_parser_t *p_parser,
                           const uint8_t *p_frame, uint32_t len,
                           bool b_ready, uint32_t *p_map_word)
{
    uint32_t raw;
    uint8_t  code[CMD_MAP_MOVES];
    uint8_t  i;
    uint8_t  j;

    if (p_parser == NULL)
    {
        return CMD_ERR_NULL;
    }

    if (p_map_word == NULL)
    {
        return cmd_reject(p_parser, CMD_ERR_NULL);
    }

    *p_map_word = 0UL;

    if (p_frame == NULL)
    {
        return cmd_reject(p_parser, CMD_ERR_NULL);
    }

    if (len != CMD_MAP_FRAME_LEN)
    {
        return cmd_reject(p_parser, CMD_ERR_LEN);
    }

    raw = (uint32_t)p_frame[0] |
          ((uint32_t)p_frame[1] << 8) |
          ((uint32_t)p_frame[2] << 16);

    if ((raw & CMD_MAP_SPARE_MASK) != 0UL)
    {
        return cmd_reject(p_parser, CMD_ERR_RESERVED);
    }

    for (i = 0U; i < CMD_MAP_MOVES; i++)
    {
        code[i] = (uint8_t)((raw >> (i * CMD_MAP_CODE_BITS)) &
                            CMD_MAP_CODE_MASK);
        if (code[i] > CMD_MAP_LETTER_MAX)
        {
            return cmd_reject(p_parser, CMD_ERR_LETTER);
        }
    }

    for (i = 0U; i < CMD_MAP_MOVES; i++)
    {
        for (j = (uint8_t)(i + 1U); j < CMD_MAP_MOVES; j++)
        {
            if (code[i] == code[j])
            {
                return cmd_reject(p_parser, CMD_ERR_DUPLICATE);
            }
        }
    }

    if (!b_ready)
    {
        return cmd_reject(p_parser, CMD_ERR_STATE);
    }

    *p_map_word = (raw & CMD_MAP_FIELDS_MASK) | CMD_MAP_VALID;

    return CMD_OK;
}

/*
 * MAP_ACK reason code (2 bits) for a rejected map.
 */
uint8_t cmd_map_reason(cmd_result_t result)
{
    uint8_t reason;

    switch (result)
    {
        case CMD_ERR_LETTER:
            reason = CMD_MAP_REASON_LETTER;
            break;

        case CMD_ERR_DUPLICATE:
            reason = CMD_MAP_REASON_DUPLICATE;
            break;

        case CMD_ERR_STATE:
            reason = CMD_MAP_REASON_STATE;
            break;

        default:
            reason = CMD_MAP_REASON_FORMAT;
            break;
    }
    return reason;
}

/*
 * Find which movement a letter (0 = A .. 25 = Z) is mapped to.
 * Returns false, leaving *p_move unchanged, if the map is not valid or
 * the letter is not in it.
 */
bool cmd_map_lookup(uint32_t map_word, uint8_t letter, uint8_t *p_move)
{
    uint8_t i;

    if ((p_move == NULL) || ((map_word & CMD_MAP_VALID) == 0UL))
    {
        return false;
    }

    for (i = 0U; i < CMD_MAP_MOVES; i++)
    {
        if (((map_word >> (i * CMD_MAP_CODE_BITS)) & CMD_MAP_CODE_MASK) ==
            (uint32_t)letter)
        {
            *p_move = i;
            return true;
        }
    }
    return false;
}

/*
 * Number of frames rejected since init, commands and maps together
 * (saturates at 65535).
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