/*
 *	subsystem3.c
 *	Subsystem 3 -- IR Perception: creates its kernel objects and starts its tasks.
 *
 *	APIs used: tk_cre_mbf, tk_cre_mbx, tk_cre_mtx
 */

#include "subsystem3.h"
#include "code39.h" /* code39_init() -- Code 39 lookup table */

/* ---- S3 interface objects (declared IMPORT in subsystem3.h) ---- */
EXPORT ID q_line_data;    /* IF-05 */
EXPORT ID q_gp28_samples; /* IF-06 */
EXPORT ID q_decoded_char; /* IF-07 */
EXPORT ID q_barcode_cmd;  /* IF-07 */
EXPORT ID mbx_char_map;   /* MQTT char map */

/* ---- Legacy template objects (still used by S2 and S4) ---- */
EXPORT ID mbx_samples;
EXPORT ID mtx_stats;
EXPORT APP_STATS app_stats;

/* Each message in a message buffer takes a 4-byte header plus the
   message rounded up to 4 bytes. */
#define MBF_SLOT(sz) ((((sz) + 3) & ~3) + 4)

LOCAL ID make_mbf(INT msgsz, INT depth)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = msgsz;
    cmbf.bufsz = depth * MBF_SLOT(msgsz);
    return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem3_init(void)
{
    T_CMBX cmbx = {0};
    T_CMTX cmtx = {0};

    /* Fill the 512-entry Code 39 lookup table before anything can decode.
       Pure computation: no kernel objects, cannot fail. */
    code39_init();

    /* S3 queues */
    q_line_data = make_mbf(sizeof(LINE_DATA_MSG), 8);
    if (q_line_data < E_OK)
        return q_line_data;

    q_gp28_samples = make_mbf(sizeof(GP28_SAMPLE_MSG), 64);
    if (q_gp28_samples < E_OK)
        return q_gp28_samples;

    q_decoded_char = make_mbf(sizeof(uint8_t), 4);
    if (q_decoded_char < E_OK)
        return q_decoded_char;

    q_barcode_cmd = make_mbf(sizeof(BARCODE_CMD), 4);
    if (q_barcode_cmd < E_OK)
        return q_barcode_cmd;

    mbx_char_map = make_mbf(sizeof(CHAR_MAP_MSG), 2);
    if (mbx_char_map < E_OK)
        return mbx_char_map;

    /* Legacy: mailbox that S2's producer sends sample pointers into */
    cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
    mbx_samples = tk_cre_mbx(&cmbx);
    if (mbx_samples < E_OK)
        return mbx_samples;

    /* Legacy: mutex guarding app_stats (read by S4) */
    cmtx.mtxatr = TA_INHERIT;
    mtx_stats = tk_cre_mtx(&cmtx);
    if (mtx_stats < E_OK)
        return mtx_stats;

    return E_OK;
}

EXPORT ER subsystem3_start(void)
{
    /* Legacy example consumer, keeps S2/S4 working until they are replaced */
    ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, 0, "consumer_task");
    return (tid < E_OK) ? tid : E_OK;
}