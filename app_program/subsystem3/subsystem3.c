/*
 *	subsystem3.c
 *	Subsystem 3 -- IR Perception: creates its kernel objects and starts its tasks.
 *
 *	Two-phase startup (app_main.c / TEAM.md):
 *	  subsystem3_init()   – called first for EVERY subsystem before any
 *	                        task is started; creates all kernel objects
 *	                        and initialises hardware / lookup tables.
 *	  subsystem3_start()  – called after all _init() functions; tasks
 *	                        are already running (started inside the
 *	                        task-init helpers), so this just returns E_OK.
 *
 *	APIs used: tk_cre_mbf  (queues created in line_sensor_task_init)
 *	           tk_cre_tsk, tk_sta_tsk  (tasks created in task-init helpers)
 */

#include "subsystem3.h"
#include "code39.h"

/* -------------------------------------------------------------------------
 *  Forward declarations for the per-file init helpers.
 *  Each helper creates its own queues and starts its task.
 * ------------------------------------------------------------------------- */
void line_sensor_task_init(void);     /* defined in line_sensor.c   */
void barcode_decode_task_init(void);  /* defined in barcode_decode.c */

/* =========================================================================
 *  subsystem3_init
 *
 *  Called by usermain() BEFORE any task is started (phase 1).
 *  Creates all kernel objects owned by S3 and initialises the Code 39
 *  lookup table.  Every queue and task used by S3 is set up here so that
 *  IDs are valid the moment other subsystems call _start().
 *
 *  Returns E_OK on success, or the first negative error code on failure.
 * ========================================================================= */
EXPORT ER subsystem3_init(void)
{
    /* Initialise the Code 39 symbol lookup table (code39.c).
       Must run before barcode_decode_task reads any samples. */
    code39_init();

    /* Create the message queues and spawn the line sensor task.
       Queues created here: q_line_data (IF-05), q_gp28_samples (IF-06),
       q_decoded_char, q_barcode_cmd (IF-07), mbx_char_map (MQTT map). */
    line_sensor_task_init();

    /* Create the barcode decode task.
       Reads from q_gp28_samples (IF-06), writes to q_decoded_char and
       q_barcode_cmd (IF-07), polls mbx_char_map for mapping updates. */
    barcode_decode_task_init();

    return E_OK;
}

/* =========================================================================
 *  subsystem3_start
 *
 *  Called by usermain() AFTER all _init() functions (phase 2).
 *  Both S3 tasks (line_sensor_task, barcode_decode_task) are already
 *  running — they were started inside their respective _init helpers —
 *  so nothing more is needed here.
 * ========================================================================= */
EXPORT ER subsystem3_start(void)
{
    /* Tasks started inside line_sensor_task_init() and
       barcode_decode_task_init() during phase 1 — nothing to do. */
    return E_OK;
}
