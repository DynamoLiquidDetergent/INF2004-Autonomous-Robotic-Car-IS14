/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects and starts its tasks.
 *
 *	Two parts, see subsystem2.h:
 *	  A. Motion Control (motion_control.c, drv_motor.c, drv_encoder.c)
 *	  B. The template's example producer, kept running unchanged because
 *	     subsystems 3 and 4 still use sem_slots / mpf_samples (IF-01/02).
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf (example), tk_cre_mbf, tk_cre_cyc,
 *	            tk_def_int (motion)
 */

#include "subsystem2.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

LOCAL ER example_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}

EXPORT ER subsystem2_init(void)
{
	ER	er;

	er = example_init();
	if (er < E_OK) {
		return er;
	}
	return motion_init();		/* q_motion_cmd, cyclic handler, GPIO IRQ */
}

EXPORT ER subsystem2_start(void)
{
	ID	tid;

#if S2_KEEP_EXAMPLE
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
			     "producer_task");
	if (tid < E_OK) {
		return tid;
	}
#else
	(void)tid;
#endif
	return motion_start();
}
