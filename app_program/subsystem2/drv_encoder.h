/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoder driver and the shared GPIO interrupt.
 *
 *	Private to subsystem 2 except for the GPIO hook, which is declared in
 *	subsystem2.h because subsystem 5 (HC-SR04 echo on GP17) needs it.
 *
 *	The encoder ISR only counts pulses and timestamps them.  It never
 *	wakes a task and never runs the PID (Week 6 feedback, Tribunal 1).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "../app.h"

#define ENC_LEFT		0U
#define ENC_RIGHT		1U
#define ENC_COUNT		2U

/* One consistent copy of an encoder's state, taken with interrupts off */
typedef struct {
	W	count;		/* signed pulse count since boot */
	UW	last_edge_us;	/* time of the last accepted edge */
	UW	period_us;	/* time between the last two edges, 0 = none yet */
	UW	edges;		/* accepted edges since boot (unsigned, for NFR13) */
} ENC_SNAPSHOT;

IMPORT ER	encoder_init(void);
IMPORT ER	encoder_enable_irq(void);	/* call from the task that owns the IRQ core */
IMPORT void	encoder_snapshot(UINT wheel, ENC_SNAPSHOT *snap);
IMPORT void	encoder_set_direction(UINT wheel, INT dir);
IMPORT UW	encoder_time_us(void);

#endif /* DRV_ENCODER_H */
