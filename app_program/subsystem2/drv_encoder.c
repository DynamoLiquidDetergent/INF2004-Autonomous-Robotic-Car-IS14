/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoders and the shared IO_BANK0 GPIO interrupt.
 *
 *	Encoders (TEAM.md section 2):
 *	  left  A = GP14, B = GP15
 *	  right A = GP4,  B = GP5
 *
 *	Only the rising edge of channel A is counted.  The direction of each
 *	count comes from the commanded motor direction, because it is not
 *	yet confirmed that the encoders give A/B quadrature (assumption A2).
 *	If the bench test shows quadrature, set S2_ENC_QUADRATURE to 1 in
 *	subsystem2.h and channel B is read at each A edge to give direction.
 *
 *	The RP2040 has ONE interrupt (IO_IRQ_BANK0) for every GPIO, and a
 *	kernel interrupt number takes one handler.  This file owns that
 *	handler and passes edges on other pins to hooks registered with
 *	s2_gpio_hook_register() (TEAM.md section 3: S2 + S5 share it; S5
 *	can use a hook for the HC-SR04 echo on GP17).
 *
 *	Concurrency: the interrupt is enabled only on the core that runs the
 *	Motion Control task, so encoder_snapshot() can stop the ISR from
 *	running halfway through a read just by masking interrupts on that
 *	core (DI/EI).  Hooks registered from other tasks use the atomic
 *	SET alias of the INTE register, so they never read-modify-write a
 *	register the other core may also be changing.
 */

#include "drv_encoder.h"
#include "subsystem2.h"
#include <tk/syslib.h>		/* in_w/out_w, DI/EI, EnableInt */

/* ---------------------------------------------------------------------
 *  RP2040 IO_BANK0 interrupt registers (datasheet 2.19.6).  GPIO n uses
 *  4 bits in register n/8 at bit 4*(n%8): LEVEL_LOW, LEVEL_HIGH,
 *  EDGE_LOW, EDGE_HIGH.  Edge bits are cleared by writing 1 to INTR.
 * --------------------------------------------------------------------- */
#define ENC_INTNO_IO_BANK0	13U		/* IO_IRQ_BANK0 */
#define ENC_IRQ_LEVEL		2		/* NVIC priority level (0..3) */
#define ENC_IO_INTR(r)		(IO_BANK0_BASE + 0x0F0U + (4U * (r)))
#define ENC_IO_P0_INTE(r)	(IO_BANK0_BASE + 0x100U + (4U * (r)))
#define ENC_IO_P0_INTS(r)	(IO_BANK0_BASE + 0x120U + (4U * (r)))
#define ENC_IO_P1_INTE(r)	(IO_BANK0_BASE + 0x130U + (4U * (r)))
#define ENC_IO_P1_INTS(r)	(IO_BANK0_BASE + 0x150U + (4U * (r)))
#define ENC_ALIAS_SET		0x2000U		/* atomic bit-set alias offset */
#define ENC_ALIAS_CLR		0x3000U		/* atomic bit-clear alias offset */
#define ENC_INT_REGS		4U
#define ENC_GPIO_MAX		30U

#define ENC_REG_OF(gpio)	((gpio) / 8U)
#define ENC_SHIFT_OF(gpio)	(4U * ((gpio) % 8U))
#define ENC_EVENT_MASK		0xFU

/* RESETS block: TIMER must be out of reset for the microsecond clock */
#define ENC_RESETS_CLR		(RESETS_BASE + 0x3000U)

/* Timer: free-running 1 MHz counter, low word (same as knl_arch_time_us) */
#define ENC_TIMER_RAWL		TIMER_TIMERAWL

#define ENC_HOOK_MAX		4U

typedef struct {
	UINT		gpio;
	UINT		events;
	S2_GPIO_HOOK	cb;
} ENC_HOOK;

typedef struct {
	volatile W	count;
	volatile UW	last_edge_us;
	volatile UW	period_us;
	volatile UW	edges;
	volatile INT	dir;		/* +1 or -1, set by the task */
} ENC_STATE;

typedef struct {
	UINT	pin_a;
	UINT	pin_b;
	BOOL	invert;
} ENC_PINS;

LOCAL const ENC_PINS enc_pins[ENC_COUNT] = {
	{ S2_PIN_ENC_L_A, S2_PIN_ENC_L_B, S2_ENC_L_INVERT },
	{ S2_PIN_ENC_R_A, S2_PIN_ENC_R_B, S2_ENC_R_INVERT },
};

LOCAL ENC_STATE		enc_state[ENC_COUNT];
LOCAL ENC_HOOK		enc_hooks[ENC_HOOK_MAX];
LOCAL volatile UINT	enc_hook_count = 0U;
LOCAL volatile UINT	enc_irq_core = 0U;	/* 0 = not enabled yet, else 1 or 2 */

EXPORT UW encoder_time_us(void)
{
	return in_w(ENC_TIMER_RAWL);
}

LOCAL UW enc_inte_addr(UINT core, UINT reg)
{
	return (core == 2U) ? ENC_IO_P1_INTE(reg) : ENC_IO_P0_INTE(reg);
}

LOCAL UW enc_ints_addr(UINT core, UINT reg)
{
	return (core == 2U) ? ENC_IO_P1_INTS(reg) : ENC_IO_P0_INTS(reg);
}

LOCAL void enc_enable_pin_events(UINT core, UINT gpio, UINT events)
{
	UW	bits = ((UW)(events & ENC_EVENT_MASK)) << ENC_SHIFT_OF(gpio);
	UINT	reg  = ENC_REG_OF(gpio);

	out_w(ENC_IO_INTR(reg), bits);				/* drop stale edges */
	out_w(enc_inte_addr(core, reg) + ENC_ALIAS_SET, bits);	/* atomic set */
}

LOCAL void enc_config_input(UINT gpio)
{
	(void)gpio_set_pin(gpio, GPIO_MODE_IN);
#if S2_ENC_PULL_UP
	set_w(GPIO(gpio), GPIO_PUE);		/* pull-up for open-collector outputs */
	clr_w(GPIO(gpio), GPIO_PDE);
#endif
}

LOCAL INT enc_find_wheel(UINT gpio)
{
	UINT	w;

	for (w = 0U; w < ENC_COUNT; w++) {
		if (enc_pins[w].pin_a == gpio) {
			return (INT)w;
		}
	}
	return -1;
}

/* Count one accepted rising edge on channel A of a wheel */
LOCAL void enc_count_edge(UINT wheel, UW now_us)
{
	ENC_STATE	*st = &enc_state[wheel];
	UW		since = now_us - st->last_edge_us;
	INT		dir;

	if ((st->edges != 0U) && (since < S2_ENC_MIN_EDGE_US)) {
		return;				/* too soon: contact or optical noise */
	}

#if S2_ENC_QUADRATURE
	dir = (gpio_get_val(enc_pins[wheel].pin_b) != 0U) ? 1 : -1;
#else
	dir = st->dir;
#endif
	if (enc_pins[wheel].invert) {
		dir = -dir;
	}

	st->count += dir;
	st->period_us = (st->edges != 0U) ? since : 0U;
	st->last_edge_us = now_us;
	st->edges++;
}

/*
 * IO_IRQ_BANK0 handler.  Task-independent part: no waiting, no printing.
 */
LOCAL void enc_gpio_irq_handler(UINT intno)
{
	UINT	core = enc_irq_core;
	UINT	reg;
	UINT	gpio;
	UINT	h;
	UW	status;
	UW	events;
	UW	now = encoder_time_us();
	INT	wheel;

	(void)intno;

	for (reg = 0U; reg < ENC_INT_REGS; reg++) {
		status = in_w(enc_ints_addr(core, reg));
		if (status == 0U) {
			continue;
		}
		for (gpio = reg * 8U; (gpio < (reg * 8U) + 8U) && (gpio < ENC_GPIO_MAX); gpio++) {
			events = (status >> ENC_SHIFT_OF(gpio)) & ENC_EVENT_MASK;
			if (events == 0U) {
				continue;
			}
			/* Clear edge events (level events clear themselves) */
			out_w(ENC_IO_INTR(reg), events << ENC_SHIFT_OF(gpio));

			wheel = enc_find_wheel(gpio);
			if (wheel >= 0) {
				if ((events & S2_GPIO_EDGE_RISE) != 0U) {
					enc_count_edge((UINT)wheel, now);
				}
				continue;
			}
			for (h = 0U; h < enc_hook_count; h++) {
				if ((enc_hooks[h].gpio == gpio) && (enc_hooks[h].cb != NULL)) {
					enc_hooks[h].cb(gpio, (UINT)events, now);
				}
			}
		}
	}
}

EXPORT ER encoder_init(void)
{
	T_DINT	dint = {0};
	UINT	w;

	/* The microsecond timer must be running (it is for the kernel's own
	   statistics, but make sure without resetting it if it is). */
	if ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U) {
		out_w(ENC_RESETS_CLR, RESETS_RESET_TIMER);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0U) {
			/* wait */
		}
	}

	for (w = 0U; w < ENC_COUNT; w++) {
		enc_state[w].count = 0;
		enc_state[w].last_edge_us = 0U;
		enc_state[w].period_us = 0U;
		enc_state[w].edges = 0U;
		enc_state[w].dir = 1;
		enc_config_input(enc_pins[w].pin_a);
		enc_config_input(enc_pins[w].pin_b);
	}

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)enc_gpio_irq_handler;
	return tk_def_int(ENC_INTNO_IO_BANK0, &dint);
}

/*
 * Enable the GPIO interrupt on the CALLING core.  The Motion Control task
 * calls this once at start-up, so the ISR always runs on the same core
 * as the task that reads the counters.
 */
EXPORT ER encoder_enable_irq(void)
{
	UINT	core;
	UINT	w;
	UINT	h;

#if TK_SUPPORT_SMP
	core = (UINT)tk_get_prc();		/* 1 = core 0, 2 = core 1 */
#else
	core = 1U;
#endif
	enc_irq_core = core;

	for (w = 0U; w < ENC_COUNT; w++) {
		enc_enable_pin_events(core, enc_pins[w].pin_a, S2_GPIO_EDGE_RISE);
	}
	for (h = 0U; h < enc_hook_count; h++) {
		enc_enable_pin_events(core, enc_hooks[h].gpio, enc_hooks[h].events);
	}
	EnableInt(ENC_INTNO_IO_BANK0, ENC_IRQ_LEVEL);
	return E_OK;
}

EXPORT void encoder_snapshot(UINT wheel, ENC_SNAPSHOT *snap)
{
	UINT	imask;

	if ((wheel >= ENC_COUNT) || (snap == NULL)) {
		return;
	}
	DI(imask);
	snap->count        = enc_state[wheel].count;
	snap->last_edge_us = enc_state[wheel].last_edge_us;
	snap->period_us    = enc_state[wheel].period_us;
	snap->edges        = enc_state[wheel].edges;
	EI(imask);
}

EXPORT void encoder_set_direction(UINT wheel, INT dir)
{
	if (wheel < ENC_COUNT) {
		enc_state[wheel].dir = (dir < 0) ? -1 : 1;	/* one aligned store */
	}
}

/*
 * Shared GPIO interrupt hook (public, see subsystem2.h).
 * Register in subsystemN_init(), before tasks start, if possible.
 */
EXPORT ER s2_gpio_hook_register(UINT gpio, UINT events, S2_GPIO_HOOK cb)
{
	UINT	n;
	UINT	core;

	if ((gpio >= ENC_GPIO_MAX) || (cb == NULL) || ((events & ENC_EVENT_MASK) == 0U)) {
		return E_PAR;
	}
	if (enc_find_wheel(gpio) >= 0) {
		return E_OBJ;			/* encoder pins belong to subsystem 2 */
	}
	n = enc_hook_count;
	if (n >= ENC_HOOK_MAX) {
		return E_LIMIT;
	}
	enc_hooks[n].gpio   = gpio;
	enc_hooks[n].events = events & ENC_EVENT_MASK;
	enc_hooks[n].cb     = cb;
	__asm__ volatile ("dmb" ::: "memory");	/* entry visible before the count */
	enc_hook_count = n + 1U;		/* publish after the entry is filled */

	core = enc_irq_core;
	if (core != 0U) {			/* IRQ already live: enable this pin now */
		enc_enable_pin_events(core, gpio, events);
	}
	return E_OK;
}
