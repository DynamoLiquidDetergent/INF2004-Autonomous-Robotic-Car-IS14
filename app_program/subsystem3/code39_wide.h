/*
 *	code39_wide.h
 *	Subsystem 3 -- Code 39 decoding from the WIDE BARS only.
 *
 *	Why: on the track barcode print, GP28 sees the wide bars clearly
 *	(peaks 1400-2800) but the narrow bars only as faint bumps (+50..+80
 *	over white), too weak to threshold.  The positions of the 6 wide bars
 *	of "*X*" (2 in each '*', 2 in every letter) still identify the letter:
 *
 *	  1. collect the centres of 6 consecutive wide-bar pulses
 *	  2. measure the wide/narrow ratio and the scale from the 4 wide bars
 *	     of the start and stop '*' (the same for every letter)
 *	  3. with that ratio, fit all 6 centres against every letter A-Z in
 *	     both scan directions; the best fit wins
 *	  4. 13 letters have a unique wide-bar pattern (G H I J Q R S T V W X
 *	     Y Z).  The others come in look-alike groups -- A/K/U, B/L, C/M,
 *	     D/N, E/O, F/P -- that differ only in which SPACE is wide.  For
 *	     those, the faint narrow bars break the tie: each candidate
 *	     predicts where its narrow bars are; the one whose predicted bars
 *	     read darkest against the white right beside them wins.  Too small
 *	     a lead is reported as "uncertain" and NOT sent.
 *
 *	Known limit: D and N differ only in a narrow bar that sits between
 *	two wide bars, where the white reference is spoiled by their blurred
 *	edges, so D and N are always reported "uncertain".  Avoid D and N as
 *	command letters on this print.
 *
 *	Sized for the 141 mm track barcode ("*X*", narrow ~3.0-3.7 mm, wide
 *	~7.4-9 mm) at 50-150 mm/s.  Tested on the PC (testing/subsystem3) with
 *	real GP28 captures rescaled to 141 mm and synthetic barcodes for all
 *	26 letters.
 *
 *	Pure computation: no kernel calls, so it can be tested on a PC.
 *	Uses float (software floating point on the Cortex-M0+); a decode
 *	runs only once per barcode.
 */

#ifndef CODE39_WIDE_H
#define CODE39_WIDE_H

#include <stdint.h>
#include <stdbool.h>

/* Timing, in 2 ms GP28 samples.  141 mm barcode:
     whole symbol      141 mm  = 1410 samples at 50 mm/s (705 at 100)
     wide bar          <= 9 mm (+ ~3 mm sensor blur) = 120 samples at 50 mm/s
     white between two wide bars <= 33 mm           = 334 samples at 50 mm/s */
#define C39W_BUF_LEN       2048u   /* raw samples kept (power of 2): 4.1 s
                                      = whole symbol down to ~35 mm/s      */
#define C39W_NWIDE         6u      /* wide bars in "*X*"                   */
#define C39W_MIN_PULSE     6u      /* shorter dark runs = noise (a wide bar
                                      at 150 mm/s is still >= 15)          */
#define C39W_MAX_PULSE     150u    /* longer = not a bar (floor, line);
                                      a wide bar at 50 mm/s is <= 120      */
#define C39W_MAX_GAP       400u    /* white between wide bars before
                                      starting a new symbol (<= 334 at
                                      50 mm/s)                             */
#define C39W_MAX_ERR_X100  35      /* fit error limit, 1/100 narrow unit   */
#define C39W_TIE_ERR_X100  10      /* letters this close in fit = tied     */
#define C39W_MIN_MARGIN    6       /* ADC counts a tie-break must win by   */

/* The 6 pulses must all be wide bars: none narrower than half the widest.
   On a print where the narrow bars ARE visible, every 6 consecutive pulses
   include narrow ones, so this decoder stays quiet and the normal
   29-element decoder (code39_decode_symbol) reads the barcode instead. */
#define C39W_WIDTH_SPREAD  2u

#define C39W_NONE          0xFFu

typedef struct {
    uint8_t  ch;          /* letter, or C39W_NONE                          */
    bool     uncertain;   /* tie-break lead below C39W_MIN_MARGIN          */
    uint8_t  reversed;    /* 1 = scanned stop-to-start                     */
    uint16_t err_x100;    /* fit error of the winner, 1/100 narrow unit    */
    uint16_t ratio_x100;  /* wide/narrow ratio of the best fit             */
    uint16_t unit_x10;    /* samples per narrow unit, x10                  */
    uint8_t  ntied;       /* letters in the tie group (1 = unique)         */
    uint8_t  ch2;         /* runner-up of the tie group, or C39W_NONE      */
    int16_t  contrast;    /* tie-break score of the winner (ADC)           */
    int16_t  contrast2;   /* ... and of the runner-up                      */
} C39W_RESULT;

typedef struct {
    uint16_t buf[C39W_BUF_LEN];   /* raw GP28, ring buffer                 */
    uint32_t idx;                 /* samples fed so far                    */
    bool     dark;
    uint32_t start;               /* first sample of the current pulse     */
    uint32_t last_end;
    uint32_t cx2[C39W_NWIDE];     /* pulse centres x 2 (start + end)       */
    uint16_t pw[C39W_NWIDE];      /* pulse widths, samples                 */
    uint8_t  n;
} C39W_STATE;

void c39w_reset(C39W_STATE *st);

/* Feed one sample (raw ADC + dark/light decision with hysteresis).
   Returns true when 6 wide bars were collected and a decode was tried;
   *res then holds the result (res->ch == C39W_NONE if nothing fitted). */
bool c39w_feed(C39W_STATE *st, uint16_t adc, bool dark, C39W_RESULT *res);

/* The decode step on its own (used by c39w_feed and the tests). */
void c39w_decode(const C39W_STATE *st, C39W_RESULT *res);

#endif /* CODE39_WIDE_H */
