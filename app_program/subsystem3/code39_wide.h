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
 *	     both scan directions.  Every letter+direction that fits almost as
 *	     well as the best one is kept (a letter scanned backwards can have
 *	     the same wide bars as another scanned forwards: N back = D fwd)
 *	  4. look-alikes (A/K/U, B/L, C/M, D/N, E/O, F/P, G/Z/T, ...) differ
 *	     only in where the narrow bars are.  GP28's spot is wider than a
 *	     narrow bar, so the narrow bars never show as separate bumps above
 *	     white: they only lift the reading a little where they are
 *	     crowded together (seen on the robot: the reading drifts 205..330
 *	     between the wide bars with no bump per bar).  So each candidate
 *	     predicts that BLURRED lift -- how much of its narrow bars falls in
 *	     a window C39W_TPL_W_X10 / 10 narrow units wide around each sample
 *	     -- and is fitted to the real readings by least squares, away from
 *	     the wide bars.  The letter that leaves the smallest error wins.
 *
 *	  5. the read is SENT only if the runner-up (best OTHER letter) leaves
 *	     clearly more error than the winner (C39W_MIN_EVIDENCE), the
 *	     winner's narrow bars add at least C39W_MIN_NARROW counts, and
 *	     enough samples were used.  Otherwise "uncertain", not sent.
 *
 *	Wide bars in pairs ("W n W") are split even when the blur fills the
 *	narrow space between them (C39W_SPLIT_PCT).
 *
 *	Sized for the 141 mm track barcode ("*X*", narrow ~3.0-3.7 mm, wide
 *	~7.4-9 mm) at 50-150 mm/s.  Tuned on a real GP28 capture of *Z* and
 *	on synthetic traces of all 26 letters, both directions, 50/100/150
 *	mm/s, with sharp and blurred narrow bars.  Re-check with a capture
 *	(S3_BC_CAPTURE) if the sensor height or print changes.
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
     white between two wide bars <= 51 mm ('C')     = 510 samples at 50 mm/s */
#define C39W_BUF_LEN       2048u   /* raw samples kept (power of 2): 4.1 s
                                      = whole symbol down to ~35 mm/s      */
#define C39W_NWIDE         6u      /* wide bars in "*X*"                   */
#define C39W_MIN_PULSE     6u      /* shorter dark runs = noise (a wide bar
                                      at 150 mm/s is still >= 15)          */
#define C39W_MAX_PULSE     150u    /* longer = not a bar (floor, line);
                                      a wide bar at 50 mm/s is <= 120      */
#define C39W_MAX_GAP       650u    /* white between wide bars before
                                      starting a new symbol.  Worst case is
                                      'C': ~51 mm between its last wide bar
                                      and the stop '*' (+ ~5 mm because a
                                      pulse is only counted above 840) =
                                      ~560 samples at 50 mm/s.  (Was 400:
                                      C E F M O P W never read at 50 mm/s.) */
/* Splitting two wide bars whose narrow space is filled in by blur
   (measured on the track print: peaks ~2200-2400, dip between them only
   ~650-760).  A pulse ends below C39W_SPLIT_PCT % of its peak; the next
   starts once the reading is C39W_SPLIT_RISE above the dip.  Inside a
   single wide bar the reading stays above ~45 % of its peak. */
#define C39W_SPLIT_PCT     40u
#define C39W_SPLIT_RISE    150u
#define C39W_MAX_ERR_X100  35      /* fit error limit, 1/100 narrow unit   */
#define C39W_TIE_ERR_X100  35      /* letters this close in fit are all
                                      compared by the tie-break (blur moves
                                      wide-bar centres, so the right letter
                                      is not always the best fit)          */
#define C39W_MAX_TIED      8u      /* letter+direction pairs compared      */

/* ---- Tie-break between look-alikes (blurred narrow-bar template) ----
   Tuned on a real capture of the 141 mm *Z* at 100 mm/s (Z error 12 vs
   T 19, G 21 ADC rms) and on synthetic traces of all 26 letters. */
#define C39W_TPL_W_X10        30   /* template window, narrow units x10    */
#define C39W_TPL_EXCL_X10     13   /* skip samples this close to a wide bar */
#define C39W_TPL_MARGIN       4    /* narrow units used either side of the
                                      symbol (quiet zone)                  */
#define C39W_TPL_MIN_UNITS    8    /* need samples covering this many units */

/* Send only if the runner-up leaves at least C39W_MIN_EVIDENCE "noise
   units" more squared error than the winner (n x (err2 - err1) / err1;
   real *Z*: ~340), and the winner's narrow bars lift the reading by at
   least C39W_MIN_NARROW ADC counts (real *Z*: ~100). */
#define C39W_MIN_EVIDENCE        100
#define C39W_MIN_NARROW          20

/* The 6 pulses must all be wide bars: none narrower than half the widest.
   On a print where the narrow bars ARE visible, every 6 consecutive pulses
   include narrow ones, so this decoder stays quiet and the normal
   29-element decoder (code39_decode_symbol) reads the barcode instead. */
#define C39W_WIDTH_SPREAD  2u

#define C39W_NONE          0xFFu

typedef struct {
    uint8_t  ch;          /* letter, or C39W_NONE                          */
    bool     uncertain;   /* not clear-cut (step 5 above): do not send     */
    uint8_t  reversed;    /* 1 = scanned stop-to-start                     */
    uint16_t err_x100;    /* fit error of the winner, 1/100 narrow unit    */
    uint16_t ratio_x100;  /* wide/narrow ratio of the best fit             */
    uint16_t unit_x10;    /* samples per narrow unit, x10                  */
    uint8_t  ntied;       /* different letters in the tie group (1 = unique) */
    uint8_t  ch2;         /* runner-up of the tie group, or C39W_NONE      */
    uint16_t rms_x10;     /* tie-break: winner's error, ADC rms x10        */
    uint16_t rms2_x10;    /* ... runner-up's error                         */
    int16_t  narrow;      /* tie-break: ADC counts the winner's narrow bars
                             add (should be clearly positive)              */
    uint16_t nsamp;       /* tie-break: samples used                       */
    uint16_t evidence;    /* tie-break: how clearly the winner beat ch2    */
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
    uint16_t peak;                /* highest reading of the current pulse  */
    uint16_t valley;              /* lowest reading since the last pulse   */
    uint32_t allow;               /* letters that can appear: bit 0 = 'A';
                                     0 = any letter                        */
} C39W_STATE;

/* Only these letters can be on the track (bit 0 = 'A' ... bit 25 = 'Z';
   0 = any letter).  With the 4 mapped letters, look-alikes of a letter
   that are NOT on the track are never considered, so most reads are
   decided by the wide bars alone. */
void c39w_set_letters(C39W_STATE *st, uint32_t allow);

void c39w_reset(C39W_STATE *st);

/* Feed one sample (raw ADC + dark/light decision with hysteresis).
   Returns true when 6 wide bars were collected and a decode was tried;
   *res then holds the result (res->ch == C39W_NONE if nothing fitted). */
bool c39w_feed(C39W_STATE *st, uint16_t adc, bool dark, C39W_RESULT *res);

/* The decode step on its own (used by c39w_feed and the tests). */
void c39w_decode(const C39W_STATE *st, C39W_RESULT *res);

#endif /* CODE39_WIDE_H */
