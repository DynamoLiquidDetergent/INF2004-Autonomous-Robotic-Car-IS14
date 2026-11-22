// code39.h -- Code 39 lookup table and symbol decoder (subsystem 3)

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

// Returned by code39_lookup() when a pattern does not match any letter.
// 0xFF because it isn't a printable ASCII character (letters A–Z are 0x41–0x5A)
#define CODE39_INVALID 0xFFu

// 9-bit pattern for the '*' start/stop marker, scanned forward.
// Bit 8 = first element (a bar), 1 = wide:  n W n n W n W n n  = 0 1 0 0 1 0 1 0 0
#define CODE39_STAR_FWD 0x094u

// The same marker scanned backward (exact bit-reversal of FWD): n n W n W n n W n.
// Note: this is also the forward pattern of 'P', so direction must be decided
// from the start marker, never from a single character on its own.
#define CODE39_STAR_REV 0x052u

typedef struct
{
    // element width in GP28 sample counts (2 ms per sample)
    uint16_t w;
} BcElem;

// Fill the 512-entry lookup table that maps 9-bit barcode patterns to characters.
void code39_init(void);

// Look up a 9-bit pattern and return its character, or CODE39_INVALID.
uint8_t code39_lookup(uint16_t pattern9);

// Decode one Code 39 symbol from 29 element widths:
// start '*' (9) + gap (1) + data character (9) + gap (1) + stop '*' (9).
// Works in either scan direction.  Returns the data character or CODE39_INVALID.
uint8_t code39_decode_symbol(const BcElem elems[29],
                             uint16_t min_narrow_samples,
                             uint16_t wide_ratio_x10);

#endif