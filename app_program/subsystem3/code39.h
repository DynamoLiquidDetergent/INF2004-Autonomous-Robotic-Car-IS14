// types and function declarations

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

// Returned by code39_lookup() when a pattern does not match any letter.
// 0xFF because it isnt a printable ASCII character (letters A–Z are 0x41–0x5A)
#define CODE39_INVALID   0xFFu

// The 9-bit pattern for the '//' start/stop marker, scanned forward.
#define CODE39_STAR_FWD  0x0A4u

// The 9-bit pattern for '//' scanned backward (the exact bit-reversal of FWD).
#define CODE39_STAR_REV  0x04Au

typedef struct {
    // element width in GP28 sample counts (2 ms per sample)
    uint16_t w;
} BcElem;


// 512-entry lookup table that maps 9-bit barcode patterns to letters.
void code39_init(void);

// Looks up a 9-bit pattern in the lookup table and returns the corresponding letter.
uint8_t code39_lookup(uint16_t pattern9);

// Decodes a single Code 39 symbol from 29 GP28 samples (9 elements + 2 gaps).
uint8_t code39_decode_symbol(const BcElem elems[29],
                              uint16_t min_narrow_samples,
                              uint16_t wide_ratio_x10);

#endif
