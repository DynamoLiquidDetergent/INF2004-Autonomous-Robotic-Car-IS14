#include "code39.h"   
#include <string.h>                
#include <stdbool.h>                                

// Look up a decoded letter in the current character-to-movement mapping and return the corresponding movement command
static uint8_t s_lut[512];

// Lookup table for Code 39 patterns. Each entry maps a 9-bit pattern to a letter. The table is filled in code39_init() at startup
typedef struct {
    uint8_t  ch;   // the letter this pattern represents 
    uint16_t pat;  // the 9-bit narrow/wide pattern       
} Code39Entry;

// Table of Code 39 patterns for letters A–Z and the start/stop marker
static const Code39Entry k_code39[] = {

    // The '*' start/stop marker. not a data character but needs to be recognised so we know where a barcode begins and ends. 
    { '*', 0b010100100 },   // 0x0A4 

    // Letters A–Z
    { 'A', 0b100001001 },   // 0x109  W N N N N W N N W 
    { 'B', 0b001001001 },   // 0x049  N N W N N W N N W 
    { 'C', 0b101001000 },   // 0x148  W N W N N W N N N 
    { 'D', 0b000011001 },   // 0x019  N N N N W W N N W 
    { 'E', 0b100011000 },   // 0x118  W N N N W W N N N 
    { 'F', 0b001011000 },   // 0x058  N N W N W W N N N 
    { 'G', 0b000001101 },   // 0x00D  N N N N N W W N W 
    { 'H', 0b100001100 },   // 0x10C  W N N N N W W N N 
    { 'I', 0b001001100 },   // 0x04C  N N W N N W W N N 
    { 'J', 0b000011100 },   // 0x01C  N N N N W W W N N 
    { 'K', 0b100000011 },   // 0x103  W N N N N N N W W 
    { 'L', 0b001000011 },   // 0x043  N N W N N N N W W 
    { 'M', 0b101000010 },   // 0x142  W N W N N N N W N 
    { 'N', 0b000010011 },   // 0x013  N N N N W N N W W 
    { 'O', 0b100010010 },   // 0x112  W N N N W N N W N 
    { 'P', 0b001010010 },   // 0x052  N N W N W N N W N 
    { 'Q', 0b000000111 },   // 0x007  N N N N N N W W W 
    { 'R', 0b100000110 },   // 0x106  W N N N N N W W N 
    { 'S', 0b001000110 },   // 0x046  N N W N N N W W N 
    { 'T', 0b000010110 },   // 0x016  N N N N W N W W N 
    { 'U', 0b110000001 },   // 0x181  W W N N N N N N W 
    { 'V', 0b011000001 },   // 0x0C1  N W W N N N N N W 
    { 'W', 0b111000000 },   // 0x1C0  W W W N N N N N N 
    { 'X', 0b010010001 },   // 0x091  N W N N W N N N W 
    { 'Y', 0b110010000 },   // 0x190  W W N N W N N N N 
    { 'Z', 0b011010000 },   // 0x0D0  N W W N W N N N N 
};

// Number of entries in the pattern table
#define N_ENTRIES ((int)(sizeof(k_code39) / sizeof(k_code39[0])))

// The forward and reverse patterns for the start/stop marker. The forward pattern is 0x0A4, and the reverse pattern is 0x04A.
#define CODE39_STAR_FWD_LOCAL  0x0A4u
#define CODE39_STAR_REV_LOCAL  0x04Au


// Reverse the bits of a 9-bit value. For example, 0b010100100 becomes 0b001001010.
static uint16_t reverse9(uint16_t v)
{
    uint16_t r = 0u;  // r will hold the reversed result, starts at 0 

    // Loop 9 times, once per bit 
    for (int i = 0; i < 9; i++) {
        r = (uint16_t)((r << 1u) | (v & 1u)); // Shift r left and add the least significant bit of v to r
        v >>= 1u;                             // Shift v right to process the next bit in the next iteration
    }
    return r;
}


// Fill the lookup table with the patterns for letters A–Z and the start/stop marker
void code39_init(void)
{
    // fill with invalid marker so any unassigned pattern returns CODE39_INVALID
    memset(s_lut, CODE39_INVALID, sizeof(s_lut));

    // iterate through every letter in the pattern table and store it 
    for (int i = 0; i < N_ENTRIES; i++) {

        // Mask to 9 bits to not write outside the array 
        uint16_t fwd = k_code39[i].pat & 0x1FFu;

        // Store the letter at the index matching its forward pattern
        s_lut[fwd] = k_code39[i].ch;
    }
}


// look up a forward 9-bit pattern in the lookup table and return the corresponding letter
uint8_t code39_lookup(uint16_t pattern9)
{
    // If the pattern is out of bounds, return CODE39_INVALID
    if (pattern9 >= 512u) return CODE39_INVALID;

    return s_lut[pattern9];
}


// Bidirectional Code 39 decoder. Takes 29 GP28 samples and decodes them into a letter A–Z
uint8_t code39_decode_symbol(const BcElem elems[29],
                              uint16_t min_narrow_samples,
                              uint16_t wide_ratio_x10)
{
    // find the narrowest element width in the 29-element array. This is used to determine the threshold for wide vs narrow elements.
    uint16_t min_w = elems[0].w;                     // start with the first element's width 
    for (int i = 1; i < 29; i++) {
        if (elems[i].w < min_w) min_w = elems[i].w;  // update if we find something narrower 
    }

    // if the narrowest element is narrower than the minimum allowed, reject the reading as invalid
    if (min_w < min_narrow_samples) return CODE39_INVALID;

    // calculate the threshold for wide elements. Any element wider than this is considered wide, otherwise narrow.
    uint32_t wide_thresh = ((uint32_t)min_w * wide_ratio_x10) / 10u;

    // convert the 29 elements into three 9-bit patterns: start, data, and stop. 
    uint16_t start_pat = 0u;   // 9-bit pattern for elements [0 - 8]  = start '//' 
    uint16_t data_pat  = 0u;   // 9-bit pattern for elements [10 - 18] = data letter 
    uint16_t stop_pat  = 0u;   // 9-bit pattern for elements [20 - 28] = stop '//' 

    // Start marker: elements 0 to 8 
    for (int i = 0; i < 9; i++) {
        start_pat = (uint16_t)(start_pat << 1u) |
                    (elems[i].w >= wide_thresh ? 1u : 0u);
    }

    // Data character: elements 10 to 18  (element 9 is the inter-character gap, which we ignore)
    for (int i = 10; i < 19; i++) {
        data_pat = (uint16_t)(data_pat << 1u) |
                   (elems[i].w >= wide_thresh ? 1u : 0u);
    }

    // Stop marker: elements 20 to 28  (element 19 is another inter-character gap) 
    for (int i = 20; i < 29; i++) {
        stop_pat = (uint16_t)(stop_pat << 1u) |
                   (elems[i].w >= wide_thresh ? 1u : 0u);
    }

    // determine the scanning direction based on the strt marker
    bool forward;   // will be set to true (L to R) or false (R to L) 

    if (start_pat == CODE39_STAR_FWD_LOCAL) {
        forward = true;   // robot drove past the barcode left-to-right 
    } else if (start_pat == CODE39_STAR_REV_LOCAL) {
        forward = false;  // robot drove past the barcode right-to-left 
    } else {
        return CODE39_INVALID;  // no valid start marker found 
    }

    // check that the stop marker matches the expected pattern for the scanning direction
    uint16_t expected_stop = forward ? CODE39_STAR_FWD_LOCAL : CODE39_STAR_REV_LOCAL;

    // if the stop marker doesn't match, reject the reading as invalid
    if (stop_pat != expected_stop) return CODE39_INVALID;

    // if reading was right-to-left, reverse the data pattern so we can look it up in the table
    uint16_t decode_pat = forward ? data_pat : reverse9(data_pat);

    // look up the decoded pattern in the lookup table to get the corresponding letter
    uint8_t ch = code39_lookup(decode_pat);

    // The '*' character should only appear as a start/stop marker, never as a data character. If the decode somehow gives us '*', reject it. 
    if (ch == '*') return CODE39_INVALID;

    return ch;   // returns A–Z or CODE39_INVALID 
}
