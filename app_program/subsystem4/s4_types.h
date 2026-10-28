#ifndef S4_TYPES_H
#define S4_TYPES_H
#include <stdint.h>
#include <stdbool.h>
/* Bounded buffers use this count type; do not redeclare kernel s4_size_t. */
typedef uint32_t s4_size_t;
#ifndef NULL
#define NULL ((void *)0)
#endif
#endif
