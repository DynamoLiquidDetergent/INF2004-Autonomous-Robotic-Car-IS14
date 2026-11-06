#ifndef S4_MEMORY_H
#define S4_MEMORY_H
#include "s4_types.h"
static inline void s4_zero(void *address,s4_size_t count)
{
    unsigned char *p=(unsigned char *)address;
    while(count!=0U){*p=0U;++p;--count;}
}
#endif
