/*
 *	host_shim.h
 *	Lets app_program/subsystem2/motion_math.c and the packing helpers in
 *	subsystem2.h compile on a PC (no kernel, no hardware), so the maths
 *	can be unit-tested with a normal gcc.  See run_host_tests.sh.
 */

#ifndef HOST_SHIM_H
#define HOST_SHIM_H

#include <stdint.h>
#include <stddef.h>

typedef int8_t		B;
typedef int16_t		H;
typedef int32_t		W;
typedef uint8_t		UB;
typedef uint16_t	UH;
typedef uint32_t	UW;
typedef int		INT;
typedef unsigned int	UINT;
typedef int		BOOL;
typedef int		ID;
typedef int		ER;
typedef int		PRI;
typedef int		SZ;
typedef void		(*FP)(void);
typedef struct { void *msgque[1]; } T_MSG;

#define TRUE		1
#define FALSE		0
#define IMPORT		extern
#define EXPORT
#define LOCAL		static
#define Inline		static inline
#define TP_PRC2		2

#define APP_H		/* skip ../app.h (kernel headers) */

#include "../../../app_program/subsystem2/subsystem2.h"

#endif /* HOST_SHIM_H */
