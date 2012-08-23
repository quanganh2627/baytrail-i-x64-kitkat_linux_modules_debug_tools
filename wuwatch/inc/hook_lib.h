/*****************************************************************************
#                      INTEL CONFIDENTIAL
#
# Copyright 2009-2012 Intel Corporation All Rights Reserved.
#
# This file is provided for internal use only. You are free to modify it
# for your own internal purposes, but you cannot release it to the external
# open source community under any circumstances without SSG.DPD's approval.
#
# If you make significant improvements, fix bugs, or have suggestions for
# additional features or improvements, please provide your changes to
# Robert or Gautam for inclusion in this tool.
#
# Please contact Robert Knight (robert.knight@intel.com) or Gautam Upadhyaya
# (gautam.upadhyaya@intel.com) if you have any questions.
#
*****************************************************************************
*/


#ifndef _HOOK_LIB_H_
#define _HOOK_LIB_H_

#include "pw_bt.h"

/*
 * Atomic operations
 */
#define LOCK_PREFIX "lock ; "

#define __xg(x) ((volatile long *)(x))

typedef struct { volatile int count; } pw_atomic_t;

#define PW_ATOMIC_INIT(i)	{ (i) }

#define pw_atomic_read(v)		((v)->count)

static inline unsigned long __pw_cmpxchg(volatile void *ptr, unsigned long old,
					 unsigned long _new, int size)
{
	unsigned long prev;
	switch (size) {
	case 1:
		__asm__ __volatile__(LOCK_PREFIX "cmpxchgb %b1,%2"
				     : "=a"(prev)
				     : "q"(_new), "m"(*__xg(ptr)), "0"(old)
				     : "memory");
		return prev;
	case 2:
		__asm__ __volatile__(LOCK_PREFIX "cmpxchgw %w1,%2"
				     : "=a"(prev)
				     : "q"(_new), "m"(*__xg(ptr)), "0"(old)
				     : "memory");
		return prev;
	case 4:
		__asm__ __volatile__(LOCK_PREFIX "cmpxchgl %k1,%2"
				     : "=a"(prev)
				     : "q"(_new), "m"(*__xg(ptr)), "0"(old)
				     : "memory");
		return prev;
	}
	return old;
};

#define pw_cmpxchg(ptr,o,n)\
	((__typeof__(*(ptr)))__pw_cmpxchg((ptr),(unsigned long)(o),\
					  (unsigned long)(n),sizeof(*(ptr))))

#define pw_cas(ptr, o, n) ( (pw_cmpxchg( (ptr), (o), (n) ) ) )
#define pw_bool_cas(ptr, o, n) ( (pw_cas( (ptr), (o), (n) ) ) == (o) )

#define PW_CAS(ptr, o, n) ( pw_bool_cas( (ptr), (o), (n) ) )

#endif // _HOOK_LIB_H_
