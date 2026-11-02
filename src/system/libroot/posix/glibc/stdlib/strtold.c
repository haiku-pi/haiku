/* Copyright (C) 1999 Free Software Foundation, Inc.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with the GNU C Library; see the file COPYING.LIB.  If not,
   write to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

#include <float.h>
#include <math.h>
#include <stdlib.h>
#ifdef USE_WIDE_CHAR
# include <wchar.h>
#endif

#ifndef __NO_LONG_DOUBLE_MATH
/* The actual implementation for all floating point sizes is in strtod.c.
   These macros tell it to produce the `long double' version, `strtold'
   (or `wcstold', when included from wcstold.c with USE_WIDE_CHAR).  */

# define FLOAT		long double
# define FLT		LDBL
# ifdef USE_WIDE_CHAR
#  ifdef USE_IN_EXTENDED_LOCALE_MODEL
#   define STRTOF	__wcstold_l
#  else
#   define STRTOF	wcstold
#  endif
# else
#  ifdef USE_IN_EXTENDED_LOCALE_MODEL
#   define STRTOF	__strtold_l
#  else
#   define STRTOF	strtold
#  endif
# endif
# define MPN2FLOAT	__mpn_construct_long_double
# define FLOAT_HUGE_VAL	HUGE_VALL

/* SET_MANTISSA (see strtod.c) depends on the long double format.  */
# if LDBL_MANT_DIG == 113
/* IEEE 754 binary128 (arm64, riscv64, sparc64): the number goes into the
   low 64 of the 111 payload bits; the quiet bit stays set.  */
#  define SET_MANTISSA(flt, mant) \
  do { union ieee854_long_double u;					      \
       u.d = (flt);							      \
       u.ieee_nan.mantissa0 = 0;					      \
       u.ieee_nan.mantissa1 = 0;					      \
       u.ieee_nan.mantissa2 = (mant) >> 32;				      \
       u.ieee_nan.mantissa3 = (mant) & 0xffffffff;			      \
       (flt) = u.d;							      \
  } while (0)
# elif LDBL_MANT_DIG == 64
/* The x87 80-bit extended format (x86, x86_64): the number goes into the
   62 payload bits; the explicit integer bit and the quiet bit stay set.  */
#  define SET_MANTISSA(flt, mant) \
  do { union ieee854_long_double u;					      \
       u.d = (flt);							      \
       u.ieee_nan.mantissa0 = ((mant) >> 32) & 0x3fffffff;		      \
       u.ieee_nan.mantissa1 = (mant) & 0xffffffff;			      \
       (flt) = u.d;							      \
  } while (0)
# else
#  error "long double format not supported: define __NO_LONG_DOUBLE_MATH"
# endif

# include "strtod.c"
#else
/* There is no `long double' type, use the `double' implementations.  */
# ifdef USE_WIDE_CHAR
long double
__wcstold_internal (const wchar_t *nptr, wchar_t **endptr, int group)
{
  return __wcstod_internal (nptr, endptr, group);
}
libc_hidden_def (__wcstold_internal)

long double
wcstold (const wchar_t *nptr, wchar_t **endptr)
{
  return __wcstod_internal (nptr, endptr, 0);
}
# else
long double
__strtold_internal (const char *nptr, char **endptr, int group)
{
  return __strtod_internal (nptr, endptr, group);
}

long double
strtold (const char *nptr, char **endptr)
{
  return __strtod_internal (nptr, endptr, 0);
}
# endif
#endif
