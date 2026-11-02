/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include <locale.h>
#include <stdlib.h>


// The conversions with an explicit locale: the thread uses that locale for
// the conversion only (its decimal point is all they depend on).


extern "C" double
strtod_l(const char* string, char** end, locale_t locale)
{
	locale_t previous = uselocale(locale);
	double value = strtod(string, end);
	uselocale(previous);
	return value;
}


extern "C" float
strtof_l(const char* string, char** end, locale_t locale)
{
	locale_t previous = uselocale(locale);
	float value = strtof(string, end);
	uselocale(previous);
	return value;
}


extern "C" long double
strtold_l(const char* string, char** end, locale_t locale)
{
	locale_t previous = uselocale(locale);
	long double value = strtold(string, end);
	uselocale(previous);
	return value;
}
