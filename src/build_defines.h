/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "app_version.h"

#define FW_NAME CONFIG_NCS_APPLICATION_BOOT_BANNER_STRING
#define FW_VERSION_MAJOR APP_VERSION_MAJOR
#define FW_VERSION_MINOR APP_VERSION_MINOR
#define FW_VERSION_PATCH APP_PATCHLEVEL

// https://stackoverflow.com/questions/11697820/how-to-use-date-and-time-predefined-macros-in-as-two-integers-then-stri
#define COMPUTE_BUILD_YEAR                                                             \
	((__DATE__[7] - '0') * 1000 + (__DATE__[8] - '0') * 100 + (__DATE__[9] - '0') * 10 \
	 + (__DATE__[10] - '0'))

#define COMPUTE_BUILD_DAY \
	(((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + (__DATE__[5] - '0'))

#define BUILD_MONTH_IS_JAN \
	(__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR \
	(__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY \
	(__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN \
	(__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL \
	(__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')

#define COMPUTE_BUILD_MONTH      \
	((BUILD_MONTH_IS_JAN)   ? 1  \
	 : (BUILD_MONTH_IS_FEB) ? 2  \
	 : (BUILD_MONTH_IS_MAR) ? 3  \
	 : (BUILD_MONTH_IS_APR) ? 4  \
	 : (BUILD_MONTH_IS_MAY) ? 5  \
	 : (BUILD_MONTH_IS_JUN) ? 6  \
	 : (BUILD_MONTH_IS_JUL) ? 7  \
	 : (BUILD_MONTH_IS_AUG) ? 8  \
	 : (BUILD_MONTH_IS_SEP) ? 9  \
	 : (BUILD_MONTH_IS_OCT) ? 10 \
	 : (BUILD_MONTH_IS_NOV) ? 11 \
	 : (BUILD_MONTH_IS_DEC) ? 12 \
							: /* error default */ 99)

#define COMPUTE_BUILD_HOUR ((__TIME__[0] - '0') * 10 + __TIME__[1] - '0')
#define COMPUTE_BUILD_MIN ((__TIME__[3] - '0') * 10 + __TIME__[4] - '0')
#define COMPUTE_BUILD_SEC ((__TIME__[6] - '0') * 10 + __TIME__[7] - '0')

#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')

#define BUILD_YEAR ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
#define BUILD_DAY ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)

#define BUILD_TIME_IS_BAD (__TIME__[0] == '?')

#define BUILD_HOUR ((BUILD_TIME_IS_BAD) ? 99 : COMPUTE_BUILD_HOUR)
#define BUILD_MIN ((BUILD_TIME_IS_BAD) ? 99 : COMPUTE_BUILD_MIN)
#define BUILD_SEC ((BUILD_TIME_IS_BAD) ? 99 : COMPUTE_BUILD_SEC)

#define TOSTRING(x) STRINGIFY(x)

#define FW_STRING                                       \
	FW_NAME " " APP_VERSION_EXTENDED_STRING             \
			" "                                         \
			"(Commit " TOSTRING(APP_BUILD_VERSION       \
			) ", Build %d-%02d-%02d %02d:%02d:%02d)\n", \
		BUILD_YEAR, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN, BUILD_SEC
