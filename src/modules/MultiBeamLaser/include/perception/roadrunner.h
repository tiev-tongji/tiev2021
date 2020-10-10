

#ifndef DGC_ROADRUNNER_H
#define DGC_ROADRUNNER_H

#define __STDC_CONSTANT_MACROS

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifdef __DARWIN__
#include <macAddOns.h>
#define de_const_ 		// scandir parameter dirent is unfortunately not const on macs :-(
#else
#define de_const_ const
#endif

#include <unistd.h>
#include <ctype.h>
#ifdef __USE_BSD
#undef __USE_BSD
#include <string.h>
#define __USE_BSD
#else
#include <string.h>
#endif
#include <strings.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include <float.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <sys/stat.h>

#define MAXDOUBLE DBL_MAX

#include <global.h>

#endif
