/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/linux_us/config/compiler.h
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

   #include <sys/types.h>
   #include <sys/time.h>
   #include <unistd.h>
   #include <stdlib.h>
   #include <stdio.h>
   #include <string.h>
   #include "errno.h"
   #include <pthread.h>
   #include <limits.h>
   #include <math.h>
   #include <signal.h>
   #include <time.h>
   #include <stdarg.h>

 	#include <malloc.h>
 	#include <semaphore.h>
	#include <assert.h>
	#include "wmp_config.h"
	#include "../../../core/include/wmp_utils.h"

	#define MALLOC(p) malloc(p)
	#define FREE(p) free(p)

   #define SEM_T sem_t
   #define SEM_INIT(p,q,r) sem_init(p,q,r)
 	#define WAIT(nMutex)  sem_wait(&nMutex )
   #define WAIT_TIMED(nMutex,time)                 \
   ({                                              \
      struct timespec ts;                          \
      clock_gettime(CLOCK_REALTIME,&ts);           \
      wmp_add_ms(&ts,time);                        \
      sem_timedwait(&nMutex, &ts);                 \
   })
	#define SIGNAL(nMutex) sem_post(&nMutex)
   #define SEM_GET_COUNT(nMutex) ({int val; sem_getvalue(&nMutex, &val); val;})

   #define MUTEX pthread_mutex_t
   #define MUTEX_INIT(m) pthread_mutex_init(m,0);
   #define MUTEX_WAIT(m) pthread_mutex_lock(&m)
   #define MUTEX_WAIT_SPIN(m) pthread_mutex_lock(&m)
   #define MUTEX_SIGNAL(m) pthread_mutex_unlock(&m)


   #define THREAD_T(name) pthread_t name = 0
   #define THREAD_CREATE(th_var,th_fun,name) pthread_create(&th_var, NULL, th_fun, NULL)
   #define THREAD_STOP(th_var)

   #define THREAD_SEM_T pthread_mutex_t
   #define THREAD_SEM_INIT_LOCKED(p) pthread_mutex_init(p,0); pthread_mutex_lock(p);
   #define THREAD_SEM_WAIT(p) pthread_mutex_lock(p)
   #define THREAD_SEM_WAIT_TIMED(nMutex,time)      \
   ({                                              \
      struct timespec ts;                          \
      clock_gettime(CLOCK_REALTIME,&ts);           \
      wmp_add_ms(&ts,time);                        \
      pthread_mutex_timedlock(&nMutex, &ts);       \
   })
   #define THREAD_SEM_SIGNAL(p) pthread_mutex_unlock(p)

   #define ASSERT(p) assert(p);

   #define GETNSTIMEOFDAY(p) clock_gettime(CLOCK_REALTIME, p)

   #define FLOAT_OPS_START()
   #define FLOAT_OPS_END()

   #define DO_DIV64(n , b) ((n)/(b))

   #define EXIT(val) exit(val)

   #define ATOF(src, dst) *dst = atof(src)

   #define EXPORT_SYMBOLS()

#if WMP_DEBUG_LEVEL > 0
	#define DEBUG(p)  p
	#define WMP_DEBUG(output, ...)   fprintf(output, __VA_ARGS__)
	#define WMP_DEBUG1(output, ...)  fprintf(output, __VA_ARGS__)
	#define WMP_DBG(level,...) if (level & (WMP_DEBUG_LEVEL)) fprintf(stderr, __VA_ARGS__)
#else
	 #define DEBUG(p)
     #define WMP_DEBUG(output, ...)
     #define WMP_DEBUG1(output, ...)
     #define WMP_DBG(output,...)

#endif
	#define WMP_MSG(output, ...) fprintf(output, __VA_ARGS__)
	//#define WMP_ERROR(output, ...) textcolor(1,1,0); fprintf(output, __VA_ARGS__) ; textcolor(0,7,0);
	#define WMP_ERROR(output, ...) fprintf(output, __VA_ARGS__);
#endif
/*CONFIGURATION_H_*/

