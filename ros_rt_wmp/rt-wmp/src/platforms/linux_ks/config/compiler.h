/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         
 *
 *
 *  File: ./src/platforms/linux_ks/config/compiler.h
 *  Authors: Rubén Durán
 *           Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include "wmp_config.h"
#include <linux/kthread.h>
#include <asm/div64.h>
#include <linux/time.h>
#include <linux/random.h>
#include <linux/slab.h>
#define MALLOC(size) kmalloc(size, GFP_KERNEL)
#define FREE(p) kfree(p)

//#define PRINTK(str, ...) printk(KERN_ERR "[%s:%d]: " str, __FILE__, __LINE__, __VA_ARGS__)
#define PRINTK(str, ...)

#define SEM_T struct semaphore
#define SEM_INIT(p,q,r) sema_init(p,r)
#define WAIT(nMutex) ({int res; PRINTK("WT %s\n", #nMutex); res = down_interruptible(&nMutex);})
#define WAIT_TIMED(nMutex, time) ({PRINTK("WT %s\n", #nMutex); down_timeout(&nMutex, msecs_to_jiffies(time)); })
#define SIGNAL(nMutex) ({PRINTK("S %s\n", #nMutex); up(&nMutex); })
#define SEM_GET_COUNT(nMutex) (nMutex).count

typedef struct {
	spinlock_t spin;
	unsigned long flags;
} MUTEX;
#define MUTEX_INIT(m) spin_lock_init(&(m)->spin);
#define MUTEX_WAIT(m) ({PRINTK("MW %s\n", #m); spin_lock_irqsave(&(m).spin,(m).flags); })
#define MUTEX_WAIT_SPIN(m) ({PRINTK("MWS %s\n", #m); spin_lock_irqsave(&(m).spin,(m).flags); })
#define MUTEX_SIGNAL(m) ({PRINTK("MS %s\n", #m); spin_unlock_irqrestore(&(m).spin,(m).flags); })


/*#define MUTEX_WAIT(m) spin_lock_irqsave(&(m).spin,(m).flags)
 #define MUTEX_WAIT_SPIN(m) spin_lock_irqsave(&(m).spin,(m).flags)
 #define MUTEX_SIGNAL(m) spin_unlock_irqrestore(&(m).spin,(m).flags)*/

#define THREAD_T(name) struct task_struct * name = NULL
#define THREAD_CREATE(th_var,th_fun,name) th_var = kthread_run(th_fun,NULL,name)
#define THREAD_STOP(th_var) kthread_stop(th_var)

#define THREAD_SEM_T struct semaphore
#define THREAD_SEM_INIT_LOCKED(p) sema_init(p,0)
#define THREAD_SEM_WAIT(p) ({int res; PRINTK("TSW %s\n", #p); res = down_interruptible(p); })
#define THREAD_SEM_WAIT_TIMED(p, time) ({ PRINTK("TSWT %s\n", #p); down_timeout(&p, msecs_to_jiffies(time)); })
#define THREAD_SEM_SIGNAL(p) ({ PRINTK("TSS %s\n", #p); up(p); })




#define ASSERT(test) if(!(test)) printk(KERN_INFO "ASSERT: \"%s\" failed.\n", #test)

#define GETNSTIMEOFDAY(p) getnstimeofday(p)

#define DO_DIV64(n , b)                \
   ({                                     \
      unsigned long long __n = n;         \
      do_div(__n,b);                      \
      __n;                                \
   })

#define EXIT(val) return val

#define rand()                                              \
   ({                                                          \
      int __a;                                                 \
      get_random_bytes(&__a, sizeof(__a));                     \
      __a;                                                     \
   })

#define floor(n) ((int)(n))

#define atoi(s)                                             \
   ({                                                          \
      char *next;                                              \
      (int) simple_strtol(s,&next,10);                         \
   })

void atof(const char *cadena, double *res);

#define ATOF(src, dst) atof(src, dst)

char * fgets(char *s, int size, struct file *f);

#define usleep(usecs) 	\
			{unsigned long timeout = ((usecs * (msecs_to_jiffies(1) + 1)) / (int)1000);\
			 while (timeout) {\
			  timeout = schedule_timeout_uninterruptible(timeout);\
			}};

#ifdef USE_MESSAGE_COMPRESSION
#define EXPORT_MESSAGE_COMPRESSION() EXPORT_SYMBOL(wmpZPush)
#else
#define EXPORT_MESSAGE_COMPRESSION()
#endif
struct proc_dir_entry * get_proc_root(void);

#define EXPORT_SYMBOLS()   EXPORT_SYMBOL(wmpPush);\
                              EXPORT_SYMBOL(wmpPop);\
                              EXPORT_SYMBOL(wmpTimedPop);\
                              EXPORT_SYMBOL(wmpNonBlockingPop);\
                              EXPORT_MESSAGE_COMPRESSION()\
                              EXPORT_SYMBOL(wmpGetNodeId);\
                              EXPORT_SYMBOL(wmpGetNumOfNodes);\
                              EXPORT_SYMBOL(wmpGetLatestLQM);\
                              EXPORT_SYMBOL(wmpIsNetworkConnected);\
                              EXPORT_SYMBOL(wmpIsNetworkConnectedBlocking);\
                              EXPORT_SYMBOL(wmp_queue_tx_remove_head);\
                              EXPORT_SYMBOL(wmpSetCpuDelay);\
                              EXPORT_SYMBOL(wmpSetTimeout);\
                              EXPORT_SYMBOL(wmpSetWCMult);\
                              EXPORT_SYMBOL(wmpSetRate);\
                              EXPORT_SYMBOL(wmpGetCpuDelay);\
                              EXPORT_SYMBOL(wmpGetTimeout);\
                              EXPORT_SYMBOL(wmpGetWCMult);\
                              EXPORT_SYMBOL(wmpGetRate);\
                              \
                              EXPORT_SYMBOL(wmp_queue_tx_get_room);\
                              EXPORT_SYMBOL(wmpGetNumOfElementsInTXQueue);\
                              EXPORT_SYMBOL(wmpGetNumOfElementsInRXQueue);\
                              EXPORT_SYMBOL(wmpSetup);\
                              EXPORT_SYMBOL(wmpSetupList);\
                              EXPORT_SYMBOL(wmpRun);\
                              EXPORT_SYMBOL(wmpRunBG);\
                              EXPORT_SYMBOL(wmpExit);\
                              EXPORT_SYMBOL(wmpInmediateExit);\
                              EXPORT_SYMBOL(wmpSetQuiet);\
                              EXPORT_SYMBOL(wmpGetNetIT);\
                              EXPORT_SYMBOL(wmpGetMTU);\
                              EXPORT_SYMBOL(wmpSetActiveSearch);\
                              EXPORT_SYMBOL(wmpGetActiveSearch);\
                              EXPORT_SYMBOL(wmpSetInstanceId);\
                              EXPORT_SYMBOL(wmpSetPrimBasedRouting);\
                              EXPORT_SYMBOL(wmpSetMessageReschedule);\
                              EXPORT_SYMBOL(wmpSetFlowControl);\
                              EXPORT_SYMBOL(wmpGetInstanceId);\
                              EXPORT_SYMBOL(wmpGetPrimBasedRouting);\
                              EXPORT_SYMBOL(wmpGetMessageReschedule);\
                              EXPORT_SYMBOL(wmpGetFlowControl);\
                              EXPORT_SYMBOL(wmpGetSerial);\
                              EXPORT_SYMBOL(wmpGetLoopId);\
							  EXPORT_SYMBOL(wmpPushData);\
                              EXPORT_SYMBOL(wmpPopData);\
                              EXPORT_SYMBOL(wmpPopDataTimeout);\
                              EXPORT_SYMBOL(wmpPopDataDone);\
							  EXPORT_SYMBOL(wmpPopDataTimeoutCopy);\
							  EXPORT_SYMBOL(wmpPopDataCopy);\
							  EXPORT_SYMBOL(wmp_queue_rx_get_elem_size);\
							  EXPORT_SYMBOL(wmp_queue_rx_get_elem_priority);\
							  EXPORT_SYMBOL(wmp_queue_rx_get_elem_source);\
							  EXPORT_SYMBOL(wmp_queue_rx_get_elem_data);\
							  EXPORT_SYMBOL(wmpWaitData);\
						      EXPORT_SYMBOL(wmp_queue_rx_set_elem_done);\
						      EXPORT_SYMBOL(wmpForceTopology);\
						      EXPORT_SYMBOL(wmpSetTaskMinimumSeparation);\
						      EXPORT_SYMBOL(lqm_get_distance);\
						      EXPORT_SYMBOL(get_proc_root);\
						      EXPORT_SYMBOL(wmpForceBurst);


#if WMP_DEBUG_LEVEL > 0
#define DEBUG(p)  p
#define WMP_DEBUG(output, ...)   printk(KERN_DEBUG __VA_ARGS__)
#define WMP_DEBUG1(output, ...)  printk(KERN_DEBUG __VA_ARGS__)
#define WMP_DBG(level,...) if (level & (WMP_DEBUG_LEVEL)) printk(KERN_DEBUG __VA_ARGS__)
#else
#define DEBUG(p)
#define WMP_DEBUG(output, ...)
#define WMP_DEBUG1(output, ...)
#define WMP_DBG(output,...)
#endif
//#define WMP_MSG(output, ...) fprintf(output, __VA_ARGS__)
#define WMP_MSG(output, ...) printk(KERN_INFO __VA_ARGS__)
//#define WMP_ERROR(output, ...) textcolor(1,1,0); fprintf(output, __VA_ARGS__) ; textcolor(0,7,0);
#define WMP_ERROR(output, ...) printk(KERN_ERR __VA_ARGS__)
#endif
/*CONFIGURATION_H_*/
