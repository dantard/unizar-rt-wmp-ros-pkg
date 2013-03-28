/*------------------------------------------------------------------------------
 *-------------------------        ATH5K Driver          -----------------------
 *------------------------------------------------------------------------------
 *                                                           V1.0  08/02/2010
 *
 *
 *  Feb 2010 - Samuel Cabrero <samuelcabrero@gmail.com>
 *		Initial release
 *
 *  ----------------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Autors:
 *		Samuel Cabrero        <samuelcabrero@gmail.com>
 *		Danilo Tardioli	      <dantard@unizar.es>
 *		Jose Luis Villarroel  <jlvilla@unizar.es>
 *
 *  This is a simplified version of the original ath5k driver. It should work 
 *  with all Atheros 5xxx WLAN cards. The 802.11 layer have been removed so it
 *  just send and receive frames over the air, as if it were an Ethernet bus
 *  interface.
 *
 *  Please read ath5k_interface.h for instructions.
 *
 *  This program is distributed under the terms of GPL version 2 and in the 
 *  hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *----------------------------------------------------------------------------*/

#ifndef _ATH5K_LINUX_LAYER_H_
#define _ATH5K_LINUX_LAYER_H_

#include <limits.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include <malloc.h>
#include <sys/pci.h>
#include <string.h>
#include <pthread.h>
#include <intr.h>
#include <sys/pio.h>
#include <sys/io.h>
#include <drivers/if_ether.h>
#include <stdint.h>
#include <math.h>

/********************************\
	PCI VENDOR AND DEVICE ID's	
\********************************/
#define PCI_VENDOR_ID_ATHEROS			0x168c


/***********\
	TYPES
\***********/
typedef uint64_t	u64;
typedef uint32_t	u32;
typedef uint16_t	u16;
typedef uint8_t		u8;
typedef int64_t		s64;
typedef int32_t		s32;
typedef int16_t		s16;
typedef int8_t		s8;
typedef uintptr_t	dma_addr_t;
typedef s8			__le8;
typedef s16			__le16;
typedef s32			__le32;
typedef s64			__le64;
typedef u8			__u8;
typedef u16			__u16;
typedef u32			__u32;
typedef u64			__u64;

typedef unsigned int u_int;

/*********************\
	GENERAL DEFINES
\*********************/
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define __packed __attribute__((__packed__))
#define __iomem
#define __devinit
#define __devexit

#define likely(x)	(x)
#define unlikely(x)	(x)

/***********************************\
	LITTLE/BIG ENDIAN CONVERSIONS
\***********************************/
#define cpu_to_le16(x) (u16)(x)
#define le16_to_cpu(x) (x)

/***********************\
	MEMORY ALLOCATION
\***********************/
#define GFP_KERNEL
#define kzalloc(size,mode)			malloc(size)
#define kmalloc(size,mode)			malloc(size)
#define kcalloc(elems,size,mode)	calloc(elems,size)
#define kfree(x)					free(x)

/********\
	IO
\********/
#define ioread32(addr)				readl(addr)
#define iowrite32(val,addr)			writel(val,addr)

/************\
	PRINTs
\************/
#define printk						printc

#define KERN_ERR
#define	KERN_EMERG
#define	KERN_ALERT
#define	KERN_CRIT
#define	KERN_WARNING
#define	KERN_NOTICE
#define	KERN_INFO
#define	KERN_DEBUG


#define BUG() do { \
	printe("BUG: failure at %s:%d/%s()!\n", __FILE__, __LINE__, __func__); \
} while (0)

#define BUG_ON(condition) do { if (condition) BUG(); } while(0)

/* Force a compilation error if condition is true */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#define __WARN() do { \
	printk("WARNING: failure at %s:%d/%s()!\n", __FILE__, __LINE__, __func__); \
} while (0)

#define WARN_ON(condition) ({						\
	int __ret_warn_on = !!(condition);				\
	if (unlikely(__ret_warn_on))					\
		__WARN();						\
	unlikely(__ret_warn_on);					\
})


/****************\
	INTERRUPTS
\****************/
#define IRQ_NONE	POSIX_INTR_NOT_HANDLED
#define IRQ_HANDLED POSIX_INTR_HANDLED_NOTIFY


/********************\
	BIT OPERATIONS
\********************/
#define BITOP_ADDR(x) "=m" (*(volatile long *) (x))
#define ADDR BITOP_ADDR(addr)
#define BITS_PER_LONG		32

static inline int fls(int x)
{
	int r = 32;

	if (!x)
		return 0;
	if (!(x & 0xffff0000u)) {
		x <<= 16;
		r -= 16;
	}
	if (!(x & 0xff000000u)) {
		x <<= 8;
		r -= 8;
	}
	if (!(x & 0xf0000000u)) {
		x <<= 4;
		r -= 4;
	}
	if (!(x & 0xc0000000u)) {
		x <<= 2;
		r -= 2;
	}
	if (!(x & 0x80000000u)) {
		x <<= 1;
		r -= 1;
	}
	return r;
}

static inline void __set_bit(int nr, volatile unsigned long *addr)
{
	asm volatile("bts %1,%0" : ADDR : "Ir" (nr) : "memory");
}

static inline void __clear_bit(int nr, volatile unsigned long *addr)
{
	asm volatile("btr %1,%0" : ADDR : "Ir" (nr));
}

static inline int get_bitmask_order(unsigned int count)
{
	int order;

	order = fls(count);
	return order;   /* We could be slightly more clever with -1 here... */
}

static inline int constant_test_bit(int nr, const volatile unsigned long *addr)
{
        return ((1UL << (nr % BITS_PER_LONG)) &
            (((unsigned long *)addr)[nr / BITS_PER_LONG])) != 0;
}

static inline int variable_test_bit(int nr, volatile const unsigned long *addr)
{
        int oldbit;

        asm volatile("bt %2,%1\n\t"
                     "sbb %0,%0"
                     : "=r" (oldbit)
                     : "m" (*(unsigned long *)addr), "Ir" (nr));

        return oldbit;
}

#define test_bit(nr, addr)                      \
        (__builtin_constant_p((nr))             \
         ? constant_test_bit((nr), (addr))      \
         : variable_test_bit((nr), (addr)))

#define BITS_PER_BYTE		8
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define BITS_TO_LONGS(nr)	DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))
#define DECLARE_BITMAP(name,bits) \
   	unsigned long name[BITS_TO_LONGS(bits)]
 	

/**********\
	MATH
\**********/
#define roundup(x, y) ((((x) + ((y) - 1)) / (y)) * (y))
#define max(x, y) ({                            \
        typeof(x) _max1 = (x);                  \
        typeof(y) _max2 = (y);                  \
        (void) (&_max1 == &_max2);              \
        _max1 > _max2 ? _max1 : _max2; })
#define min(x, y) ({                            \
        typeof(x) _min1 = (x);                  \
        typeof(y) _min2 = (y);                  \
        (void) (&_min1 == &_min2);              \
        _min1 < _min2 ? _min1 : _min2; })

/**
 * Returns the floor form of binary logarithm for a 32 bit integer.
 * -1 is returned if n is 0.
 */
static inline int ilog2(unsigned int n) {
  int pos = 0;
  if (n >= 1<<16) { n >>= 16; pos += 16; }
  if (n >= 1<< 8) { n >>=  8; pos +=  8; }
  if (n >= 1<< 4) { n >>=  4; pos +=  4; }
  if (n >= 1<< 2) { n >>=  2; pos +=  2; }
  if (n >= 1<< 1) {           pos +=  1; }
  return ((n == 0) ? (-1) : pos);
}


/***********\
	SLEEP
\***********/
#define udelay(u) do { \
	struct timespec ts = {0, u*1000}; \
	nanosleep (&ts, NULL); \
} while (0);

#define mdelay(u) do { \
	struct timespec ts = {0, u*1000*1000}; \
	nanosleep (&ts, NULL); \
} while (0);


/*****************\
	ERROR CODES
\*****************/
#define EPERM            1      /* Operation not permitted */
#define ENOENT           2      /* No such file or directory */
#define ESRCH            3      /* No such process */
#define EINTR            4      /* Interrupted system call */
#define EIO              5      /* I/O error */
#define ENXIO            6      /* No such device or address */
#define E2BIG            7      /* Argument list too long */
#define ENOEXEC          8      /* Exec format error */
#define EBADF            9      /* Bad file number */
#define ECHILD          10      /* No child processes */
#define EAGAIN          11      /* Try again */
#define ENOMEM          12      /* Out of memory */
#define EACCES          13      /* Permission denied */
#define EFAULT          14      /* Bad address */
#define ENOTBLK         15      /* Block device required */
#define EBUSY           16      /* Device or resource busy */
#define EEXIST          17      /* File exists */
#define EXDEV           18      /* Cross-device link */
#define ENODEV          19      /* No such device */
#define ENOTDIR         20      /* Not a directory */
#define EISDIR          21      /* Is a directory */
#define EINVAL          22      /* Invalid argument */
#define ENFILE          23      /* File table overflow */
#define EMFILE          24      /* Too many open files */
#define ENOTTY          25      /* Not a typewriter */
#define ETXTBSY         26      /* Text file busy */
#define EFBIG           27      /* File too large */
#define ENOSPC          28      /* No space left on device */
#define ESPIPE          29      /* Illegal seek */
#define EROFS           30      /* Read-only file system */
#define EMLINK          31      /* Too many links */
#define EPIPE           32      /* Broken pipe */
#define EDOM            33      /* Math argument out of domain of func */
#define ERANGE          34      /* Math result not representable */
#define EOPNOTSUPP      95      /* Operation not supported on transport endpoint */
#define ETIMEDOUT       110     /* Connection timed out */
#define EINPROGRESS     115     /* Operation now in progress */
#define ENOTSUPP        524     /* Operation is not supported */
#define MAX_ERRNO       4095
#define NETDEV_TX_BUSY	1		/* driver tx path was busy*/
#define NETDEV_TX_OK	0       /* driver took care of packet */

#define IS_ERR(x) ((unsigned long)(x) >= (unsigned long)-MAX_ERRNO)
#define PTR_ERR(x) ((unsigned long)(x))
#define ERR_PTR(x) ((void *)(x))



/*******************\
	SOCKET BUFFER
\*******************/
struct sk_buff
{
	unsigned char *head;
	unsigned char *data;
	unsigned char *tail;
	unsigned char *end;

	unsigned int len;
};

static inline struct sk_buff *dev_alloc_skb (unsigned int len)
{	
	struct sk_buff *skb = (struct sk_buff *) malloc(sizeof (struct sk_buff));
	if (!skb)
		printe("Error instanciando sk_buff\n");
	
	skb->data = (unsigned char *)malloc(len);
	if (!skb->data)
		printe("Error instanciando skb->data\n");

	memset(skb->data, 0, len);
	
	skb->head = skb->data;
	skb->tail = skb->data;
	skb->end  = skb->tail + len;
	skb->len  = 0;

	return skb;
}

#define dev_kfree_skb(a)        dev_kfree_skb_any(a)

static inline void dev_kfree_skb_any(struct sk_buff *skb)
{
	if (!skb)
		return;
	else
		free(skb);
}

static inline unsigned char *skb_put (struct sk_buff *skb, int len)
{
	unsigned char *prev_tail = skb->tail;

	skb->tail += len;
	skb->len += len;

	return prev_tail;
}

static inline void skb_reserve(struct sk_buff *skb, int len)
{
	skb->data += len;
	skb->tail += len;
}

static inline int skb_tailroom(const struct sk_buff *skb)
{	
	return skb->end - skb->tail;
}

static inline unsigned int skb_headroom(const struct sk_buff *skb)
{
	return skb->data - skb->head;
}

static inline unsigned char *skb_pull(struct sk_buff *skb, unsigned int len)
{
	if (len > skb->len)
		return NULL;
	else
	{
		skb->len -= len;
		return skb->data += len;
	}
}

static inline unsigned char *skb_push(struct sk_buff *skb, unsigned int len)
{
	skb->data -= len;
	skb->len += len;
	if (skb->data < skb->head)
		printe("skb_push: no hay suficiente espacio disponible");

	return skb->data;
}

static inline void skb_copy_from_linear_data(const struct sk_buff *skb,
                                             void *to,
                                             const unsigned int len)
{
        memcpy(to, skb->data, len);
}

static inline void skb_copy_from_linear_data_offset(const struct sk_buff *skb,
                                                    const int offset, void *to,
                                                    const unsigned int len)
{
        memcpy(to, skb->data + offset, len);
}

static inline void skb_copy_to_linear_data(struct sk_buff *skb,
                                           const void *from,
                                           const unsigned int len)
{
        memcpy(skb->data, from, len);
}

static inline void skb_copy_to_linear_data_offset(struct sk_buff *skb,
                                                  const int offset,
                                                  const void *from,
                                                  const unsigned int len)
{
        memcpy(skb->data + offset, from, len);
}


/****************\
	LINUX LIST
\****************/
#define list_first_entry(ptr, type, member) list_entry((ptr)->next, type, member)

#endif

