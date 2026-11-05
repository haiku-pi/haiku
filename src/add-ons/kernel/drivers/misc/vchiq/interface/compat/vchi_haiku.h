/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * The interface follows FreeBSD's vchi_bsd.h (Copyright (c) 2010 Max Khon,
 * (c) 2012 Oleksandr Tymoshenko, (c) 2013 Jared D. McNeill; BSD-2-Clause),
 * which gives the VCHIQ core the Linux kernel primitives it was written for.
 */
#ifndef VCHI_HAIKU_H
#define VCHI_HAIKU_H


/*!	The Linux kernel primitives the VCHIQ core uses, on Haiku's.

	Errors: the core returns Linux-style negated errno values ("-EINVAL").
	Haiku's errno values are negative already, so the port writes them
	without the minus; the functions here return EINTR, EFAULT etc. as they
	are.
*/


#include <KernelExport.h>
#include <OS.h>
#include <SupportDefs.h>
#include <lock.h>

#include <errno.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioccom.h>
#include <sys/types.h>


#ifdef __cplusplus
extern "C" {
#endif


/* FreeBSD's <sys/ioccom.h> has these (vchiq_ioctl.h's 32-bit commands) */
#ifndef _IOC_NEWTYPE
#	define _IOC_NEWLEN(ioc, len) \
	((~(IOCPARM_MASK << 16) & (ioc)) | (((len) & IOCPARM_MASK) << 16))
#	define _IOC_NEWTYPE(ioc, type)	_IOC_NEWLEN((ioc), sizeof(type))
#endif


/* Copying from and to user memory: 0, or not 0 when it faulted */
#define copy_from_user(to, from, n)	(user_memcpy((to), (from), (n)) != B_OK)
#define copy_to_user(to, from, n)	(user_memcpy((to), (from), (n)) != B_OK)


/* Atomics (Linux's argument order and return values) */
typedef volatile int atomic_t;

#define ATOMIC_INIT(v)			(v)
#define atomic_set(p, v)		__atomic_store_n((p), (v), __ATOMIC_SEQ_CST)
#define atomic_read(p)			__atomic_load_n((p), __ATOMIC_SEQ_CST)
#define atomic_inc(p)			((void)__atomic_add_fetch((p), 1, \
	__ATOMIC_SEQ_CST))
#define atomic_dec(p)			((void)__atomic_sub_fetch((p), 1, \
	__ATOMIC_SEQ_CST))
#define atomic_add(i, p)		((void)__atomic_add_fetch((p), (i), \
	__ATOMIC_SEQ_CST))
#define atomic_sub(i, p)		((void)__atomic_sub_fetch((p), (i), \
	__ATOMIC_SEQ_CST))
#define atomic_inc_return(p)	__atomic_add_fetch((p), 1, __ATOMIC_SEQ_CST)
#define atomic_dec_return(p)	__atomic_sub_fetch((p), 1, __ATOMIC_SEQ_CST)
#define atomic_xchg(p, v)		__atomic_exchange_n((p), (v), __ATOMIC_SEQ_CST)

/*!	Returns the value \a v held before: \a oldValue when it was swapped. */
static inline int
atomic_cmpxchg(atomic_t* v, int oldValue, int newValue)
{
	__atomic_compare_exchange_n(v, &oldValue, newValue, 0, __ATOMIC_SEQ_CST,
		__ATOMIC_SEQ_CST);
	return oldValue;
}


/* Barriers. The VideoCore is outside the inner shareable domain and the
   memory shared with it is not cacheable: the full-system DSB forms, as
   Linux's mandatory barriers on arm64. */
#define dsb(option)		__asm__ __volatile__("dsb " #option : : : "memory")
#define mb()			dsb(sy)
#define rmb()			dsb(ld)
#define wmb()			dsb(st)
#define smp_mb()		mb()
#define smp_rmb()		rmb()
#define smp_wmb()		wmb()


/* Spinlocks. As in FreeBSD's port, a sleeping lock: no spinlock is taken in
   interrupt context (the doorbell handler only up()s semaphores). */
typedef mutex spinlock_t;

#define DEFINE_SPINLOCK(name)	mutex name = MUTEX_INITIALIZER(#name)
#define spin_lock_init(lock)	mutex_init((lock), "vchiq spinlock")
#define spin_lock(lock)			mutex_lock(lock)
#define spin_unlock(lock)		mutex_unlock(lock)
#define spin_lock_bh(lock)		mutex_lock(lock)
#define spin_unlock_bh(lock)	mutex_unlock(lock)


/* Mutexes: Linux's struct mutex is Haiku's. */
#define lmutex_init(lock)		mutex_init((lock), "vchiq mutex")
#define lmutex_lock(lock)		mutex_lock(lock)
#define lmutex_unlock(lock)		mutex_unlock(lock)
#define lmutex_destroy(lock)	mutex_destroy(lock)

static inline int
lmutex_lock_interruptible(struct mutex* lock)
{
	return mutex_lock_with_timeout(lock, B_CAN_INTERRUPT, 0) == B_OK
		? 0 : EINTR;
}


/* Read-write locks */
typedef rw_lock rwlock_t;

#define rwlock_init(lock)		rw_lock_init((lock), "vchiq rwlock")
#define read_lock(lock)			rw_lock_read_lock(lock)
#define read_unlock(lock)		rw_lock_read_unlock(lock)
#define write_lock(lock)		rw_lock_write_lock(lock)
#define write_unlock(lock)		rw_lock_write_unlock(lock)
#define read_lock_bh(lock)		rw_lock_read_lock(lock)
#define read_unlock_bh(lock)	rw_lock_read_unlock(lock)
#define write_lock_bh(lock)		rw_lock_write_lock(lock)
#define write_unlock_bh(lock)	rw_lock_write_unlock(lock)


/* Counting semaphores and completions: a count under a spinlock and a
   condition variable (vchi_haiku.cpp), without kernel semaphores (the core
   makes one per possible service: 4096). up() and complete() are safe in
   interrupt context. */
struct vchiq_condition {
	uint64	storage[10];
		/* a ConditionVariable */
};

struct semaphore {
	spinlock				lock;
	int32					count;
	struct vchiq_condition	condition;
};

void _sema_init(struct semaphore* s, int value);
void _sema_destroy(struct semaphore* s);
void down(struct semaphore* s);
int down_interruptible(struct semaphore* s);
int down_trylock(struct semaphore* s);
void up(struct semaphore* s);

struct completion {
	spinlock				lock;
	int32					done;
		/* -1: complete_all() was called, every wait succeeds */
	struct vchiq_condition	condition;
};

void init_completion(struct completion* c);
void destroy_completion(struct completion* c);
void complete(struct completion* c);
void complete_all(struct completion* c);
void INIT_COMPLETION_locked(struct completion* c);
void wait_for_completion(struct completion* c);
int try_wait_for_completion(struct completion* c);
int wait_for_completion_interruptible(struct completion* c);
long wait_for_completion_interruptible_timeout(struct completion* c,
	unsigned long timeout);
int wait_for_completion_killable(struct completion* c);

#define INIT_COMPLETION(x)		INIT_COMPLETION_locked(&(x))


/* Timers. The core's only timer is the suspend timer, which is never started
   on this platform (vchiq_platform_use_suspend_timer() is 0). */
struct timer_list {
	unsigned long	expires;
	void			(*function)(unsigned long);
	unsigned long	data;
};

void vchiq_init_timer(struct timer_list* t);
void vchiq_setup_timer(struct timer_list* t, void (*function)(unsigned long),
	unsigned long data);
void vchiq_mod_timer(struct timer_list* t, unsigned long expires);
void vchiq_add_timer(struct timer_list* t);
int vchiq_del_timer(struct timer_list* t);
int vchiq_del_timer_sync(struct timer_list* t);


/* Kernel threads. A VCHIQ_THREAD_T holds a thread_id. */
typedef struct vchiq_thread* VCHIQ_THREAD_T;

#define VCHIQ_THREAD_ID(thread)	((thread_id)(addr_t)(thread))
#define current					((VCHIQ_THREAD_T)(addr_t)find_thread(NULL))

VCHIQ_THREAD_T vchiq_thread_create(int (*threadfn)(void* data), void* data,
	const char namefmt[], ...) __attribute__((format(printf, 3, 4)));
void set_user_nice(VCHIQ_THREAD_T thread, int nice);
void wake_up_process(VCHIQ_THREAD_T thread);
void flush_signals(VCHIQ_THREAD_T thread);
int fatal_signal_pending(VCHIQ_THREAD_T thread);


/* Logging and assertions */
#define printf				dprintf
#define vprintf				dvprintf
#define printk(fmt, ...)	dprintf(fmt, ##__VA_ARGS__)
#define KERN_EMERG		""
#define KERN_ALERT		""
#define KERN_CRIT		""
#define KERN_ERR		""
#define KERN_WARNING	""
#define KERN_NOTICE		""
#define KERN_INFO		""
#define KERN_DEBUG		""
#define KERN_CONT		""

#define might_sleep()

#define WARN(condition, msg)							\
({														\
	int __ret_warn_on = !!(condition);					\
	if (unlikely(__ret_warn_on))						\
		dprintf("vchiq: %s", (msg));					\
	unlikely(__ret_warn_on);							\
})

#define WARN_ON(condition)								\
({														\
	int __ret_warn_on = !!(condition);					\
	if (unlikely(__ret_warn_on))						\
		dprintf("vchiq: WARN_ON: " #condition "\n");	\
	unlikely(__ret_warn_on);							\
})

#define WARN_ON_ONCE(condition)							\
({														\
	static int __warned;								\
	int __ret_warn_once = !!(condition);				\
	if (unlikely(__ret_warn_once) && !__warned) {		\
		__warned = 1;									\
		WARN_ON(1);										\
	}													\
	unlikely(__ret_warn_once);							\
})

#define BUG_ON(condition)								\
	do {												\
		if (condition)									\
			panic("vchiq: BUG_ON: " #condition);		\
	} while (0)

#define BUG()											\
	panic("vchiq: BUG: %s:%d", __FILE__, __LINE__)


/* Memory */
#define GFP_KERNEL				0
#define GFP_ATOMIC				0

#define kmalloc(size, flags)	calloc(1, (size))
#define kzalloc(size, flags)	calloc(1, (size))
#define kcalloc(n, size, flags)	calloc((n), (size))
#define kfree(p)				free(p)

#ifndef PAGE_SIZE
#	define PAGE_SIZE			B_PAGE_SIZE
#endif
#ifndef PAGE_ALIGN
#	define PAGE_ALIGN(x)		(((x) + B_PAGE_SIZE - 1) & ~(B_PAGE_SIZE - 1))
#endif


/* Time */
#define HZ						1000
#define jiffies					((unsigned long)(system_time() / 1000))
#define msecs_to_jiffies(ms)	((unsigned long)(ms))
#define udelay(us)				spin(us)
#define mdelay(ms)				spin((bigtime_t)(ms) * 1000)
#define msleep(ms)				snooze((bigtime_t)(ms) * 1000)

#define time_after(a, b)		((long)((b) - (a)) < 0)
#define time_after_eq(a, b)		((long)((a) - (b)) >= 0)
#define time_before(a, b)		time_after(b, a)


/* Kernel module boilerplate and the rest */
#define __init
#define __exit
#define __devinit
#define __devexit
#define __devinitdata
#define __user
#define EXPORT_SYMBOL(x)

#ifndef __unused
#	define __unused				__attribute__((__unused__))
#endif
#define likely(x)				__builtin_expect(!!(x), 1)
#define unlikely(x)				__builtin_expect(!!(x), 0)

typedef off_t loff_t;


#ifdef __cplusplus
}
#endif


#endif	/* VCHI_HAIKU_H */
