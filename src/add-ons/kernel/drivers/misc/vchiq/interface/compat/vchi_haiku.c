/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Linux kernel primitives for the VCHIQ core, on Haiku's (see vchi_haiku.h).
	Thread and signal helpers that need the kernel's private C++ headers are
	in vchiq_haiku.cpp.
*/


#include <interface/compat/vchi_haiku.h>


//	#pragma mark - semaphores


void
_sema_init(struct semaphore* s, int value)
{
	s->id = create_sem(value, "vchiq semaphore");
	if (s->id < 0)
		panic("vchiq: cannot create a semaphore: %s", strerror(s->id));
}


void
_sema_destroy(struct semaphore* s)
{
	if (s->id >= 0)
		delete_sem(s->id);
	s->id = -1;
}


void
down(struct semaphore* s)
{
	acquire_sem(s->id);
}


int
down_interruptible(struct semaphore* s)
{
	return acquire_sem_etc(s->id, 1, B_CAN_INTERRUPT, 0) == B_OK ? 0 : EINTR;
}


int
down_trylock(struct semaphore* s)
{
	return acquire_sem_etc(s->id, 1, B_RELATIVE_TIMEOUT, 0) == B_OK
		? 0 : EAGAIN;
}


/*!	May be called with interrupts disabled (from the doorbell handler). */
void
up(struct semaphore* s)
{
	release_sem_etc(s->id, 1, B_DO_NOT_RESCHEDULE);
}


//	#pragma mark - completions


/*!	A completion is a semaphore that counts complete() calls; complete_all()
	sets "all" and releases far more than anyone waits for, so that every
	present and later wait succeeds until INIT_COMPLETION.
*/
static const int32 kCompleteAllCount = 0x10000000;


void
init_completion(struct completion* c)
{
	c->all = 0;
	c->sem = create_sem(0, "vchiq completion");
	if (c->sem < 0)
		panic("vchiq: cannot create a semaphore: %s", strerror(c->sem));
}


void
destroy_completion(struct completion* c)
{
	if (c->sem >= 0)
		delete_sem(c->sem);
	c->sem = -1;
}


void
complete(struct completion* c)
{
	if (atomic_get(&c->all) == 0)
		release_sem_etc(c->sem, 1, B_DO_NOT_RESCHEDULE);
}


void
complete_all(struct completion* c)
{
	if (atomic_get_and_set(&c->all, 1) == 0)
		release_sem_etc(c->sem, kCompleteAllCount, 0);
}


void
INIT_COMPLETION_locked(struct completion* c)
{
	if (atomic_get_and_set(&c->all, 0) != 0) {
		delete_sem(c->sem);
		init_completion(c);
		return;
	}
	while (acquire_sem_etc(c->sem, 1, B_RELATIVE_TIMEOUT, 0) == B_OK)
		;
}


static void
completion_claimed(struct completion* c)
{
	// keep complete_all()'s count from running out
	if (atomic_get(&c->all) != 0)
		release_sem_etc(c->sem, 1, B_DO_NOT_RESCHEDULE);
}


void
wait_for_completion(struct completion* c)
{
	acquire_sem(c->sem);
	completion_claimed(c);
}


/*!	Returns 1 when the completion was done (and claims it), 0 otherwise. */
int
try_wait_for_completion(struct completion* c)
{
	if (acquire_sem_etc(c->sem, 1, B_RELATIVE_TIMEOUT, 0) != B_OK)
		return 0;
	completion_claimed(c);
	return 1;
}


int
wait_for_completion_interruptible(struct completion* c)
{
	if (acquire_sem_etc(c->sem, 1, B_CAN_INTERRUPT, 0) != B_OK)
		return EINTR;
	completion_claimed(c);
	return 0;
}


/*!	Returns the jiffies left (at least 1) when completed, 0 on timeout, or
	EINTR (negative) when interrupted.
*/
long
wait_for_completion_interruptible_timeout(struct completion* c,
	unsigned long timeout)
{
	bigtime_t start = system_time();
	status_t status = acquire_sem_etc(c->sem, 1,
		B_CAN_INTERRUPT | B_RELATIVE_TIMEOUT, (bigtime_t)timeout * 1000);
	if (status == B_TIMED_OUT || status == B_WOULD_BLOCK)
		return 0;
	if (status != B_OK)
		return EINTR;
	completion_claimed(c);

	long left = (long)timeout - (long)((system_time() - start) / 1000);
	return left > 0 ? left : 1;
}


int
wait_for_completion_killable(struct completion* c)
{
	if (acquire_sem_etc(c->sem, 1, B_KILL_CAN_INTERRUPT, 0) != B_OK)
		return EINTR;
	completion_claimed(c);
	return 0;
}


//	#pragma mark - timers


void
vchiq_init_timer(struct timer_list* t)
{
	t->expires = 0;
}


void
vchiq_setup_timer(struct timer_list* t, void (*function)(unsigned long),
	unsigned long data)
{
	t->function = function;
	t->data = data;
	vchiq_init_timer(t);
}


void
vchiq_mod_timer(struct timer_list* t, unsigned long expires)
{
	dprintf("vchiq: timers are not supported (suspend timer started)\n");
}


void
vchiq_add_timer(struct timer_list* t)
{
	vchiq_mod_timer(t, t->expires);
}


int
vchiq_del_timer(struct timer_list* t)
{
	return 0;
}


int
vchiq_del_timer_sync(struct timer_list* t)
{
	return 0;
}


//	#pragma mark - kernel threads


struct thread_start {
	int		(*function)(void* data);
	void*	data;
};


static status_t
vchiq_thread_entry(void* _start)
{
	struct thread_start start = *(struct thread_start*)_start;
	free(_start);
	return start.function(start.data);
}


/*!	Linux's kthread_create(): the thread waits for wake_up_process(). */
VCHIQ_THREAD_T
vchiq_thread_create(int (*threadfn)(void* data), void* data,
	const char namefmt[], ...)
{
	char name[B_OS_NAME_LENGTH];
	va_list args;
	va_start(args, namefmt);
	vsnprintf(name, sizeof(name), namefmt, args);
	va_end(args);

	struct thread_start* start = malloc(sizeof(struct thread_start));
	if (start == NULL)
		return NULL;
	start->function = threadfn;
	start->data = data;

	thread_id thread = spawn_kernel_thread(vchiq_thread_entry, name,
		B_NORMAL_PRIORITY, start);
	if (thread < 0) {
		free(start);
		return NULL;
	}
	return (VCHIQ_THREAD_T)(addr_t)thread;
}


/*!	The core asks for nice -19 and -20 (the slot handler, the recycle and
	the sync thread): they carry every message.
*/
void
set_user_nice(VCHIQ_THREAD_T thread, int nice)
{
	set_thread_priority(VCHIQ_THREAD_ID(thread),
		nice < 0 ? B_URGENT_DISPLAY_PRIORITY : B_NORMAL_PRIORITY);
}


void
wake_up_process(VCHIQ_THREAD_T thread)
{
	resume_thread(VCHIQ_THREAD_ID(thread));
}


void
flush_signals(VCHIQ_THREAD_T thread)
{
	// kernel threads get no signals
}
