/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Linux kernel primitives for the VCHIQ core, on Haiku's (see
	vchi_haiku.h).
*/


#include <new>

#include <condition_variable.h>
#include <thread.h>
#include <util/AutoLock.h>

#include <interface/compat/vchi_haiku.h>


static_assert(sizeof(ConditionVariable) <= sizeof(vchiq_condition),
	"vchiq_condition cannot hold a ConditionVariable");
static_assert(alignof(ConditionVariable) <= alignof(vchiq_condition),
	"vchiq_condition is not aligned for a ConditionVariable");


static inline ConditionVariable*
condition(vchiq_condition& storage)
{
	return (ConditionVariable*)storage.storage;
}


static void
init_condition(vchiq_condition& storage, const void* object,
	const char* name)
{
	ConditionVariable* variable = new(storage.storage) ConditionVariable;
	variable->Init(object, name);
}


/*!	Waits, with \a lock held, until \a count is not 0 (or for ever when it
	is negative, a completion that is complete for all), and takes one.
	Returns B_OK, or B_INTERRUPTED, B_TIMED_OUT... with nothing taken.
*/
static status_t
take(spinlock& lock, int32& count, vchiq_condition& storage, uint32 flags,
	bigtime_t timeout)
{
	InterruptsSpinLocker locker(lock);
	if ((flags & B_RELATIVE_TIMEOUT) != 0 && timeout > 0) {
		timeout += system_time();
		flags = (flags & ~B_RELATIVE_TIMEOUT) | B_ABSOLUTE_TIMEOUT;
	}

	while (count == 0) {
		if ((flags & B_RELATIVE_TIMEOUT) != 0)
			return B_WOULD_BLOCK;

		ConditionVariableEntry entry;
		condition(storage)->Add(&entry);
		locker.Unlock();
		status_t status = entry.Wait(flags, timeout);
		locker.Lock();

		if (status != B_OK && count == 0)
			return status;
		if (status != B_OK) {
			// we were woken up, or another waiter has to be
			condition(storage)->NotifyOne();
			return status;
		}
	}

	if (count > 0)
		count--;
	return B_OK;
}


//	#pragma mark - semaphores


void
_sema_init(struct semaphore* s, int value)
{
	B_INITIALIZE_SPINLOCK(&s->lock);
	s->count = value;
	init_condition(s->condition, s, "vchiq semaphore");
}


void
_sema_destroy(struct semaphore* s)
{
	condition(s->condition)->~ConditionVariable();
}


void
down(struct semaphore* s)
{
	take(s->lock, s->count, s->condition, 0, 0);
}


int
down_interruptible(struct semaphore* s)
{
	return take(s->lock, s->count, s->condition, B_CAN_INTERRUPT, 0) == B_OK
		? 0 : EINTR;
}


int
down_trylock(struct semaphore* s)
{
	return take(s->lock, s->count, s->condition, B_RELATIVE_TIMEOUT, 0)
		== B_OK ? 0 : EAGAIN;
}


/*!	May be called with interrupts disabled (from the doorbell handler). */
void
up(struct semaphore* s)
{
	InterruptsSpinLocker locker(s->lock);
	s->count++;
	condition(s->condition)->NotifyOne();
}


//	#pragma mark - completions


void
init_completion(struct completion* c)
{
	B_INITIALIZE_SPINLOCK(&c->lock);
	c->done = 0;
	init_condition(c->condition, c, "vchiq completion");
}


void
destroy_completion(struct completion* c)
{
	condition(c->condition)->~ConditionVariable();
}


void
complete(struct completion* c)
{
	InterruptsSpinLocker locker(c->lock);
	if (c->done >= 0) {
		c->done++;
		condition(c->condition)->NotifyOne();
	}
}


void
complete_all(struct completion* c)
{
	InterruptsSpinLocker locker(c->lock);
	c->done = -1;
	condition(c->condition)->NotifyAll();
}


void
INIT_COMPLETION_locked(struct completion* c)
{
	InterruptsSpinLocker locker(c->lock);
	c->done = 0;
}


void
wait_for_completion(struct completion* c)
{
	take(c->lock, c->done, c->condition, 0, 0);
}


/*!	Returns 1 when the completion was done (and claims it), 0 otherwise. */
int
try_wait_for_completion(struct completion* c)
{
	return take(c->lock, c->done, c->condition, B_RELATIVE_TIMEOUT, 0)
		== B_OK;
}


int
wait_for_completion_interruptible(struct completion* c)
{
	return take(c->lock, c->done, c->condition, B_CAN_INTERRUPT, 0) == B_OK
		? 0 : EINTR;
}


/*!	Returns the jiffies left (at least 1) when completed, 0 on timeout, or
	EINTR (negative) when interrupted.
*/
long
wait_for_completion_interruptible_timeout(struct completion* c,
	unsigned long timeout)
{
	if (timeout == 0)
		return try_wait_for_completion(c);

	bigtime_t start = system_time();
	status_t status = take(c->lock, c->done, c->condition,
		B_CAN_INTERRUPT | B_RELATIVE_TIMEOUT, (bigtime_t)timeout * 1000);
	if (status == B_TIMED_OUT || status == B_WOULD_BLOCK)
		return 0;
	if (status != B_OK)
		return EINTR;

	long left = (long)timeout - (long)((system_time() - start) / 1000);
	return left > 0 ? left : 1;
}


int
wait_for_completion_killable(struct completion* c)
{
	return take(c->lock, c->done, c->condition, B_KILL_CAN_INTERRUPT, 0)
		== B_OK ? 0 : EINTR;
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
	thread_start start = *(thread_start*)_start;
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

	thread_start* start = (thread_start*)malloc(sizeof(thread_start));
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


int
fatal_signal_pending(VCHIQ_THREAD_T thread)
{
	return thread_is_interrupted(thread_get_current_thread(),
		B_KILL_CAN_INTERRUPT);
}
