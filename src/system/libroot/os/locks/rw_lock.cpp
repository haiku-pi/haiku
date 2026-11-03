/*
 * Copyright 2009, Michael Lotz, mmlr@mlotz.ch.
 * Distributed under the terms of the MIT License.
 */


#include <locks.h>

#include <OS.h>

#include <syscalls.h>
#include <user_thread.h>


typedef struct rw_lock_waiter {
	rw_lock_waiter *	next;
	thread_id			thread;
	bool				writer;
	bool				queued;
	status_t			status;
} rw_lock_waiter;


static status_t
rw_lock_wait(rw_lock *lock, bool writer)
{
	rw_lock_waiter waiter;
	waiter.thread = find_thread(NULL);
	waiter.next = NULL;
	waiter.writer = writer;
	waiter.queued = true;
	waiter.status = B_OK;

	// The waiter is dequeued before this function returns (see below), which
	// GCC cannot see
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wdangling-pointer"
	if (lock->waiters != NULL)
		lock->last_waiter->next = &waiter;
	else
		lock->waiters = &waiter;

	lock->last_waiter = &waiter;
	#pragma GCC diagnostic pop

	// The rw_lock is locked when entering; it is released while blocking.
	// Whoever dequeues the waiter (rw_lock_unblock(), __rw_lock_destroy())
	// holds it, sets the waiter's status and unblocks the thread. A block can
	// also end without that, interrupted (a signal, resume_thread()): then
	// block again, with the wait status set anew. The kernel ignores an
	// unblock while the wait status is not positive, so blocking with the
	// one an interruption left would return at once forever and miss the
	// wake-up of the dequeue, keeping the lock it was given.
	while (waiter.queued) {
		get_user_thread()->wait_status = 1;
		mutex_unlock(&lock->lock);

		_kern_block_thread(0, 0);

		mutex_lock(&lock->lock);
	}

	return waiter.status;
}


static void
rw_lock_unblock(rw_lock *lock)
{
	// this is called locked
	if (lock->holder >= 0)
		return;

	rw_lock_waiter *waiter = lock->waiters;
	if (waiter == NULL)
		return;

	if (waiter->writer) {
		if (lock->reader_count > 0)
			return;

		lock->waiters = waiter->next;
		lock->holder = waiter->thread;
		waiter->queued = false;
		waiter->status = B_OK;
		_kern_unblock_thread(waiter->thread, B_OK);
		return;
	}

	while (waiter != NULL && !waiter->writer) {
		lock->reader_count++;
		lock->waiters = waiter->next;
		waiter->queued = false;
		waiter->status = B_OK;
		_kern_unblock_thread(waiter->thread, B_OK);
		waiter = lock->waiters;
	}
}


void
__rw_lock_init(rw_lock *lock, const char *name)
{
	rw_lock_init_etc(lock, name, 0);
}


void
__rw_lock_init_etc(rw_lock *lock, const char *name, uint32 flags)
{
	lock->waiters = NULL;
	lock->holder = -1;
	lock->reader_count = 0;
	lock->writer_count = 0;
	lock->owner_count = 0;
	mutex_init_etc(&lock->lock, name, flags);
}


void
__rw_lock_destroy(rw_lock *lock)
{
	mutex_lock(&lock->lock);

	rw_lock_waiter *waiter = lock->waiters;
	while (waiter != NULL) {
		rw_lock_waiter *next = waiter->next;
		waiter->queued = false;
		waiter->status = B_ERROR;
		_kern_unblock_thread(waiter->thread, B_ERROR);
		waiter = next;
	}

	mutex_destroy(&lock->lock);
}


status_t
__rw_lock_read_lock(rw_lock *lock)
{
	MutexLocker locker(lock->lock);

	if (lock->writer_count == 0) {
		lock->reader_count++;
		return B_OK;
	}

	if (lock->holder == find_thread(NULL)) {
		lock->owner_count++;
		return B_OK;
	}

	return rw_lock_wait(lock, false);
}


status_t
__rw_lock_read_unlock(rw_lock *lock)
{
	MutexLocker locker(lock->lock);

	if (lock->holder == find_thread(NULL)) {
		if (--lock->owner_count > 0)
			return B_OK;

		// this originally has been a write lock
		lock->writer_count--;
		lock->holder = -1;

		rw_lock_unblock(lock);
		return B_OK;
	}

	if (lock->reader_count <= 0) {
		debugger("rw_lock not read locked");
		return B_ERROR;
	}

	lock->reader_count--;
	rw_lock_unblock(lock);
	return B_OK;
}


status_t
__rw_lock_write_lock(rw_lock *lock)
{
	MutexLocker locker(lock->lock);

	if (lock->reader_count == 0 && lock->writer_count == 0) {
		lock->writer_count++;
		lock->holder = find_thread(NULL);
		lock->owner_count = 1;
		return B_OK;
	}

	if (lock->holder == find_thread(NULL)) {
		lock->owner_count++;
		return B_OK;
	}

	lock->writer_count++;

	status_t result = rw_lock_wait(lock, true);
	if (result != B_OK)
		return result;

	if (lock->holder != find_thread(NULL)) {
		debugger("write locked but holder not set");
		return B_ERROR;
	}

	lock->owner_count = 1;
	return B_OK;
}


status_t
__rw_lock_write_unlock(rw_lock *lock)
{
	MutexLocker locker(lock->lock);

	if (lock->holder != find_thread(NULL)) {
		debugger("rw_lock not write locked");
		return B_ERROR;
	}

	if (--lock->owner_count > 0)
		return B_OK;

	lock->writer_count--;
	lock->holder = -1;
	rw_lock_unblock(lock);
	return B_OK;
}
