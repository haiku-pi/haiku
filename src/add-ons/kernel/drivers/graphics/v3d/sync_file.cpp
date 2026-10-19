/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Sync files: a fence as a file descriptor, as Linux' sync_file. Mesa
	moves fences between syncobjs through them (SYNCOBJ_HANDLE_TO_FD and
	SYNCOBJ_FD_TO_HANDLE with the sync file flags) and merges them
	(SYNC_IOC_MERGE): Vulkan's queue does so for every submission that
	signals a fence or semaphore.

	A fence is a job's sequence number (see v3d_syncobj), so a sync file
	holds one, a merge keeps the later of the two, and it signals once that
	job has completed; select()/poll() report it readable then.

	The descriptors are of their own kind (not opened from devfs), in the
	team that asked for them; they only refer to the device, which is never
	unloaded once it has been opened.
*/


#include "v3d_private.h"

#include <Select.h>

#include <fcntl.h>

#include <fs/fd.h>
#include <util/AutoLock.h>


// Linux' <linux/sync_file.h>
struct sync_merge_data {
	char				name[32];
	int32				fd2;
	int32				fence;
	uint32				flags;
	uint32				pad;
};

struct sync_fence_info {
	char				obj_name[32];
	char				driver_name[32];
	int32				status;
	uint32				flags;
	uint64				timestamp_ns;
};

struct sync_file_info {
	char				name[32];
	int32				status;
	uint32				flags;
	uint32				num_fences;
	uint32				pad;
	uint64				sync_fence_info;
};

enum {
	kSyncIoctlGroup		= '>',
	kSyncIoctlMerge		= 3,
	kSyncIoctlFileInfo	= 4,
};


struct v3d_sync_file {
	v3d_device*			device;
	uint64				seqno;
};


struct v3d_sync_file_select {
	v3d_sync_file_select* next;
	v3d_sync_file*		file;
	selectsync*			sync;
};


static status_t
merge(v3d_sync_file* file, void* buffer)
{
	sync_merge_data data;
	if (!IS_USER_ADDRESS(buffer)
		|| user_memcpy(&data, buffer, sizeof(data)) != B_OK) {
		return B_BAD_ADDRESS;
	}
	if (data.flags != 0 || data.pad != 0)
		return B_BAD_VALUE;

	uint64 other;
	status_t status = v3d_sync_file_get_seqno(file->device, data.fd2, &other);
	if (status != B_OK)
		return status;

	int fd;
	status = v3d_sync_file_create(file->device, max_c(file->seqno, other), &fd);
	if (status != B_OK)
		return status;

	data.fence = fd;
	if (user_memcpy(buffer, &data, sizeof(data)) != B_OK) {
		close_fd_index(get_current_io_context(false), fd);
		return B_BAD_ADDRESS;
	}
	return B_OK;
}


/*!	As Linux: with num_fences 0 only the count is returned, else the fence
	(there is one) is written to the array sync_fence_info points to.
*/
static status_t
file_info(v3d_sync_file* file, void* buffer)
{
	sync_file_info info;
	if (!IS_USER_ADDRESS(buffer)
		|| user_memcpy(&info, buffer, sizeof(info)) != B_OK) {
		return B_BAD_ADDRESS;
	}
	if (info.flags != 0 || info.pad != 0)
		return B_BAD_VALUE;

	const int32 status = v3d_seqno_completed(file->device, file->seqno) ? 1 : 0;
	if (info.num_fences != 0) {
		sync_fence_info fence;
		memset(&fence, 0, sizeof(fence));
		strlcpy(fence.obj_name, "v3d", sizeof(fence.obj_name));
		strlcpy(fence.driver_name, "v3d", sizeof(fence.driver_name));
		fence.status = status;

		void* fences = (void*)(addr_t)info.sync_fence_info;
		if (!IS_USER_ADDRESS(fences)
			|| user_memcpy(fences, &fence, sizeof(fence)) != B_OK) {
			return B_BAD_ADDRESS;
		}
	}

	strlcpy(info.name, "v3d", sizeof(info.name));
	info.status = status;
	info.num_fences = 1;
	if (user_memcpy(buffer, &info, sizeof(info)) != B_OK)
		return B_BAD_ADDRESS;
	return B_OK;
}


static status_t
sync_file_ioctl(file_descriptor* descriptor, ulong op, void* buffer,
	size_t length)
{
	v3d_sync_file* file = (v3d_sync_file*)descriptor->cookie;
	if (IOCGROUP(op) != kSyncIoctlGroup)
		return B_DEV_INVALID_IOCTL;

	switch (op & 0xff) {
		case kSyncIoctlMerge:
			return merge(file, buffer);
		case kSyncIoctlFileInfo:
			return file_info(file, buffer);
	}
	return B_DEV_INVALID_IOCTL;
}


static status_t
sync_file_select(file_descriptor* descriptor, uint8 event,
	selectsync* sync)
{
	v3d_sync_file* file = (v3d_sync_file*)descriptor->cookie;
	v3d_device* device = file->device;
	if (event != B_SELECT_READ)
		return B_BAD_VALUE;

	v3d_sync_file_select* entry
		= (v3d_sync_file_select*)malloc(sizeof(v3d_sync_file_select));
	if (entry == NULL)
		return B_NO_MEMORY;

	// Checked under the lock v3d_sync_file_notify() takes after the
	// completion: either this sees it, or that sees the entry.
	MutexLocker locker(device->select_lock);
	if (v3d_seqno_completed(device, file->seqno)) {
		free(entry);
		return notify_select_event(sync, event);
	}
	entry->file = file;
	entry->sync = sync;
	entry->next = device->selects;
	device->selects = entry;
	return B_OK;
}


/*!	Removes the entries of \a file (of \a sync only, unless NULL). */
static void
remove_selects(v3d_sync_file* file, selectsync* sync)
{
	v3d_device* device = file->device;
	MutexLocker locker(device->select_lock);
	v3d_sync_file_select** link = &device->selects;
	while (*link != NULL) {
		v3d_sync_file_select* entry = *link;
		if (entry->file == file && (sync == NULL || entry->sync == sync)) {
			*link = entry->next;
			free(entry);
		} else
			link = &entry->next;
	}
}


static status_t
sync_file_deselect(file_descriptor* descriptor, uint8 event,
	selectsync* sync)
{
	remove_selects((v3d_sync_file*)descriptor->cookie, sync);
	return B_OK;
}


static void
sync_file_free(file_descriptor* descriptor)
{
	v3d_sync_file* file = (v3d_sync_file*)descriptor->cookie;
	// (closing deselected everything already)
	remove_selects(file, NULL);
	free(file);
}


static struct fd_ops sSyncFileOps = {
	NULL,		// close
	&sync_file_free,
	NULL,		// read
	NULL,		// write
	NULL,		// readv
	NULL,		// writev
	NULL,		// seek
	&sync_file_ioctl,
	NULL,		// set_flags
	&sync_file_select,
	&sync_file_deselect,
	NULL,		// read_dir
	NULL,		// rewind_dir
	NULL,		// read_stat
	NULL,		// write_stat
};


//	#pragma mark - driver interface


/*!	A new sync file of the calling team for the fence \a seqno. Must not be
	called with the device lock held (the VFS locks the team's I/O context).
*/
status_t
v3d_sync_file_create(v3d_device* device, uint64 seqno, int* _fd)
{
	v3d_sync_file* file = (v3d_sync_file*)malloc(sizeof(v3d_sync_file));
	if (file == NULL)
		return B_NO_MEMORY;
	file->device = device;
	file->seqno = seqno;

	file_descriptor* descriptor = alloc_fd();
	if (descriptor == NULL) {
		free(file);
		return B_NO_MEMORY;
	}
	descriptor->ops = &sSyncFileOps;
	descriptor->cookie = file;
	descriptor->open_mode = O_RDWR;

	io_context* context = get_current_io_context(false);
	int fd = new_fd(context, descriptor);
	if (fd < 0) {
		put_fd(descriptor);
			// frees the file
		return fd;
	}

	// as Linux: close-on-exec
	rw_lock_write_lock(&context->lock);
	fd_set_close_on_exec(context, fd, true);
	rw_lock_write_unlock(&context->lock);

	*_fd = fd;
	return B_OK;
}


/*!	The fence of the calling team's sync file \a fd (of this device). Must
	not be called with the device lock held.
*/
status_t
v3d_sync_file_get_seqno(v3d_device* device, int fd, uint64* _seqno)
{
	file_descriptor* descriptor = get_fd(get_current_io_context(false), fd);
	if (descriptor == NULL)
		return B_FILE_ERROR;

	status_t status = B_BAD_VALUE;
	if (descriptor->ops == &sSyncFileOps) {
		v3d_sync_file* file = (v3d_sync_file*)descriptor->cookie;
		if (file->device == device) {
			*_seqno = file->seqno;
			status = B_OK;
		}
	}
	put_fd(descriptor);
	return status;
}


/*!	Tells select()/poll() about sync files that signalled. Called after a
	job completed, with the device lock held.
*/
void
v3d_sync_file_notify(v3d_device* device)
{
	MutexLocker locker(device->select_lock);
	v3d_sync_file_select** link = &device->selects;
	while (*link != NULL) {
		v3d_sync_file_select* entry = *link;
		if (v3d_seqno_completed(device, entry->file->seqno)) {
			notify_select_event(entry->sync, B_SELECT_READ);
			*link = entry->next;
			free(entry);
		} else
			link = &entry->next;
	}
}
