/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	A kernel client of the firmware's general command service, "GCMD" (what
	Raspberry Pi OS's vcgencmd talks to): a command goes out as one message,
	the text with its NUL; the reply is one message, the firmware's 32-bit
	result code followed by the text (Broadcom's userland,
	interface/vmcs_host/vc_vchi_gencmd.c).

	The driver uses it to show the firmware's version in its read(), the
	first proof that messages go both ways.
*/


#include <interface/compat/vchi_haiku.h>

#include "vchiq_core.h"

#include "vchiq_haiku.h"


#define GENCMD_SERVICE		VCHIQ_MAKE_FOURCC('G', 'C', 'M', 'D')
#define GENCMD_VERSION		1
#define GENCMD_MAX_COMMAND	512

static const bigtime_t kReplyTimeout = 2000000;


static struct {
	mutex					lock;
		/* one command at a time */
	VCHIQ_INSTANCE_T		instance;
	VCHIQ_SERVICE_HANDLE_T	service;
	sem_id					reply_ready;
	char					reply[VCHIQ_MAX_MSG_SIZE];
	size_t					reply_size;
} sGencmd = {
	MUTEX_INITIALIZER("vchiq gencmd"),
	NULL,
	0,
	-1,
	{},
	0
};


static VCHIQ_STATUS_T
gencmd_callback(VCHIQ_REASON_T reason, VCHIQ_HEADER_T *header,
	VCHIQ_SERVICE_HANDLE_T service, void *bulkUserdata)
{
	if (reason != VCHIQ_MESSAGE_AVAILABLE)
		return VCHIQ_SUCCESS;

	size_t size = header->size;
	if (size > sizeof(sGencmd.reply))
		size = sizeof(sGencmd.reply);
	memcpy(sGencmd.reply, header->data, size);
	sGencmd.reply_size = size;
	vchiq_release_message(service, header);

	release_sem_etc(sGencmd.reply_ready, 1, B_DO_NOT_RESCHEDULE);
	return VCHIQ_SUCCESS;
}


static ssize_t
copy_command(void *context, void *dest, size_t offset, size_t maxSize)
{
	const char *command = context;
	size_t size = strlen(command) + 1 - offset;
	if (size > maxSize)
		size = maxSize;
	memcpy(dest, command + offset, size);
	return size;
}


/*!	A kernel instance with the service open, kept for later commands.
	Called with the lock held.
*/
static status_t
open_service(void)
{
	if (sGencmd.instance != NULL)
		return B_OK;

	if (sGencmd.reply_ready < 0) {
		sGencmd.reply_ready = create_sem(0, "vchiq gencmd reply");
		if (sGencmd.reply_ready < 0)
			return sGencmd.reply_ready;
	}

	VCHIQ_INSTANCE_T instance;
	if (vchiq_initialise(&instance) != VCHIQ_SUCCESS)
		return B_DEV_NOT_READY;
	if (vchiq_connect(instance) != VCHIQ_SUCCESS) {
		vchiq_shutdown(instance);
		return B_DEV_NOT_READY;
	}

	VCHIQ_SERVICE_PARAMS_T params = {
		.fourcc = GENCMD_SERVICE,
		.callback = gencmd_callback,
		.userdata = NULL,
		.version = GENCMD_VERSION,
		.version_min = GENCMD_VERSION
	};
	VCHIQ_SERVICE_HANDLE_T service;
	if (vchiq_open_service(instance, &params, &service) != VCHIQ_SUCCESS) {
		vchiq_shutdown(instance);
		return B_NAME_NOT_FOUND;
	}

	sGencmd.instance = instance;
	sGencmd.service = service;
	return B_OK;
}


status_t
vchiq_haiku_gencmd(const char *command, char *reply, size_t size,
	int32 *_result)
{
	if (strlen(command) >= GENCMD_MAX_COMMAND || size == 0)
		return B_BAD_VALUE;

	mutex_lock(&sGencmd.lock);

	status_t status = open_service();
	if (status != B_OK) {
		mutex_unlock(&sGencmd.lock);
		return status;
	}

	// a reply that came after an earlier command timed out
	while (acquire_sem_etc(sGencmd.reply_ready, 1, B_RELATIVE_TIMEOUT, 0)
			== B_OK) {
	}

	vchiq_use_service(sGencmd.service);
	if (vchiq_queue_message(sGencmd.service, copy_command, (void *)command,
			strlen(command) + 1) != VCHIQ_SUCCESS) {
		status = B_IO_ERROR;
	} else {
		status = acquire_sem_etc(sGencmd.reply_ready, 1,
			B_CAN_INTERRUPT | B_RELATIVE_TIMEOUT, kReplyTimeout);
	}
	vchiq_release_service(sGencmd.service);

	if (status == B_OK && sGencmd.reply_size < sizeof(int32))
		status = B_BAD_DATA;
	if (status == B_OK) {
		int32 result;
		memcpy(&result, sGencmd.reply, sizeof(result));
		if (_result != NULL)
			*_result = result;

		size_t length = sGencmd.reply_size - sizeof(int32);
		if (length > size - 1)
			length = size - 1;
		memcpy(reply, sGencmd.reply + sizeof(int32), length);
		reply[length] = '\0';
	}

	mutex_unlock(&sGencmd.lock);
	return status;
}
