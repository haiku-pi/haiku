/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's HEVC decoder block, "rpivid" ("brcm,bcm2711-hevc-dec").

	A stateless, register-driven decoder: the client parses the stream and
	hands over each picture as a phase-1 command buffer (see <rpivid.h> and
	decode.cpp, which runs the block's two phases as a pipeline in a thread of
	its own); this driver owns the memory the block reads and writes, its
	clock and its interrupt. It publishes /dev/video/rpivid/0; reading the
	device returns the block's identification.

	Nothing touches the block while the system boots: an MMIO access to the
	block without its clock does not complete. The first open() switches the
	clock on (through the VideoCore firmware: the block needs no other power
	or reset), checks the version register and installs the interrupt
	handler; the last close switches the clock off again.
*/


#include "rpivid_private.h"

#include <ByteOrder.h>
#include <Drivers.h>
#include <Errors.h>
#include <bus/FDT.h>
#include <arch/cpu.h>
#include <kernel.h>
#include <team.h>
#include <util/AutoLock.h>
#include <vm/vm.h>

#include <stdio.h>

#include <bcm2835_mailbox.h>


#define RPIVID_MODULE_NAME "drivers/video/rpivid/driver_v1"
#define RPIVID_DEVICE_MODULE_NAME "drivers/video/rpivid/device_v1"
#define RPIVID_DEVICE_NAME "video/rpivid/0"

static const char* kCompatible = "brcm,bcm2711-hevc-dec";

// A bitstream buffer beyond this is not a picture's slices
static const size_t kMaxBitstreamSize = 64 * 1024 * 1024;


static device_manager_info* sDeviceManager;
static bcm2835_mailbox_module_info* sMailbox;


//	#pragma mark - memory


uint32
rpivid_bus_address(rpivid_device* device, phys_addr_t physical)
{
	// address registers hold the bus address in 64-byte units
	return (uint32)((physical + device->bus_offset) >> 6);
}


/*!	Contiguous memory the block reaches, cleared. \a memoryType is
	B_WRITE_COMBINING_MEMORY or 0 (cached).
*/
status_t
rpivid_dma_alloc(rpivid_device* device, const char* name, size_t size,
	uint32 memoryType, dma_memory& memory)
{
	memory.area = -1;
	size = ROUNDUP(size, B_PAGE_SIZE);

	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = device->dma_end;

	void* address;
	area_id area = create_area_etc(B_SYSTEM_TEAM, name, size, B_CONTIGUOUS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, &address);
	if (area < 0)
		return area;

	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, address, B_PAGE_SIZE,
		&entry, &count);
	if (status == B_BUFFER_OVERFLOW)
		status = B_OK;
	if (status == B_OK) {
		// The pages were cleared through the cacheable mapping: write those
		// lines back and drop them, so that no dirty line can be evicted over
		// what the block writes later.
		arm64_dcache_flush_range((addr_t)address, size);
		if (memoryType != 0)
			status = vm_set_area_memory_type(area, entry.address, memoryType);
	}
	if (status == B_OK && entry.address + size > device->dma_end)
		status = B_NO_MEMORY;
	if (status != B_OK) {
		delete_area(area);
		return status;
	}

	memory.area = area;
	memory.address = address;
	memory.physical = entry.address;
	memory.size = size;
	return B_OK;
}


void
rpivid_dma_free(dma_memory& memory)
{
	if (memory.area >= 0)
		delete_area(memory.area);
	memory.area = -1;
	memory.address = NULL;
	memory.size = 0;
}


/*!	Maps \a buffer into the calling team. A clone of RAM does not inherit the
	memory type of its source: write-combining is set again.
*/
static status_t
map_into_client(rpivid_buffer* buffer, uint32 protection, uint32 memoryType,
	void** _address)
{
	team_id team = team_get_current_team_id();
	void* address = NULL;
	area_id clone = vm_clone_area(team, "rpivid buffer", &address,
		B_ANY_ADDRESS, protection, REGION_NO_PRIVATE_MAP, buffer->memory.area,
		true);
	if (clone < 0)
		return clone;

	if (memoryType != 0) {
		status_t status = vm_set_area_memory_type(clone,
			buffer->memory.physical, memoryType);
		if (status != B_OK) {
			vm_delete_area(team, clone, true);
			return status;
		}
		// the clone was mapped cacheable for a moment: drop what that may
		// have pulled in (the lines are clean)
		arm64_dcache_flush_range((addr_t)buffer->memory.address,
			buffer->memory.size);
	}

	buffer->client_area = clone;
	buffer->client_team = team;
	*_address = address;
	return B_OK;
}


static void
buffer_free(rpivid_buffer* buffer)
{
	if (buffer->pinned) {
		// a block that stopped responding may still write it
		free(buffer);
		return;
	}
	rpivid_dma_free(buffer->memory);
	free(buffer);
}


/*!	Drops a reference to \a buffer (the file's handle or a job's); the last
	one frees it. Called with the device lock held.
*/
void
rpivid_buffer_release(rpivid_buffer* buffer)
{
	if (--buffer->references == 0)
		buffer_free(buffer);
}


rpivid_buffer*
rpivid_buffer_lookup(rpivid_file* file, uint32 handle, rpivid_buffer_type type)
{
	if (handle == 0 || handle > file->buffer_slots)
		return NULL;
	rpivid_buffer* buffer = file->buffers[handle - 1];
	if (buffer == NULL || buffer->type != type)
		return NULL;
	return buffer;
}


static status_t
handle_alloc(rpivid_file* file, rpivid_buffer* buffer, uint32* _handle)
{
	for (uint32 i = 0; i < file->buffer_slots; i++) {
		if (file->buffers[i] == NULL) {
			file->buffers[i] = buffer;
			*_handle = i + 1;
			return B_OK;
		}
	}

	uint32 count = file->buffer_slots == 0 ? 32 : file->buffer_slots * 2;
	rpivid_buffer** slots = (rpivid_buffer**)calloc(count,
		sizeof(rpivid_buffer*));
	if (slots == NULL)
		return B_NO_MEMORY;
	if (file->buffer_slots > 0) {
		memcpy(slots, file->buffers,
			file->buffer_slots * sizeof(rpivid_buffer*));
	}
	free(file->buffers);

	slots[file->buffer_slots] = buffer;
	*_handle = file->buffer_slots + 1;
	file->buffers = slots;
	file->buffer_slots = count;
	return B_OK;
}


/*!	The SAND (column 128) geometry the block writes: each plane in columns
	of 128 bytes holding the picture height (rounded to 16 rows) of luma, or
	half of it of interleaved chroma. 8 bit: 128 samples per column; 10 bit:
	96 (three in each 32-bit word). Then the collocated motion vectors, as
	Linux sizes them.
*/
static void
picture_geometry(rpivid_buffer* picture)
{
	const uint32 height16 = ROUNDUP(picture->height, 16);
	uint32 columnBytes;
	if (picture->bit_depth == 8)
		columnBytes = ROUNDUP(picture->width, 128);
	else
		columnBytes = ROUNDUP((picture->width + 2) / 3, 32) * 4;

	picture->columns = columnBytes / 128;
	picture->luma_stride = height16 * 128;
	picture->chroma_stride = picture->luma_stride / 2;

	const size_t lumaSize = (size_t)picture->luma_stride * picture->columns;
	const size_t chromaSize = (size_t)picture->chroma_stride
		* picture->columns;
	picture->chroma_offset = ROUNDUP(lumaSize, B_PAGE_SIZE);
	picture->planes_size = picture->chroma_offset + chromaSize;

	picture->mv_stride = ROUNDUP(picture->width, 64);
	const size_t mvSize = (size_t)picture->mv_stride
		* (ROUNDUP(picture->height, 64) / 16);
	picture->mv_offset = ROUNDUP(picture->planes_size, B_PAGE_SIZE);
	picture->memory.size = (picture->flags & RPIVID_PICTURE_MOTION_VECTORS)
			!= 0
		? picture->mv_offset + mvSize : picture->planes_size;
}


static status_t
create_buffer(rpivid_file* file, rpivid_create_buffer* args)
{
	rpivid_device* device = file->device;
	if (args->size == 0 || args->size > kMaxBitstreamSize)
		return B_BAD_VALUE;

	rpivid_buffer* buffer = (rpivid_buffer*)calloc(1, sizeof(rpivid_buffer));
	if (buffer == NULL)
		return B_NO_MEMORY;
	buffer->type = kBitstreamBuffer;
	buffer->references = 1;

	status_t status = rpivid_dma_alloc(device, "rpivid bitstream", args->size,
		B_WRITE_COMBINING_MEMORY, buffer->memory);
	void* address = NULL;
	if (status == B_OK) {
		status = map_into_client(buffer, B_READ_AREA | B_WRITE_AREA,
			B_WRITE_COMBINING_MEMORY, &address);
	}
	MutexLocker locker(device->lock);
	uint32 handle = 0;
	if (status == B_OK)
		status = handle_alloc(file, buffer, &handle);
	if (status != B_OK) {
		if (address != NULL)
			vm_delete_area(buffer->client_team, buffer->client_area, true);
		buffer_free(buffer);
		return status;
	}

	args->address = (addr_t)address;
	args->handle = handle;
	return B_OK;
}


static status_t
create_picture(rpivid_file* file, rpivid_create_picture* args)
{
	rpivid_device* device = file->device;
	if (args->width < 32 || args->width > 4096 || args->height < 32
		|| args->height > 4096 || (args->bit_depth != 8 && args->bit_depth != 10)
		|| (args->flags & ~(uint32)(RPIVID_PICTURE_MOTION_VECTORS
			| RPIVID_PICTURE_UNCACHED)) != 0) {
		return B_BAD_VALUE;
	}

	rpivid_buffer* picture = (rpivid_buffer*)calloc(1, sizeof(rpivid_buffer));
	if (picture == NULL)
		return B_NO_MEMORY;
	picture->type = kPictureBuffer;
	picture->references = 1;
	picture->width = args->width;
	picture->height = args->height;
	picture->bit_depth = args->bit_depth;
	picture->flags = args->flags;
	picture_geometry(picture);

	// The CPU only reads pictures: cached unless asked otherwise, and the
	// driver drops the CPU's lines after each decode into one.
	const uint32 memoryType = (args->flags & RPIVID_PICTURE_UNCACHED) != 0
		? B_WRITE_COMBINING_MEMORY : 0;
	status_t status = rpivid_dma_alloc(device, "rpivid picture",
		picture->memory.size, memoryType, picture->memory);
	void* address = NULL;
	if (status == B_OK)
		status = map_into_client(picture, B_READ_AREA, memoryType, &address);
	MutexLocker locker(device->lock);
	uint32 handle = 0;
	if (status == B_OK)
		status = handle_alloc(file, picture, &handle);
	if (status != B_OK) {
		if (address != NULL)
			vm_delete_area(picture->client_team, picture->client_area, true);
		buffer_free(picture);
		return status;
	}

	args->address = (addr_t)address;
	args->size = picture->memory.size;
	args->luma_offset = 0;
	args->chroma_offset = picture->chroma_offset;
	args->luma_column_stride = picture->luma_stride;
	args->chroma_column_stride = picture->chroma_stride;
	args->columns = picture->columns;
	args->handle = handle;
	return B_OK;
}


static status_t
delete_buffer(rpivid_file* file, rpivid_delete_buffer* args)
{
	rpivid_device* device = file->device;
	MutexLocker locker(device->lock);
	if (args->handle == 0 || args->handle > file->buffer_slots
		|| file->buffers[args->handle - 1] == NULL) {
		return B_BAD_VALUE;
	}
	rpivid_buffer* buffer = file->buffers[args->handle - 1];
	file->buffers[args->handle - 1] = NULL;
	locker.Unlock();

	if (buffer->client_team == team_get_current_team_id())
		vm_delete_area(buffer->client_team, buffer->client_area, true);

	// queued pictures may still use it: it goes with the last of them
	locker.Lock();
	rpivid_buffer_release(buffer);
	return B_OK;
}


//	#pragma mark - power


static status_t
firmware_call(uint32 tag, uint32 id, uint32 value, uint32 requestWords,
	uint32* _answer)
{
	uint32 values[3] = { id, value, 0 };
	status_t status = sMailbox->property_tag(tag, values, sizeof(values),
		requestWords * sizeof(uint32), NULL);
	if (status == B_OK && _answer != NULL)
		*_answer = values[1];
	return status;
}


/*!	The block's clock on, at the firmware's maximum rate (as Linux), and
	only B_OK when the firmware says it is on: the next step is an MMIO
	access.
*/
static status_t
clock_on(rpivid_device* device)
{
	uint32 maxRate = 0;
	status_t status = firmware_call(BCM2835_TAG_GET_MAX_CLOCK_RATE,
		device->clock_id, 0, 1, &maxRate);
	if (status == B_OK)
		status = firmware_call(BCM2835_TAG_SET_CLOCK_STATE, device->clock_id,
			1, 2, NULL);
	if (status == B_OK && maxRate != 0) {
		// clock ID, rate, skip setting turbo
		uint32 values[3] = { device->clock_id, maxRate, 0 };
		status = sMailbox->property_tag(BCM2835_TAG_SET_CLOCK_RATE, values,
			sizeof(values), sizeof(values), NULL);
	}

	uint32 state = 0, rate = 0;
	if (status == B_OK) {
		status = firmware_call(BCM2835_TAG_GET_CLOCK_STATE, device->clock_id,
			0, 1, &state);
	}
	firmware_call(BCM2835_TAG_GET_CLOCK_RATE, device->clock_id, 0, 1, &rate);
	if (status != B_OK || (state & 1) == 0 || rate == 0) {
		ERROR("clock %" B_PRIu32 " does not come on: %s, state %#" B_PRIx32
			", rate %" B_PRIu32 " (max %" B_PRIu32 ")\n", device->clock_id,
			strerror(status), state, rate, maxRate);
		return status != B_OK ? status : B_ERROR;
	}
	device->clock_rate = rate;
	return B_OK;
}


static void
clock_off(rpivid_device* device)
{
	firmware_call(BCM2835_TAG_SET_CLOCK_STATE, device->clock_id, 0, 2, NULL);
}


static status_t
map_registers(const char* name, phys_addr_t physical, size_t size,
	area_id& area, volatile uint32*& registers)
{
	void* address;
	area = map_physical_memory(name, physical, size, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	if (area < 0)
		return area;
	registers = (volatile uint32*)address;
	return B_OK;
}


static void
unmap_registers(rpivid_device* device)
{
	if (device->block_area >= 0)
		delete_area(device->block_area);
	if (device->intc_area >= 0)
		delete_area(device->intc_area);
	device->block_area = device->intc_area = -1;
	device->block = device->intc = NULL;
}


/*!	Clock on, version check, interrupts. Called with the lock held. */
static status_t
power_on(rpivid_device* device)
{
	if (device->powered)
		return B_OK;
	if (device->broken)
		return B_DEVICE_NOT_FOUND;

	status_t status = clock_on(device);
	if (status != B_OK)
		return status;

	status = map_registers("rpivid registers", device->block_physical,
		kBlockSize, device->block_area, device->block);
	if (status == B_OK) {
		status = map_registers("rpivid interrupts", device->intc_physical,
			B_PAGE_SIZE, device->intc_area, device->intc);
	}
	if (status != B_OK) {
		unmap_registers(device);
		clock_off(device);
		return status;
	}

	device->version = block_read(device, kVersion);
	if (device->version != kExpectedVersion) {
		ERROR("unexpected version %#" B_PRIx32 " (want %#" B_PRIx32 "), not "
			"using the block\n", device->version, kExpectedVersion);
		unmap_registers(device);
		clock_off(device);
		return B_DEVICE_NOT_FOUND;
	}

	// The status bits latch only once enabled: enable, then clear what is
	// pending (as Linux).
	device->intc[0] = kActive1Enable | kActive2Enable;
	device->intc[0] = device->intc[0];
	device->pending = 0;

	status = install_io_interrupt_handler(device->interrupt, &rpivid_interrupt,
		device, 0);
	if (status != B_OK) {
		ERROR("cannot install interrupt %" B_PRIu64 ": %s\n",
			device->interrupt, strerror(status));
		device->intc[0] = 0;
		unmap_registers(device);
		clock_off(device);
		return status;
	}

	device->powered = true;
	TRACE_ALWAYS("version %#" B_PRIx32 ", clock %" B_PRIu32 " MHz, interrupt %"
		B_PRIu64 "\n", device->version, device->clock_rate / 1000000,
		device->interrupt);
	return B_OK;
}


/*!	Interrupts off, clock off. Called with the lock held, no decode running. */
void
rpivid_power_off(rpivid_device* device)
{
	if (!device->powered)
		return;

	device->intc[0] = 0;
	remove_io_interrupt_handler(device->interrupt, &rpivid_interrupt, device);
	{
		InterruptsSpinLocker locker(device->irq_lock);
		device->intc = NULL;
	}
	unmap_registers(device);
	clock_off(device);
	device->powered = false;
}


//	#pragma mark - device module API


static status_t
rpivid_init_device(void* driverCookie, void** _cookie)
{
	*_cookie = driverCookie;
	return B_OK;
}


static void
rpivid_uninit_device(void* cookie)
{
}


static status_t
rpivid_open(void* deviceCookie, const char* path, int openMode, void** _cookie)
{
	rpivid_device* device = (rpivid_device*)deviceCookie;

	rpivid_file* file = (rpivid_file*)calloc(1, sizeof(rpivid_file));
	if (file == NULL)
		return B_NO_MEMORY;
	file->device = device;

	MutexLocker locker(device->lock);
	status_t status = power_on(device);
	if (status != B_OK) {
		free(file);
		return status;
	}
	device->open_count++;

	*_cookie = file;
	return B_OK;
}


static status_t
rpivid_close(void* cookie)
{
	return B_OK;
}


/*!	The file's pictures in the pipeline finish, its buffers go (the
	client's mappings of them were areas of its team), and with the last file
	the block's clock.
*/
static status_t
rpivid_free(void* cookie)
{
	rpivid_file* file = (rpivid_file*)cookie;
	rpivid_device* device = file->device;

	MutexLocker locker(device->lock);
	rpivid_file_jobs_free(file);
	for (uint32 i = 0; i < file->buffer_slots; i++) {
		if (file->buffers[i] != NULL)
			rpivid_buffer_release(file->buffers[i]);
	}
	free(file->buffers);
	free(file);

	if (--device->open_count == 0 && !device->broken) {
		rpivid_decode_free_memory(device);
		rpivid_power_off(device);
	}
	return B_OK;
}


static status_t
rpivid_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	rpivid_device* device = ((rpivid_file*)cookie)->device;

	char report[320];
	MutexLocker locker(device->lock);
	int length = snprintf(report, sizeof(report),
		"rpivid: version %#" B_PRIx32 ", clock %" B_PRIu32 " MHz (id %" B_PRIu32
		"), registers %#" B_PRIxPHYSADDR "/%#" B_PRIxPHYSADDR ", interrupt %"
		B_PRIu64 " (%" B_PRIu32 " taken), DMA below %#" B_PRIxPHYSADDR
		", %" B_PRIu64 " pictures decoded, %" B_PRIu64 " failed, %" B_PRIu64
		" phase overlaps%s\n",
		device->version, device->clock_rate / 1000000, device->clock_id,
		device->block_physical, device->intc_physical, device->interrupt,
		device->interrupts, device->dma_end, device->decoded, device->failed,
		device->overlaps,
		device->broken ? ", BROKEN (stopped responding)" : "");
	locker.Unlock();
	length = min_c(length, (int)sizeof(report) - 1);

	if (position >= length) {
		*_length = 0;
		return B_OK;
	}
	size_t count = min_c(*_length, (size_t)(length - position));
	if (user_memcpy(buffer, report + position, count) != B_OK)
		return B_BAD_ADDRESS;
	*_length = count;
	return B_OK;
}


static status_t
rpivid_write(void* cookie, off_t position, const void* buffer, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static status_t
get_info(rpivid_device* device, rpivid_info* info)
{
	MutexLocker locker(device->lock);
	memset(info, 0, sizeof(*info));
	info->api_version = RPIVID_API_VERSION;
	info->hardware_version = device->version;
	info->clock_rate = device->clock_rate;
	info->flags = device->broken ? RPIVID_INFO_BROKEN : 0;
	info->pictures_decoded = device->decoded;
	info->pictures_failed = device->failed;
	info->overlaps = device->overlaps;
	return B_OK;
}


static status_t
rpivid_control(void* cookie, uint32 op, void* arg, size_t length)
{
	rpivid_file* file = (rpivid_file*)cookie;

	size_t size;
	switch (op) {
		case RPIVID_GET_INFO:
			size = sizeof(rpivid_info);
			break;
		case RPIVID_CREATE_BUFFER:
			size = sizeof(rpivid_create_buffer);
			break;
		case RPIVID_CREATE_PICTURE:
			size = sizeof(rpivid_create_picture);
			break;
		case RPIVID_DELETE_BUFFER:
			size = sizeof(rpivid_delete_buffer);
			break;
		case RPIVID_DECODE:
		case RPIVID_SUBMIT:
			size = sizeof(rpivid_decode);
			break;
		case RPIVID_WAIT:
			size = sizeof(rpivid_wait);
			break;
		default:
			return B_DEV_INVALID_IOCTL;
	}
	if (length != size)
		return B_BAD_VALUE;

	union {
		rpivid_info				info;
		rpivid_create_buffer	create_buffer;
		rpivid_create_picture	create_picture;
		rpivid_delete_buffer	delete_buffer;
		rpivid_decode			decode;
		rpivid_wait				wait;
	} data;
	if (!IS_USER_ADDRESS(arg) || user_memcpy(&data, arg, size) != B_OK)
		return B_BAD_ADDRESS;

	status_t status;
	switch (op) {
		case RPIVID_GET_INFO:
			status = get_info(file->device, &data.info);
			break;
		case RPIVID_CREATE_BUFFER:
			status = create_buffer(file, &data.create_buffer);
			break;
		case RPIVID_CREATE_PICTURE:
			status = create_picture(file, &data.create_picture);
			break;
		case RPIVID_DELETE_BUFFER:
			status = delete_buffer(file, &data.delete_buffer);
			break;
		case RPIVID_DECODE:
			status = rpivid_decode_ioctl(file, &data.decode);
			break;
		case RPIVID_SUBMIT:
			status = rpivid_submit_ioctl(file, &data.decode);
			break;
		case RPIVID_WAIT:
		default:
			status = rpivid_wait_ioctl(file, &data.wait);
			break;
	}

	// a decode reports its status words also when it failed
	if ((status == B_OK || op == RPIVID_DECODE)
		&& user_memcpy(arg, &data, size) != B_OK) {
		return B_BAD_ADDRESS;
	}
	return status;
}


//	#pragma mark - driver module API


static float
rpivid_support(device_node* parent)
{
	const char* bus;
	if (sDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
			!= B_OK || strcmp(bus, "fdt") != 0) {
		return 0.0f;
	}

	const char* compatible;
	if (sDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK) {
		return 0.0f;
	}
	return strcmp(compatible, kCompatible) == 0 ? 0.6f : 0.0f;
}


static status_t
rpivid_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom BCM2711 HEVC decoder" } },
		// the "broken" state must outlive the last close
		{ B_DEVICE_FLAGS, B_UINT32_TYPE, { .ui32 = B_KEEP_DRIVER_LOADED } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, RPIVID_MODULE_NAME, attrs, NULL,
		NULL);
}


/*!	The firmware clock the node names: <&firmware_clocks id>. */
static uint32
find_clock_id(fdt_device_module_info* fdt, fdt_device* fdtDevice)
{
	int length;
	const uint32* clocks = (const uint32*)fdt->get_prop(fdtDevice, "clocks",
		&length);
	if (clocks == NULL || length != 2 * sizeof(uint32))
		return BCM2835_CLOCK_HEVC;
	return B_BENDIAN_TO_HOST_INT32(clocks[1]);
}


static status_t
rpivid_init_driver(device_node* node, void** _driverCookie)
{
	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&fdtDevice);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	rpivid_device* device = (rpivid_device*)calloc(1, sizeof(rpivid_device));
	if (device == NULL)
		return B_NO_MEMORY;

	uint64 block, intc, size;
	if (!fdt->get_reg(fdtDevice, 0, &block, &size) || size < kBlockSize
		|| !fdt->get_reg(fdtDevice, 1, &intc, &size)
		|| !fdt->get_interrupt(fdtDevice, 0, NULL, &device->interrupt)) {
		free(device);
		return B_DEVICE_NOT_FOUND;
	}

	// The block addresses memory through the bus's dma-ranges (on the Pi 4
	// an identity map of the first 16 GiB); its address registers hold 32
	// bits of a 64-byte unit, 256 GiB at most.
	int64 busOffset = 0;
	phys_addr_t dmaEnd = (phys_addr_t)1 << 38;
	if (fdt->get_dma_window(fdtDevice, &busOffset, &dmaEnd))
		dmaEnd = min_c(dmaEnd, (phys_addr_t)1 << 38);

	device->node = node;
	device->block_physical = block;
	device->intc_physical = intc;
	device->clock_id = find_clock_id(fdt, fdtDevice);
	device->bus_offset = busOffset;
	device->dma_end = dmaEnd;
	device->block_area = device->intc_area = -1;
	device->commands.area = device->zero_mv.area = -1;
	for (uint32 i = 0; i < kScratchSets; i++)
		device->scratch[i].pu.area = device->scratch[i].coeff.area = -1;
	mutex_init(&device->lock, "rpivid");
	B_INITIALIZE_SPINLOCK(&device->irq_lock);
	device->job_done.Init(device, "rpivid job");
	device->irq_sem = create_sem(0, "rpivid completion");
	device->thread = -1;
	if (device->irq_sem >= 0) {
		device->thread = spawn_kernel_thread(&rpivid_decode_thread,
			"rpivid decode", B_URGENT_DISPLAY_PRIORITY, device);
	}
	if (device->irq_sem < 0 || device->thread < 0) {
		status = device->irq_sem < 0 ? device->irq_sem : device->thread;
		if (device->irq_sem >= 0)
			delete_sem(device->irq_sem);
		mutex_destroy(&device->lock);
		free(device);
		return status;
	}
	resume_thread(device->thread);

	TRACE_ALWAYS("block %#" B_PRIxPHYSADDR ", interrupts %#" B_PRIxPHYSADDR
		", interrupt %" B_PRIu64 ", clock %" B_PRIu32 ", DMA offset %" B_PRId64
		" below %#" B_PRIxPHYSADDR "\n", device->block_physical,
		device->intc_physical, device->interrupt, device->clock_id,
		device->bus_offset, device->dma_end);

	*_driverCookie = device;
	return B_OK;
}


static void
rpivid_uninit_driver(void* driverCookie)
{
	rpivid_device* device = (rpivid_device*)driverCookie;

	// no file is open: the queue is empty
	mutex_lock(&device->lock);
	device->quit = true;
	mutex_unlock(&device->lock);
	release_sem(device->irq_sem);
	status_t result;
	wait_for_thread(device->thread, &result);

	mutex_lock(&device->lock);
	if (!device->broken) {
		rpivid_decode_free_memory(device);
		rpivid_power_off(device);
	}
	mutex_destroy(&device->lock);
	delete_sem(device->irq_sem);
	free(device);
}


static status_t
rpivid_register_child_devices(void* driverCookie)
{
	rpivid_device* device = (rpivid_device*)driverCookie;
	return sDeviceManager->publish_device(device->node, RPIVID_DEVICE_NAME,
		RPIVID_DEVICE_MODULE_NAME);
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{ BCM2835_MAILBOX_MODULE_NAME, (module_info**)&sMailbox },
	{}
};


driver_module_info rpivid_driver_module = {
	{
		RPIVID_MODULE_NAME,
		0,
		NULL
	},

	rpivid_support,
	rpivid_register_device,
	rpivid_init_driver,
	rpivid_uninit_driver,
	rpivid_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info rpivid_device_module = {
	{
		RPIVID_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	rpivid_init_device,
	rpivid_uninit_device,
	NULL,

	rpivid_open,
	rpivid_close,
	rpivid_free,
	rpivid_read,
	rpivid_write,
	NULL,
	rpivid_control,

	NULL,
	NULL
};


module_info* modules[] = {
	(module_info*)&rpivid_driver_module,
	(module_info*)&rpivid_device_module,
	NULL
};
