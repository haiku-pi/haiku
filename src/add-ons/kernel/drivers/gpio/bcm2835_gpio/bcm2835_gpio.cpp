/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The GPIO controller of the Raspberry Pi 4 ("brcm,bcm2711-gpio"), published
	as gpio/bcm2835/<n>. Reading it gives one line per pin: function, level
	and pull. Writing text commands changes them, one per line:
		in <pin>			make the pin an input
		out <pin> <0|1>		make the pin an output, at that level
		alt <pin> <0-5>		select an alternate function
		pull <pin> <none|up|down>
*/


#include <Drivers.h>
#include <Errors.h>
#include <KernelExport.h>
#include <bus/FDT.h>
#include <kernel.h>
#include <lock.h>
#include <util/AutoLock.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define BCM2835_GPIO_MODULE_NAME "drivers/gpio/bcm2835_gpio/driver_v1"
#define BCM2835_GPIO_DEVICE_MODULE_NAME "drivers/gpio/bcm2835_gpio/device_v1"
#define BCM2835_GPIO_BASENAME "gpio/bcm2835/%d"
#define BCM2835_GPIO_PATHID_GENERATOR "bcm2835_gpio/path_id"

#define ERROR(x...)	dprintf("bcm2835_gpio: " x)


// Registers (BCM2711 ARM Peripherals, chapter 5)
enum {
	kFunctionSelect0	= 0x00,		// 3 bits per pin, 10 pins per register
	kOutputSet0			= 0x1c,
	kOutputClear0		= 0x28,
	kLevel0				= 0x34,
	kPullControl0		= 0xe4,		// 2 bits per pin, 16 pins per register
};

static const uint32 kPinCount = 58;
static const char* const kFunctionNames[8] = {
	"in", "out", "alt5", "alt4", "alt0", "alt1", "alt2", "alt3"
};
static const char* const kPullNames[4] = { "none", "up", "down", "?" };


struct gpio_device {
	device_node*	node;
	area_id			area;
	volatile uint32* registers;
	mutex			lock;
};


static device_manager_info* sDeviceManager;


static uint32
read_register(gpio_device* device, uint32 offset)
{
	return device->registers[offset / 4];
}


static void
write_register(gpio_device* device, uint32 offset, uint32 value)
{
	device->registers[offset / 4] = value;
}


static void
set_function(gpio_device* device, uint32 pin, uint32 function)
{
	const uint32 offset = kFunctionSelect0 + (pin / 10) * 4;
	const uint32 shift = (pin % 10) * 3;
	write_register(device, offset, (read_register(device, offset)
		& ~(7u << shift)) | (function << shift));
}


static void
set_level(gpio_device* device, uint32 pin, bool high)
{
	write_register(device, (high ? kOutputSet0 : kOutputClear0) + (pin / 32) * 4,
		1u << (pin % 32));
}


static void
set_pull(gpio_device* device, uint32 pin, uint32 pull)
{
	const uint32 offset = kPullControl0 + (pin / 16) * 4;
	const uint32 shift = (pin % 16) * 2;
	write_register(device, offset, (read_register(device, offset)
		& ~(3u << shift)) | (pull << shift));
}


/*!	Executes one command line; returns B_BAD_VALUE for anything malformed. */
static status_t
execute_command(gpio_device* device, const char* line)
{
	// "<command> <pin> [argument]", separated by spaces
	char buffer[64];
	strlcpy(buffer, line, sizeof(buffer));
	const char* words[3] = { NULL, NULL, NULL };
	int count = 0;
	for (char* word = buffer; *word != '\0' && count < 3; ) {
		while (*word == ' ' || *word == '\t')
			word++;
		if (*word == '\0')
			break;
		words[count++] = word;
		while (*word != '\0' && *word != ' ' && *word != '\t')
			word++;
		if (*word != '\0')
			*word++ = '\0';
	}
	if (count < 2)
		return B_BAD_VALUE;

	char* end;
	const uint32 pin = strtoul(words[1], &end, 10);
	if (*end != '\0' || pin >= kPinCount)
		return B_BAD_VALUE;
	const char* command = words[0];
	const char* argument = count == 3 ? words[2] : "";

	MutexLocker locker(device->lock);

	if (strcmp(command, "in") == 0 && count == 2) {
		set_function(device, pin, 0);
	} else if (strcmp(command, "out") == 0 && count == 3
		&& (strcmp(argument, "0") == 0 || strcmp(argument, "1") == 0)) {
		// set the level first, so the pin does not glitch
		set_level(device, pin, argument[0] == '1');
		set_function(device, pin, 1);
	} else if (strcmp(command, "alt") == 0 && count == 3
		&& argument[0] >= '0' && argument[0] <= '5' && argument[1] == '\0') {
		static const uint32 kAlternate[6] = { 4, 5, 6, 7, 3, 2 };
		set_function(device, pin, kAlternate[argument[0] - '0']);
	} else if (strcmp(command, "pull") == 0 && count == 3) {
		uint32 pull;
		if (strcmp(argument, "none") == 0)
			pull = 0;
		else if (strcmp(argument, "up") == 0)
			pull = 1;
		else if (strcmp(argument, "down") == 0)
			pull = 2;
		else
			return B_BAD_VALUE;
		set_pull(device, pin, pull);
	} else
		return B_BAD_VALUE;

	return B_OK;
}


//	#pragma mark - device module API


static status_t
bcm2835_gpio_init_device(void* driverCookie, void** _cookie)
{
	*_cookie = driverCookie;
	return B_OK;
}


static void
bcm2835_gpio_uninit_device(void* cookie)
{
}


static status_t
bcm2835_gpio_open(void* deviceCookie, const char* path, int openMode,
	void** _cookie)
{
	*_cookie = deviceCookie;
	return B_OK;
}


static status_t
bcm2835_gpio_close(void* cookie)
{
	return B_OK;
}


static status_t
bcm2835_gpio_free(void* cookie)
{
	return B_OK;
}


static status_t
bcm2835_gpio_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	gpio_device* device = (gpio_device*)cookie;

	// the whole table; reads continue in it by position
	char text[kPinCount * 32];
	size_t length = 0;
	{
		MutexLocker locker(device->lock);
		for (uint32 pin = 0; pin < kPinCount; pin++) {
			const uint32 function = (read_register(device,
				kFunctionSelect0 + (pin / 10) * 4) >> ((pin % 10) * 3)) & 7;
			const uint32 level = (read_register(device,
				kLevel0 + (pin / 32) * 4) >> (pin % 32)) & 1;
			const uint32 pull = (read_register(device,
				kPullControl0 + (pin / 16) * 4) >> ((pin % 16) * 2)) & 3;
			length += snprintf(text + length, sizeof(text) - length,
				"%2" B_PRIu32 " %-4s %" B_PRIu32 " %s\n", pin,
				kFunctionNames[function], level, kPullNames[pull]);
		}
	}

	if (position >= (off_t)length) {
		*_length = 0;
		return B_OK;
	}
	size_t toCopy = min_c(*_length, length - (size_t)position);
	if (user_memcpy(buffer, text + position, toCopy) != B_OK)
		return B_BAD_ADDRESS;
	*_length = toCopy;
	return B_OK;
}


static status_t
bcm2835_gpio_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	gpio_device* device = (gpio_device*)cookie;

	char text[256];
	size_t length = min_c(*_length, sizeof(text) - 1);
	if (user_memcpy(text, buffer, length) != B_OK)
		return B_BAD_ADDRESS;
	text[length] = '\0';

	char* line = text;
	while (line != NULL && *line != '\0') {
		char* next = strchr(line, '\n');
		if (next != NULL)
			*next++ = '\0';
		if (*line != '\0') {
			status_t status = execute_command(device, line);
			if (status != B_OK)
				return status;
		}
		line = next;
	}

	*_length = length;
	return B_OK;
}


static status_t
bcm2835_gpio_control(void* cookie, uint32 op, void* arg, size_t length)
{
	return B_DEV_INVALID_IOCTL;
}


//	#pragma mark - driver module API


static float
bcm2835_gpio_support(device_node* parent)
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

	return strcmp(compatible, "brcm,bcm2711-gpio") == 0 ? 0.6f : 0.0f;
}


static status_t
bcm2835_gpio_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Raspberry Pi GPIO" } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, BCM2835_GPIO_MODULE_NAME, attrs,
		NULL, NULL);
}


static status_t
bcm2835_gpio_init_driver(device_node* node, void** _driverCookie)
{
	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&fdtDevice);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	uint64 registers, size;
	if (!fdt->get_reg(fdtDevice, 0, &registers, &size))
		return B_DEVICE_NOT_FOUND;

	gpio_device* device = (gpio_device*)calloc(1, sizeof(gpio_device));
	if (device == NULL)
		return B_NO_MEMORY;

	// The pull registers (0xe4..0xf0) lie beyond the "reg" size the device
	// tree gives; they are within the same page.
	void* address;
	device->area = map_physical_memory("bcm2835 gpio", registers, B_PAGE_SIZE,
		B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		&address);
	if (device->area < 0) {
		status = device->area;
		free(device);
		return status;
	}

	device->node = node;
	device->registers = (volatile uint32*)address;
	mutex_init(&device->lock, "bcm2835 gpio");
	*_driverCookie = device;
	return B_OK;
}


static void
bcm2835_gpio_uninit_driver(void* driverCookie)
{
	gpio_device* device = (gpio_device*)driverCookie;
	mutex_destroy(&device->lock);
	delete_area(device->area);
	free(device);
}


static status_t
bcm2835_gpio_register_child_devices(void* driverCookie)
{
	gpio_device* device = (gpio_device*)driverCookie;

	int pathID = sDeviceManager->create_id(BCM2835_GPIO_PATHID_GENERATOR);
	if (pathID < 0) {
		ERROR("couldn't create a path ID\n");
		return B_ERROR;
	}

	char name[B_DEV_NAME_LENGTH];
	snprintf(name, sizeof(name), BCM2835_GPIO_BASENAME, pathID);
	return sDeviceManager->publish_device(device->node, name,
		BCM2835_GPIO_DEVICE_MODULE_NAME);
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{}
};


driver_module_info bcm2835_gpio_driver_module = {
	{
		BCM2835_GPIO_MODULE_NAME,
		0,
		NULL
	},

	bcm2835_gpio_support,
	bcm2835_gpio_register_device,
	bcm2835_gpio_init_driver,
	bcm2835_gpio_uninit_driver,
	bcm2835_gpio_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info bcm2835_gpio_device_module = {
	{
		BCM2835_GPIO_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	bcm2835_gpio_init_device,
	bcm2835_gpio_uninit_device,
	NULL,

	bcm2835_gpio_open,
	bcm2835_gpio_close,
	bcm2835_gpio_free,
	bcm2835_gpio_read,
	bcm2835_gpio_write,
	NULL,
	bcm2835_gpio_control,

	NULL,
	NULL
};


module_info* modules[] = {
	(module_info*)&bcm2835_gpio_driver_module,
	(module_info*)&bcm2835_gpio_device_module,
	NULL
};
