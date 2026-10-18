/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The "sdio" bus of the compatibility layer: SDIO cards that Haiku's MMC
	bus manager found, offered to the drivers of a
	HAIKU_FBSD_WLAN_SDIO_DRIVER_GLUE module. A driver gets the card as its
	device's ivars, a haiku_sdio_device (dev/sdio/haiku_sdio.h).
*/


extern "C" {
#include "device.h"
}

#include <stdlib.h>

#include <device_manager.h>
#include <mmc.h>

#include <dev/sdio/haiku_sdio.h>


struct haiku_sdio_device {
	mmc_device_interface*	mmc;
	device_node*			node;
	void*					cookie;
		// of the MMC bus the card is on
	uint16					rca;
	uint16					vendor;
	uint16					product;
	uint8					function_count;
};


static device_manager_info* sDeviceManager;


static void
free_sdio_device(void* cookie)
{
	free(cookie);
	put_module(MMC_BUS_MODULE_NAME);
}


static void
prepare_sdio_attach(void* cookie, device_t device)
{
	device_set_ivars(device, cookie);
}


static haiku_sdio_device*
create_sdio_device(device_node* node)
{
	haiku_sdio_device* sdio = (haiku_sdio_device*)calloc(1, sizeof(*sdio));
	if (sdio == NULL)
		return NULL;

	if (sDeviceManager->get_attr_uint16(node, kMmcRcaAttribute, &sdio->rca,
			false) != B_OK
		|| sDeviceManager->get_attr_uint16(node, B_DEVICE_VENDOR_ID,
			&sdio->vendor, false) != B_OK
		|| sDeviceManager->get_attr_uint16(node, B_DEVICE_ID, &sdio->product,
			false) != B_OK
		|| sDeviceManager->get_attr_uint8(node, kSdioFunctionsAttribute,
			&sdio->function_count, false) != B_OK
		|| get_module(MMC_BUS_MODULE_NAME, (module_info**)&sdio->mmc) != B_OK) {
		free(sdio);
		return NULL;
	}

	// The commands go through the MMC bus, the card node's parent
	device_node* bus = sDeviceManager->get_parent_node(node);
	driver_module_info* busModule;
	status_t status = sDeviceManager->get_driver(bus, &busModule,
		&sdio->cookie);
	sDeviceManager->put_node(bus);
	if (status != B_OK) {
		free_sdio_device(sdio);
		return NULL;
	}

	sdio->node = node;
	return sdio;
}


/*!	Offers the SDIO cards in the device tree below \a parent to \a drivers. */
static bool
probe_sdio_cards(device_node* parent, device_t root, driver_t* drivers[])
{
	const device_attr any[] = { { NULL } };
	bool found = false;

	device_node* node = NULL;
	while (sDeviceManager->get_next_child_node(parent, any, &node) == B_OK) {
		uint8 type;
		if (sDeviceManager->get_attr_uint8(node, kMmcTypeAttribute, &type,
				false) != B_OK || type != CARD_TYPE_SDIO) {
			found |= probe_sdio_cards(node, root, drivers);
			continue;
		}

		haiku_sdio_device* sdio = create_sdio_device(node);
		if (sdio == NULL)
			continue;

		struct device device = {};
		device.parent = root;
		device.root = root;
		device_set_ivars(&device, sdio);

		driver_t* driver = __haiku_probe_drivers(&device, drivers);
		if (driver == NULL) {
			free_sdio_device(sdio);
			continue;
		}

		report_probed_device(root_device_softc::BUS_sdio, sdio, driver,
			prepare_sdio_attach, free_sdio_device);
		found = true;
	}
	return found;
}


status_t
_fbsd_init_hardware_sdio(driver_t* drivers[])
{
	status_t status = get_module(B_DEVICE_MANAGER_MODULE_NAME,
		(module_info**)&sDeviceManager);
	if (status != B_OK)
		return status;

	device_t root;
	status = init_root_device(&root, root_device_softc::BUS_sdio);
	if (status != B_OK) {
		put_module(B_DEVICE_MANAGER_MODULE_NAME);
		return status;
	}

	device_node* deviceRoot = sDeviceManager->get_root_node();
	bool found = probe_sdio_cards(deviceRoot, root, drivers);
	sDeviceManager->put_node(deviceRoot);

	device_delete_child(NULL, root);
	put_module(B_DEVICE_MANAGER_MODULE_NAME);

	return found ? B_OK : B_NOT_SUPPORTED;
}


//	#pragma mark - card access


extern "C" uint16_t
haiku_sdio_vendor(struct haiku_sdio_device* device)
{
	return device->vendor;
}


extern "C" uint16_t
haiku_sdio_product(struct haiku_sdio_device* device)
{
	return device->product;
}


extern "C" int
haiku_sdio_function_count(struct haiku_sdio_device* device)
{
	return device->function_count;
}


extern "C" int
haiku_sdio_read_direct(struct haiku_sdio_device* device, int function,
	uint32_t address, uint8_t* _value)
{
	return device->mmc->sdio_read_direct(device->node, device->cookie,
		device->rca, function, address, _value);
}


extern "C" int
haiku_sdio_write_direct(struct haiku_sdio_device* device, int function,
	uint32_t address, uint8_t value)
{
	return device->mmc->sdio_write_direct(device->node, device->cookie,
		device->rca, function, address, value);
}


extern "C" int
haiku_sdio_read_write_extended(struct haiku_sdio_device* device, int function,
	uint32_t address, int incrementAddress, int isWrite, void* buffer,
	size_t length)
{
	return device->mmc->sdio_read_write_extended(device->node, device->cookie,
		device->rca, function, address, incrementAddress != 0, isWrite != 0,
		buffer, length);
}


extern "C" int
haiku_sdio_set_block_size(struct haiku_sdio_device* device, int function,
	uint16_t blockSize)
{
	return device->mmc->sdio_set_block_size(device->node, device->cookie,
		device->rca, function, blockSize);
}


extern "C" int
haiku_sdio_enable_function(struct haiku_sdio_device* device, int function,
	int enable)
{
	return device->mmc->sdio_enable_function(device->node, device->cookie,
		device->rca, function, enable != 0);
}
