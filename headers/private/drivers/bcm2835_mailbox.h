/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _BCM2835_MAILBOX_H
#define _BCM2835_MAILBOX_H


#include <module.h>


/*!	The property interface of the Raspberry Pi's VideoCore firmware, reached
	through the ARM-to-VideoCore mailbox ("brcm,bcm2835-mbox").
*/

#define BCM2835_MAILBOX_MODULE_NAME "generic/bcm2835_mailbox/v1"


typedef struct bcm2835_mailbox_module_info {
	module_info	info;

	/*!	Sends one tag. \a values holds the tag's request values (\a requestSize
		bytes) and receives its response values; \a bufferSize is its size and
		must fit both. Returns B_OK when the firmware answered the tag, with
		the size of the response values in \a _responseSize (if not NULL).
		May sleep.
	*/
	status_t	(*property_tag)(uint32 tag, void* values, uint32 bufferSize,
					uint32 requestSize, uint32* _responseSize);
} bcm2835_mailbox_module_info;


// Clock IDs of the clock tags
#define BCM2835_CLOCK_ARM					3
#define BCM2835_CLOCK_CORE					4
#define BCM2835_CLOCK_V3D					5

// Tags (https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface)
#define BCM2835_TAG_GET_FIRMWARE_REVISION	0x00000001
#define BCM2835_TAG_GET_BOARD_MODEL			0x00010001
#define BCM2835_TAG_GET_BOARD_REVISION		0x00010002
#define BCM2835_TAG_GET_BOARD_SERIAL		0x00010004
#define BCM2835_TAG_GET_ARM_MEMORY			0x00010005
#define BCM2835_TAG_GET_VC_MEMORY			0x00010006
#define BCM2835_TAG_GET_POWER_STATE			0x00020001
#define BCM2835_TAG_SET_POWER_STATE			0x00028001
#define BCM2835_TAG_GET_CLOCK_STATE			0x00030001
#define BCM2835_TAG_GET_CLOCK_RATE			0x00030002
#define BCM2835_TAG_SET_QPU_ENABLE			0x00030012
#define BCM2835_TAG_GET_CLOCK_MEASURED		0x00030047
#define BCM2835_TAG_SET_CLOCK_STATE			0x00038001
#define BCM2835_TAG_SET_DOMAIN_STATE		0x00038030
#define BCM2835_TAG_GET_TEMPERATURE			0x00030006
#define BCM2835_TAG_GET_MAX_CLOCK_RATE		0x00030004
#define BCM2835_TAG_GET_MIN_CLOCK_RATE		0x00030007
#define BCM2835_TAG_GET_MAX_TEMPERATURE		0x0003000a
#define BCM2835_TAG_GET_THROTTLED			0x00030046
#define BCM2835_TAG_SET_CLOCK_RATE			0x00038002
#define BCM2835_TAG_GET_TURBO				0x00030009
#define BCM2835_TAG_SET_TURBO				0x00038009
#define BCM2835_TAG_GET_GPIO_STATE			0x00030041
#define BCM2835_TAG_SET_GPIO_STATE			0x00038041
#define BCM2835_TAG_NOTIFY_XHCI_RESET		0x00030058
#define BCM2835_TAG_GET_EDID_BLOCK			0x00030020
#define BCM2835_TAG_FRAMEBUFFER_BLANK		0x00040002
#define BCM2835_TAG_GET_DISPLAY_ID			0x00040016
#define BCM2835_TAG_SET_DISPLAY_NUM			0x00048013
#define BCM2835_TAG_SET_PLANE				0x00048015
#define BCM2835_TAG_SET_VSYNC				0x0004800e
	// blocks until the next vertical sync

// Clock IDs for the clock tags
#define BCM2835_CLOCK_EMMC		1
#define BCM2835_CLOCK_UART		2
#define BCM2835_CLOCK_ARM		3
#define BCM2835_CLOCK_CORE		4
#define BCM2835_CLOCK_V3D		5
#define BCM2835_CLOCK_HEVC		11
#define BCM2835_CLOCK_EMMC2		12

// Power domain IDs for SET_DOMAIN_STATE
#define BCM2835_DOMAIN_V3D		10

// The GPIO tags number the firmware's GPIO expander lines from here
#define BCM2835_FIRMWARE_GPIO_BASE	128


#endif	// _BCM2835_MAILBOX_H
