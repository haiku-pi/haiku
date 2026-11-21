/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	A report of the physical memory below 1 GiB, where pictures the display
	scans out must lie: how many pages are in which state, the longest runs a
	contiguous allocation could take, a map of it in 1 MiB cells, and the
	kernel's contiguous areas there. Read from the device after the first line
	(diagnostic: why a scan-out allocation fails).
*/


#include "rpivid_private.h"

#include <stdarg.h>
#include <stdio.h>

#include <vm/VMAddressSpace.h>
#include <vm/VMArea.h>
#include <vm/vm_page.h>
#include <vm/vm_priv.h>
#include <vm/vm_types.h>


static const phys_addr_t kLimit = (phys_addr_t)1 << 30;
static const uint32 kCellPages = (1024 * 1024) / B_PAGE_SIZE;


struct report_buffer {
	char*	data;
	size_t	size;
	size_t	length;

	void Print(const char* format, ...) __attribute__((format(printf, 2, 3)))
	{
		if (length >= size)
			return;
		va_list args;
		va_start(args, format);
		int printed = vsnprintf(data + length, size - length, format, args);
		va_end(args);
		if (printed > 0)
			length = min_c(length + printed, size - 1);
	}
};


/*!	One character per MiB: '-' no RAM, '.' all free, 'c' free or cached (a
	run may take it), otherwise what blocks a run there: 'w' wired, 'a'
	mapped/user pages (active, inactive, modified); upper case if no page of
	the MiB is free or cached.
*/
static char
cell_char(const uint32* counts, uint32 missing)
{
	if (missing == kCellPages)
		return '-';
	const uint32 takeable = counts[PAGE_STATE_FREE] + counts[PAGE_STATE_CLEAR]
		+ counts[PAGE_STATE_CACHED];
	if (takeable + missing == kCellPages) {
		return counts[PAGE_STATE_CACHED] == 0 ? '.' : 'c';
	}
	const uint32 wired = counts[PAGE_STATE_WIRED] + counts[PAGE_STATE_UNUSED];
	const uint32 user = counts[PAGE_STATE_ACTIVE] + counts[PAGE_STATE_INACTIVE]
		+ counts[PAGE_STATE_MODIFIED];
	char c = wired >= user ? 'w' : 'a';
	if (takeable == 0)
		c -= 'a' - 'A';
	return c;
}


size_t
rpivid_low_memory_report(char* data, size_t size)
{
	report_buffer report = { data, size, 0 };

	uint32 states[PAGE_STATE_COUNT] = {};
	uint32 missingPages = 0;
	// the longest runs of free pages, and of free or cached ones
	page_num_t freeRun = 0, freeBest = 0, freeBestStart = 0;
	page_num_t takeRun = 0, takeBest = 0, takeBestStart = 0;
	// runs of 12.7 MiB (a 4K scan-out picture) and 3.2 MiB (1080p) that
	// could be taken one after the other
	const page_num_t k4K = (12700 * 1024) / B_PAGE_SIZE;
	const page_num_t k1080 = (3200 * 1024) / B_PAGE_SIZE;
	uint32 runs4K = 0, runs1080 = 0;

	char map[1024 + 1];
	const page_num_t pages = kLimit / B_PAGE_SIZE;
	for (page_num_t cell = 0; cell < pages / kCellPages; cell++) {
		uint32 counts[PAGE_STATE_COUNT] = {};
		uint32 missing = 0;
		for (uint32 i = 0; i < kCellPages; i++) {
			const page_num_t number = cell * kCellPages + i;
			vm_page* page = vm_lookup_page(number);
			uint8 state = page != NULL ? page->State() : PAGE_STATE_COUNT;
			if (state < PAGE_STATE_COUNT)
				counts[state]++;
			else
				missing++;

			const bool isFree = state == PAGE_STATE_FREE
				|| state == PAGE_STATE_CLEAR;
			const bool takeable = isFree || state == PAGE_STATE_CACHED;
			if (isFree) {
				if (++freeRun > freeBest) {
					freeBest = freeRun;
					freeBestStart = number + 1 - freeRun;
				}
			} else
				freeRun = 0;
			if (takeable) {
				if (++takeRun > takeBest) {
					takeBest = takeRun;
					takeBestStart = number + 1 - takeRun;
				}
				if (takeRun % k4K == 0)
					runs4K++;
				if (takeRun % k1080 == 0)
					runs1080++;
			} else
				takeRun = 0;
		}
		for (uint32 s = 0; s < PAGE_STATE_COUNT; s++)
			states[s] += counts[s];
		missingPages += missing;
		map[cell] = cell_char(counts, missing);
	}
	map[pages / kCellPages] = '\0';

	report.Print("below 1 GiB (MiB): free %" B_PRIu32 " clear %" B_PRIu32
		" cached %" B_PRIu32 " active %" B_PRIu32 " inactive %" B_PRIu32
		" modified %" B_PRIu32 " wired %" B_PRIu32 " unused %" B_PRIu32
		" no RAM %" B_PRIu32 "\n",
		states[PAGE_STATE_FREE] / kCellPages,
		states[PAGE_STATE_CLEAR] / kCellPages,
		states[PAGE_STATE_CACHED] / kCellPages,
		states[PAGE_STATE_ACTIVE] / kCellPages,
		states[PAGE_STATE_INACTIVE] / kCellPages,
		states[PAGE_STATE_MODIFIED] / kCellPages,
		states[PAGE_STATE_WIRED] / kCellPages,
		states[PAGE_STATE_UNUSED] / kCellPages, missingPages / kCellPages);
	report.Print("longest run: free %" B_PRIu64 " KiB at %#" B_PRIx64
		", free or cached %" B_PRIu64 " KiB at %#" B_PRIx64 "; runs of 12.7 MiB"
		" %" B_PRIu32 ", of 3.2 MiB %" B_PRIu32 "\n",
		(uint64)freeBest * B_PAGE_SIZE / 1024,
		(uint64)freeBestStart * B_PAGE_SIZE,
		(uint64)takeBest * B_PAGE_SIZE / 1024,
		(uint64)takeBestStart * B_PAGE_SIZE, runs4K, runs1080);
	for (uint32 row = 0; row < 1024 / 64; row++)
		report.Print("map %04" B_PRIu32 " %.64s\n", row * 64, map + row * 64);

	// the kernel's contiguous areas that start below 1 GiB, by name
	struct owner {
		char	name[B_OS_NAME_LENGTH];
		uint32	count;
		size_t	size;
	};
	static const uint32 kMaxOwners = 40;
	owner* owners = (owner*)calloc(kMaxOwners, sizeof(owner));
	uint32 ownerCount = 0, other = 0;
	VMAddressSpace* kernel = VMAddressSpace::Kernel();
	kernel->ReadLock();
	VMTranslationMap* translationMap = kernel->TranslationMap();
	for (VMAddressSpace::AreaIterator it = kernel->GetAreaIterator();
			VMArea* area = it.Next();) {
		if (area->wiring != B_CONTIGUOUS)
			continue;
		phys_addr_t physical;
		uint32 flags;
		translationMap->Lock();
		status_t status = translationMap->Query(area->Base(), &physical,
			&flags);
		translationMap->Unlock();
		if (status != B_OK || (flags & PAGE_PRESENT) == 0
			|| physical >= kLimit) {
			continue;
		}
		uint32 i = 0;
		while (owners != NULL && i < ownerCount
			&& strcmp(owners[i].name, area->name) != 0) {
			i++;
		}
		if (owners == NULL || i == kMaxOwners) {
			other++;
			continue;
		}
		if (i == ownerCount) {
			strlcpy(owners[i].name, area->name, B_OS_NAME_LENGTH);
			ownerCount++;
		}
		owners[i].count++;
		owners[i].size += area->Size();
	}
	kernel->ReadUnlock();
	for (uint32 i = 0; i < ownerCount; i++) {
		report.Print("contiguous: %" B_PRIuSIZE " KiB in %" B_PRIu32 " %s\n",
			owners[i].size / 1024, owners[i].count, owners[i].name);
	}
	if (other > 0)
		report.Print("contiguous: %" B_PRIu32 " more\n", other);
	free(owners);

	return report.length;
}
