/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	CPU jobs (SUBMIT_CPU, the DRM_V3D_EXT_ID_CPU_* extensions): small pieces
	of work the driver does on the CPU when they reach the head of the job
	queue, after every job submitted before them has completed - what
	Vulkan's timestamp queries and indirect dispatches need (as Linux
	v3d_sched.c):

	- timestamp query: the time goes into the query BO; the queries'
	  syncobjs signal with the job.
	- reset timestamp queries: their values are zeroed, their syncobjs lose
	  their fence.
	- copy timestamp query results into a buffer (available = signalled).
	- indirect dispatch: reads the workgroup counts from a buffer, sets up
	  the compute job queued right behind it and patches the counts into its
	  uniforms.

	The performance query jobs need performance monitors, which the driver
	does not have (GET_PARAM SUPPORTS_PERFMON is 0).
*/


#include "v3d_private.h"


static const uint32 kNoUniformRewrite = 0xffffffff;
static const uint32 kWorkgroupCountShift = 16;


struct v3d_cpu_job {
	uint32				type;
		// DRM_V3D_EXT_ID_CPU_*
	uint32				count;
	uint32*				offsets;
		// of each query's value in the timestamp BO
	uint32*				sync_handles;
	v3d_syncobj**		syncs;
		// each query's syncobj, referenced once the job is queued

	// copy timestamp query results
	bool				do_64bit;
	bool				do_partial;
	bool				availability_bit;
	uint32				offset;
	uint32				stride;
		// also: indirect dispatch, the offset of the workgroup counts

	// indirect dispatch
	v3d_job*			csd_job;
		// queued right behind this job
	uint32				indirect_handle;
	v3d_bo*				indirect;
		// the BO with the uniforms the counts are patched into
	uint32				wg_size;
	uint32				wg_uniform_offsets[3];
};


/*!	Whether [offset, offset + size) lies in \a bo. */
static inline bool
in_bo(const v3d_bo* bo, uint64 offset, uint64 size)
{
	return offset + size <= bo->size;
}


static status_t
read_extension(const v3d_submit_ext& ext, v3d_cpu_job* cpu)
{
	const void* user = (const void*)(addr_t)ext.cpu_extension;

	switch (ext.cpu_type) {
		case DRM_V3D_EXT_ID_CPU_TIMESTAMP_QUERY:
		{
			drm_v3d_timestamp_query query;
			if (user_memcpy(&query, user, sizeof(query)) != B_OK)
				return B_BAD_ADDRESS;
			if (query.pad != 0)
				return B_BAD_VALUE;
			cpu->count = query.count;
			status_t status = v3d_copy_user_array(query.offsets, query.count,
				&cpu->offsets);
			if (status != B_OK)
				return status;
			return v3d_copy_user_array(query.syncs, query.count,
				&cpu->sync_handles);
		}

		case DRM_V3D_EXT_ID_CPU_RESET_TIMESTAMP_QUERY:
		{
			drm_v3d_reset_timestamp_query reset;
			if (user_memcpy(&reset, user, sizeof(reset)) != B_OK)
				return B_BAD_ADDRESS;
			cpu->count = reset.count;
			status_t status = v3d_copy_user_array(reset.syncs, reset.count,
				&cpu->sync_handles);
			if (status != B_OK || reset.count == 0)
				return status;
			// consecutive 64-bit values
			cpu->offsets = (uint32*)malloc(reset.count * sizeof(uint32));
			if (cpu->offsets == NULL)
				return B_NO_MEMORY;
			for (uint32 i = 0; i < reset.count; i++)
				cpu->offsets[i] = reset.offset + 8 * i;
			return B_OK;
		}

		case DRM_V3D_EXT_ID_CPU_COPY_TIMESTAMP_QUERY:
		{
			drm_v3d_copy_timestamp_query copy;
			if (user_memcpy(&copy, user, sizeof(copy)) != B_OK)
				return B_BAD_ADDRESS;
			if (copy.pad != 0)
				return B_BAD_VALUE;
			cpu->count = copy.count;
			cpu->do_64bit = copy.do_64bit != 0;
			cpu->do_partial = copy.do_partial != 0;
			cpu->availability_bit = copy.availability_bit != 0;
			cpu->offset = copy.offset;
			cpu->stride = copy.stride;
			status_t status = v3d_copy_user_array(copy.offsets, copy.count,
				&cpu->offsets);
			if (status != B_OK)
				return status;
			return v3d_copy_user_array(copy.syncs, copy.count,
				&cpu->sync_handles);
		}

		case DRM_V3D_EXT_ID_CPU_INDIRECT_CSD:
		{
			drm_v3d_indirect_csd indirect;
			if (user_memcpy(&indirect, user, sizeof(indirect)) != B_OK)
				return B_BAD_ADDRESS;
			cpu->offset = indirect.offset;
			cpu->wg_size = indirect.wg_size;
			memcpy(cpu->wg_uniform_offsets, indirect.wg_uniform_offsets,
				sizeof(cpu->wg_uniform_offsets));
			cpu->indirect_handle = indirect.indirect;
			// (its extensions are not used: the CPU job's are the ones)
			return v3d_csd_job_create(indirect.submit, &cpu->csd_job);
		}
	}

	// the performance queries
	return B_NOT_SUPPORTED;
}


/*!	A CPU job of \a args, with the compute job of an indirect dispatch
	chained to it (next), not queued yet.
*/
status_t
v3d_cpu_job_create(const drm_v3d_submit_cpu& args, const v3d_submit_ext& ext,
	v3d_job** _first)
{
	// the BOs each job type takes (Linux cpu_job_bo_handle_count)
	uint32 boCount;
	switch (ext.cpu_type) {
		case DRM_V3D_EXT_ID_CPU_INDIRECT_CSD:
		case DRM_V3D_EXT_ID_CPU_TIMESTAMP_QUERY:
		case DRM_V3D_EXT_ID_CPU_RESET_TIMESTAMP_QUERY:
			boCount = 1;
			break;
		case DRM_V3D_EXT_ID_CPU_COPY_TIMESTAMP_QUERY:
			boCount = 2;
			break;
		default:
			return B_NOT_SUPPORTED;
	}
	if (args.bo_handle_count != boCount)
		return B_BAD_VALUE;

	v3d_job* job = (v3d_job*)calloc(1, sizeof(v3d_job));
	v3d_cpu_job* cpu = (v3d_cpu_job*)calloc(1, sizeof(v3d_cpu_job));
	if (job == NULL || cpu == NULL) {
		free(job);
		free(cpu);
		return B_NO_MEMORY;
	}
	job->type = V3D_JOB_TYPE_CPU;
	job->cpu = cpu;
	cpu->type = ext.cpu_type;
	job->handle_count = args.bo_handle_count;

	status_t status = v3d_copy_user_array(args.bo_handles,
		args.bo_handle_count, &job->handles);
	if (status == B_OK)
		status = read_extension(ext, cpu);
	if (status != B_OK) {
		// nothing is referenced yet
		if (cpu->csd_job != NULL)
			v3d_job_free(NULL, cpu->csd_job);
		v3d_job_free(NULL, job);
		return status;
	}

	job->next = cpu->csd_job;
	*_first = job;
	return B_OK;
}


/*!	Takes the syncobjs and BOs the job uses besides its BO list, and checks
	that every access lies inside its BO. Called with the device lock held,
	the BO list resolved.
*/
status_t
v3d_cpu_job_resolve(v3d_file* file, v3d_job* job)
{
	v3d_cpu_job* cpu = job->cpu;

	if (cpu->count > 0) {
		cpu->syncs = (v3d_syncobj**)calloc(cpu->count, sizeof(v3d_syncobj*));
		if (cpu->syncs == NULL)
			return B_NO_MEMORY;
	}
	for (uint32 i = 0; i < cpu->count; i++) {
		v3d_syncobj* syncobj = v3d_syncobj_lookup(file, cpu->sync_handles[i]);
		if (syncobj == NULL)
			return B_ENTRY_NOT_FOUND;
		syncobj->references++;
		cpu->syncs[i] = syncobj;
	}

	switch (cpu->type) {
		case DRM_V3D_EXT_ID_CPU_TIMESTAMP_QUERY:
		case DRM_V3D_EXT_ID_CPU_RESET_TIMESTAMP_QUERY:
			for (uint32 i = 0; i < cpu->count; i++) {
				if (!in_bo(job->bos[0], cpu->offsets[i], sizeof(uint64)))
					return B_BAD_VALUE;
			}
			break;

		case DRM_V3D_EXT_ID_CPU_COPY_TIMESTAMP_QUERY:
		{
			// the results go to bos[0], the values come from bos[1]
			const uint64 element = (cpu->availability_bit ? 2 : 1)
				* (cpu->do_64bit ? sizeof(uint64) : sizeof(uint32));
			for (uint32 i = 0; i < cpu->count; i++) {
				if (!in_bo(job->bos[1], cpu->offsets[i], sizeof(uint64))
					|| !in_bo(job->bos[0],
						cpu->offset + (uint64)cpu->stride * i, element)) {
					return B_BAD_VALUE;
				}
			}
			break;
		}

		case DRM_V3D_EXT_ID_CPU_INDIRECT_CSD:
		{
			cpu->indirect = v3d_bo_lookup(file, cpu->indirect_handle);
			if (cpu->indirect == NULL)
				return B_ENTRY_NOT_FOUND;
			cpu->indirect->references++;

			if (!in_bo(job->bos[0], cpu->offset, 3 * sizeof(uint32)))
				return B_BAD_VALUE;
			for (int32 i = 0; i < 3; i++) {
				const uint32 index = cpu->wg_uniform_offsets[i];
				if (index != kNoUniformRewrite
					&& !in_bo(cpu->indirect, (uint64)index * sizeof(uint32),
						sizeof(uint32))) {
					return B_BAD_VALUE;
				}
			}
			break;
		}
	}
	return B_OK;
}


static void
write_result(uint8* data, uint32 index, bool do64Bit, uint64 value)
{
	if (do64Bit)
		memcpy(data + index * sizeof(uint64), &value, sizeof(uint64));
	else {
		const uint32 value32 = (uint32)value;
		memcpy(data + index * sizeof(uint32), &value32, sizeof(uint32));
	}
}


/*!	As Linux v3d_rewrite_csd_job_wg_counts_from_indirect(). An empty grid
	dispatches nothing: the compute job is skipped.
*/
static void
set_up_indirect_dispatch(v3d_device* device, v3d_job* job)
{
	v3d_cpu_job* cpu = job->cpu;
	v3d_job* csd = cpu->csd_job;
	if (device->queue_head != csd) {
		// cannot happen: both were queued together
		ERROR("indirect dispatch: its compute job is not next\n");
		return;
	}

	uint32 counts[3];
	memcpy(counts, (uint8*)job->bos[0]->address + cpu->offset, sizeof(counts));
	if (counts[0] == 0 || counts[1] == 0 || counts[2] == 0) {
		csd->skip = true;
		return;
	}

	for (int32 i = 0; i < 3; i++)
		csd->csd_config[i] = counts[i] << kWorkgroupCountShift;
	// 16 invocations per batch; V3D 4.2 takes the batch count minus one
	const uint64 batches = (uint64)((cpu->wg_size + 15) / 16)
		* counts[0] * counts[1] * counts[2];
	if (batches == 0 || batches > UINT32_MAX) {
		ERROR("indirect dispatch: %" B_PRIu64 " batches, skipped\n", batches);
		csd->skip = true;
		return;
	}
	csd->csd_config[4] = (uint32)(batches - 1);

	uint32* uniforms = (uint32*)cpu->indirect->address;
	for (int32 i = 0; i < 3; i++) {
		if (cpu->wg_uniform_offsets[i] != kNoUniformRewrite)
			uniforms[cpu->wg_uniform_offsets[i]] = counts[i];
	}
}


/*!	Runs the job: every job before it has completed, the next one has not
	started. Called by the worker with the device lock held.
*/
void
v3d_cpu_job_run(v3d_device* device, v3d_job* job)
{
	v3d_cpu_job* cpu = job->cpu;

	switch (cpu->type) {
		case DRM_V3D_EXT_ID_CPU_TIMESTAMP_QUERY:
		{
			// CLOCK_MONOTONIC in nanoseconds, as Linux ktime_get_ns();
			// further queries of the same command (multiview) read 0
			const uint64 now = (uint64)system_time() * 1000;
			uint8* values = (uint8*)job->bos[0]->address;
			for (uint32 i = 0; i < cpu->count; i++) {
				const uint64 value = i == 0 ? now : 0;
				memcpy(values + cpu->offsets[i], &value, sizeof(value));
				// signals when this job completes, right after this
				cpu->syncs[i]->has_fence = true;
				cpu->syncs[i]->seqno = job->seqno;
			}
			break;
		}

		case DRM_V3D_EXT_ID_CPU_RESET_TIMESTAMP_QUERY:
		{
			uint8* values = (uint8*)job->bos[0]->address;
			for (uint32 i = 0; i < cpu->count; i++) {
				memset(values + cpu->offsets[i], 0, sizeof(uint64));
				cpu->syncs[i]->has_fence = false;
				cpu->syncs[i]->seqno = 0;
			}
			break;
		}

		case DRM_V3D_EXT_ID_CPU_COPY_TIMESTAMP_QUERY:
		{
			uint8* data = (uint8*)job->bos[0]->address + cpu->offset;
			const uint8* values = (const uint8*)job->bos[1]->address;
			for (uint32 i = 0; i < cpu->count; i++, data += cpu->stride) {
				const v3d_syncobj* syncobj = cpu->syncs[i];
				const bool available = syncobj->has_fence
					&& v3d_seqno_completed(device, syncobj->seqno);
				if (available || cpu->do_partial) {
					uint64 value;
					memcpy(&value, values + cpu->offsets[i], sizeof(value));
					write_result(data, 0, cpu->do_64bit, value);
				}
				if (cpu->availability_bit)
					write_result(data, 1, cpu->do_64bit, available ? 1 : 0);
			}
			break;
		}

		case DRM_V3D_EXT_ID_CPU_INDIRECT_CSD:
			set_up_indirect_dispatch(device, job);
			break;
	}

	// The BOs are write-combining: what was written reaches memory before
	// the next job starts, or the client is told.
	v3d_barrier();
}


/*!	Called with the device lock held if anything is referenced. */
void
v3d_cpu_job_free(v3d_device* device, v3d_cpu_job* cpu)
{
	if (cpu->syncs != NULL) {
		for (uint32 i = 0; i < cpu->count; i++) {
			if (cpu->syncs[i] != NULL)
				v3d_syncobj_put(cpu->syncs[i]);
		}
	}
	if (cpu->indirect != NULL)
		v3d_bo_put(device, cpu->indirect);

	free(cpu->syncs);
	free(cpu->sync_handles);
	free(cpu->offsets);
	free(cpu);
}
