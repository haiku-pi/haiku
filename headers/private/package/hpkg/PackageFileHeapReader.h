/*
 * Copyright 2013, Ingo Weinhold, ingo_weinhold@gmx.de.
 * Distributed under the terms of the MIT License.
 */
#ifndef _PACKAGE__HPKG__PRIVATE__PACKAGE_FILE_HEAP_READER_H_
#define _PACKAGE__HPKG__PRIVATE__PACKAGE_FILE_HEAP_READER_H_


#include <Array.h>
#include <package/hpkg/PackageFileHeapAccessorBase.h>


namespace BPackageKit {

namespace BHPKG {


class BDataReader;
class BErrorOutput;


namespace BPrivate {


class PackageFileHeapReader : public PackageFileHeapAccessorBase {
public:
								PackageFileHeapReader(BErrorOutput* errorOutput,
									BPositionIO* file, off_t heapOffset,
									off_t compressedHeapSize,
									uint64 uncompressedHeapSize,
									DecompressionAlgorithmOwner*
										decompressionAlgorithm);
								~PackageFileHeapReader();

			status_t			Init();

			PackageFileHeapReader* Clone() const;

			const OffsetArray&	Offsets() const
									{ return fOffsets; }

protected:
	virtual	status_t			ReadAndDecompressChunk(size_t chunkIndex,
									void* compressedDataBuffer,
									void* uncompressedDataBuffer,
									iovec* scratchBuffer = NULL);

private:
#ifdef _BOOT_MODE
			bool				_ReadCachedChunk(size_t chunkIndex,
									void* buffer, size_t size);
			void				_CacheChunk(size_t chunkIndex,
									const void* buffer, size_t size);
#endif

			OffsetArray			fOffsets;
#ifdef _BOOT_MODE
	// The boot loader has no file cache and reads a file in many small pieces
	// (ELF headers, segments, symbols), each of which would otherwise read and
	// decompress its whole chunk again: keep the last few chunks.
	static	const int			kCachedChunkCount = 8;

			uint8*				fChunkCache;
			size_t				fCachedChunkIndex[kCachedChunkCount];
			uint32				fCachedChunkLastUse[kCachedChunkCount];
			uint32				fChunkCacheUseCounter;
#endif
};


}	// namespace BPrivate

}	// namespace BHPKG

}	// namespace BPackageKit


#endif	// _PACKAGE__HPKG__PRIVATE__PACKAGE_FILE_HEAP_READER_H_
