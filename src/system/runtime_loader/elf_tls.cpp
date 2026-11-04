/*
 * Copyright 2014, Paweł Dziepak, pdziepak@quarnos.org.
 * Distributed under the terms of the MIT License.
 */

#include "elf_tls.h"

#include <stdlib.h>
#include <string.h>

#include <support/TLS.h>

#include <tls.h>

#include <util/kernel_cpp.h>


// The dynamic thread vector's layout is also known to __tls_get_addr() in
// libroot, which reads it directly: entry 0 points to the thread's
// Generation, entry dso + 1 is the TLSBlock of that DSO.
class TLSBlock {
public:
	inline				TLSBlock();
	inline				TLSBlock(void* pointer);

	inline	status_t	Initialize(unsigned dso);

			void		Destroy();

			bool		IsInvalid() const	{ return fPointer == NULL; }

			void*		operator+(addr_t offset) const
							{ return (void*)((addr_t)fPointer + TLS_DTV_OFFSET + offset); }

private:
			void*		fPointer;
};

class Generation {
public:
	inline				Generation();

			unsigned	Counter() const	{ return fCounter; }
			unsigned	Size() const	{ return fSize; }

			void		SetCounter(unsigned counter)	{ fCounter = counter; }
			void		SetSize(unsigned size)	{ fSize = size; }

private:
			unsigned	fCounter;
			unsigned	fSize;
};

class DynamicThreadVector {
public:
	inline				DynamicThreadVector();

			void		DestroyAll();

	inline	TLSBlock&	operator[](unsigned dso);

private:
			bool		_DoesExist() const	{ return *fVector != NULL; }
			unsigned	_Size() const
							{ return _DoesExist()
									? fGeneration->Size() : 0; }

			unsigned	_Generation() const;

			status_t	_ResizeVector(unsigned minimumSize);

			TLSBlock**	fVector;
			Generation*	fGeneration;
			TLSBlock	fNullBlock;
};


TLSBlockTemplates*	TLSBlockTemplates::fInstance;


/*!	Allocates a TLS block whose data, \a dataOffset bytes into it, is aligned
	to \a alignment (a power of two; the heap aligns to less than a TLS
	segment may ask for). The allocation's address is kept in front of the
	block, for free_tls_block().
*/
static void*
allocate_tls_block(size_t size, size_t alignment, size_t dataOffset)
{
	alignment = std::max(alignment, sizeof(void*));
	void* allocation = malloc(sizeof(void*) + dataOffset + size
		+ alignment - 1);
	if (allocation == NULL)
		return NULL;

	addr_t data = ((addr_t)allocation + sizeof(void*) + dataOffset
		+ alignment - 1) & ~(addr_t)(alignment - 1);
	void** block = (void**)(data - dataOffset);
	block[-1] = allocation;
	return block;
}


static void
free_tls_block(void* block)
{
	if (block != NULL)
		free(((void**)block)[-1]);
}


void
TLSBlockTemplate::SetBaseAddress(addr_t baseAddress)
{
	fAddress = (void*)((addr_t)fAddress + baseAddress);
}


void
TLSBlockTemplate::CopyInitImage(void* to) const
{
	memcpy(to, fAddress, fFileSize);
	if (fMemorySize > fFileSize)
		memset((char*)to + fFileSize, 0, fMemorySize - fFileSize);
}


void
TLSBlockTemplate::ForgetInitImage()
{
	// The image is unmapped: blocks made from now on start out zeroed.
	fAddress = NULL;
	fFileSize = 0;
}


TLSBlock
TLSBlockTemplate::CreateBlock()
{
	void* pointer = allocate_tls_block(fMemorySize, fAlignment, TLS_DTV_OFFSET);
	if (pointer == NULL)
		return TLSBlock();
	memset(pointer, 0, TLS_DTV_OFFSET);
	CopyInitImage((char*)pointer + TLS_DTV_OFFSET);
	return TLSBlock(pointer);
}


TLSBlockTemplates&
TLSBlockTemplates::Get()
{
	if (fInstance == NULL)
		fInstance = new TLSBlockTemplates;
	return *fInstance;
}


unsigned
TLSBlockTemplates::Register(const TLSBlockTemplate& block)
{
	unsigned dso;

	if (!fFreeDSOs.empty()) {
		dso = fFreeDSOs.back();
		fFreeDSOs.pop_back();
		fTemplates[dso] = block;
	} else {
		dso = fTemplates.size();
		fTemplates.push_back(block);
	}

	fTemplates[dso].SetGeneration(fGeneration);
#ifdef TLS_STATIC_BLOCK
	if (!fStaticBlockFrozen)
		_AddToStaticBlock(fTemplates[dso]);
#endif
	return dso;
}


void
TLSBlockTemplates::Unregister(unsigned dso)
{
	if (dso == unsigned(-1))
		return;

	if (fTemplates[dso].IsStatic()) {
		// Its place in the static block is taken for good: the module ID is
		// never handed out again, so no thread's block for it gets stale.
		fTemplates[dso].ForgetInitImage();
		return;
	}

	fGeneration++;
	fFreeDSOs.push_back(dso);
}


void
TLSBlockTemplates::SetBaseAddress(unsigned dso, addr_t baseAddress)
{
	if (dso != unsigned(-1))
		fTemplates[dso].SetBaseAddress(baseAddress);
}


unsigned
TLSBlockTemplates::GetGeneration(unsigned dso) const
{
	if (dso == unsigned(-1))
		return fGeneration;
	return fTemplates[dso].Generation();
}


TLSBlock
TLSBlockTemplates::CreateBlock(unsigned dso)
{
	return fTemplates[dso].CreateBlock();
}


/*!	Returns the value of a DTPMOD relocation for the module: its ID, or for
	a module in the static block TLS_STATIC_MODULE | its offset there.
*/
addr_t
TLSBlockTemplates::RelocationModule(unsigned dso) const
{
	unsigned offset;
	if (GetStaticOffset(dso, offset))
		return TLS_STATIC_MODULE | offset;
	return dso;
}


bool
TLSBlockTemplates::GetStaticOffset(unsigned dso, unsigned& _offset) const
{
	if (dso >= fTemplates.size() || !fTemplates[dso].IsStatic())
		return false;

	_offset = fTemplates[dso].StaticOffset();
	return true;
}


/*!	Allocates and initializes a thread's static block. Its layout is fixed
	from then on: modules registered later get blocks of their own.
*/
void*
TLSBlockTemplates::CreateStaticBlock()
{
	fStaticBlockFrozen = true;

	char* block = (char*)allocate_tls_block(fStaticBlockSize,
		fStaticBlockAlignment, 0);
	if (block == NULL)
		return NULL;

	for (unsigned dso = 0; dso < fTemplates.size(); dso++) {
		if (fTemplates[dso].IsStatic())
			fTemplates[dso].CopyInitImage(block
				+ fTemplates[dso].StaticOffset());
	}

	return block;
}


/*static*/ void
TLSBlockTemplates::DeleteStaticBlock(void* block)
{
	free_tls_block(block);
}


void
TLSBlockTemplates::_AddToStaticBlock(TLSBlockTemplate& block)
{
	size_t alignment = block.Alignment();
	size_t offset = (fStaticBlockSize + alignment - 1) & ~(alignment - 1);
	if (offset >= TLS_STATIC_MODULE
		|| block.MemorySize() >= TLS_STATIC_MODULE - offset) {
		// the offset would not fit into a module ID
		return;
	}

	block.SetStaticOffset(offset);
	fStaticBlockSize = offset + block.MemorySize();
	fStaticBlockAlignment = std::max(fStaticBlockAlignment, alignment);
}


TLSBlockTemplates::TLSBlockTemplates()
	:
	fGeneration(0),
	fStaticBlockSize(0),
	fStaticBlockAlignment(1),
	fStaticBlockFrozen(false)
{
	gRuntimeLoader.tls_generation = &fGeneration;
}


TLSBlock::TLSBlock()
	:
	fPointer(NULL)
{
}


TLSBlock::TLSBlock(void* pointer)
	:
	fPointer(pointer)
{
}


status_t
TLSBlock::Initialize(unsigned dso)
{
	fPointer = TLSBlockTemplates::Get().CreateBlock(dso).fPointer;
	return fPointer != NULL ? B_OK : B_NO_MEMORY;
}


void
TLSBlock::Destroy()
{
	free_tls_block(fPointer);
	fPointer = NULL;
}


Generation::Generation()
	:
	fCounter(0),
	fSize(0)
{
}


DynamicThreadVector::DynamicThreadVector()
	:
	fVector((TLSBlock**)tls_address(TLS_DYNAMIC_THREAD_VECTOR)),
	fGeneration(NULL)
{
	if (*fVector != NULL)
		fGeneration = (Generation*)*(void**)*fVector;
}


void
DynamicThreadVector::DestroyAll()
{
	for (unsigned i = 0; i < _Size(); i++) {
		TLSBlock& block = (*fVector)[i + 1];
		if (!block.IsInvalid())
			block.Destroy();
	}

	free(*fVector);
	*fVector = NULL;

	delete fGeneration;
}


TLSBlock&
DynamicThreadVector::operator[](unsigned dso)
{
	unsigned generation = TLSBlockTemplates::Get().GetGeneration(-1);
	if (_Generation() < generation) {
		// We need to destroy any blocks whose DSO generation has changed
		// to be greater than our own generation.
		for (unsigned dsoIndex = 0; dsoIndex < _Size(); dsoIndex++) {
			TLSBlock& block = (*fVector)[dsoIndex + 1];
			unsigned dsoGeneration
				= TLSBlockTemplates::Get().GetGeneration(dsoIndex);
			if (dsoGeneration > _Generation() && dsoGeneration <= generation)
				block.Destroy();
		}

		fGeneration->SetCounter(generation);
	}

	if (_Size() <= dso) {
		status_t result = _ResizeVector(dso + 1);
		if (result != B_OK)
			return fNullBlock;
	}

	TLSBlock& block = (*fVector)[dso + 1];
	if (block.IsInvalid()) {
		status_t result = block.Initialize(dso);
		if (result != B_OK)
			return fNullBlock;
	}

	return block;
}


unsigned
DynamicThreadVector::_Generation() const
{
	if (fGeneration != NULL)
		return fGeneration->Counter();
	return unsigned(-1);
}


status_t
DynamicThreadVector::_ResizeVector(unsigned minimumSize)
{
	static const unsigned kInitialSize = 4;
	unsigned size = std::max(minimumSize, kInitialSize);
	unsigned oldSize = _Size();
	if (size <= oldSize)
		return B_OK;

	void* newVector = realloc(*fVector, (size + 1) * sizeof(TLSBlock));
	if (newVector == NULL)
		return B_NO_MEMORY;

	*fVector = (TLSBlock*)newVector;
	memset((void*)(*fVector + oldSize + 1), 0, (size - oldSize) * sizeof(TLSBlock));
	if (fGeneration == NULL) {
		fGeneration = new Generation;
		if (fGeneration == NULL)
			return B_NO_MEMORY;

		// A new vector has no blocks yet, so none of them can be stale: it
		// is as current as the templates. At 0, once any TLS image had been
		// unloaded (generation 1 and up), the thread's next access destroyed
		// the blocks it had just made, and a thread-local value written once
		// read back as 0.
		fGeneration->SetCounter(TLSBlockTemplates::Get().GetGeneration(-1));
	}

	*(Generation**)*fVector = fGeneration;
	fGeneration->SetSize(size);

	return B_OK;
}


#ifdef TLS_STATIC_BLOCK
static const bool kHaveStaticBlock = true;
#else
static const bool kHaveStaticBlock = false;
#endif


static void*
get_static_tls_address(unsigned staticOffset, addr_t offset)
{
	void** block = tls_address(TLS_STATIC_TLS_SLOT);
	if (*block == NULL) {
		*block = TLSBlockTemplates::Get().CreateStaticBlock();
		if (*block == NULL)
			return NULL;
	}
	return (char*)*block + staticOffset + offset;
}


void*
get_tls_address(unsigned dso, addr_t offset)
{
	// __tls_get_addr() passes a module in the static block as
	// TLS_STATIC_MODULE | offset (the relocation's value), other callers may
	// pass its module ID: both get the same block. Without a static block
	// no module is in one, and every thread-local access of a library comes
	// through here: skip the lookup then.
	if (kHaveStaticBlock) {
		if (dso != unsigned(-1) && (dso & TLS_STATIC_MODULE) != 0)
			return get_static_tls_address(dso & ~TLS_STATIC_MODULE, offset);

		unsigned staticOffset;
		if (TLSBlockTemplates::Get().GetStaticOffset(dso, staticOffset))
			return get_static_tls_address(staticOffset, offset);
	}

	DynamicThreadVector dynamicThreadVector;
	TLSBlock& block = dynamicThreadVector[dso];
	if (block.IsInvalid())
		return NULL;
	return block + offset;
}


void
destroy_thread_tls()
{
	DynamicThreadVector().DestroyAll();

	void** staticBlock = tls_address(TLS_STATIC_TLS_SLOT);
	TLSBlockTemplates::DeleteStaticBlock(*staticBlock);
	*staticBlock = NULL;
}


/*!	Called once the images loaded at program start are registered, before
	any of their code runs: images loaded later get blocks of their own.
*/
void
freeze_static_tls_block()
{
	TLSBlockTemplates::Get().FreezeStaticBlock();
}

