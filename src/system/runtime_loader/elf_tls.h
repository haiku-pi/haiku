/*
 * Copyright 2014, Paweł Dziepak, pdziepak@quarnos.org.
 * Distributed under the terms of the MIT License.
 */
#ifndef ELF_TLS_H
#define ELF_TLS_H


#include "runtime_loader_private.h"
#include "utility.h"


class TLSBlock;

// The TLS of the images loaded at program start (before their init routines
// run) goes into one block per thread, found through TLS_STATIC_TLS_SLOT
// instead of the dynamic thread vector. Only arm64's __tls_get_addr() knows it
// so far: the relocation code passes TLS_STATIC_MODULE | offset as the module.
#if defined(__aarch64__)
#	define TLS_STATIC_BLOCK
#endif


class TLSBlockTemplate {
public:
						TLSBlockTemplate()
							: fStaticOffset(kNoStaticOffset) { }
	inline				TLSBlockTemplate(void* address, size_t fileSize,
							size_t memorySize, size_t alignment);

			void		SetGeneration(unsigned generation)
							{ fGeneration = generation; }
			unsigned	Generation() const	{ return fGeneration; }

			void		SetBaseAddress(addr_t baseAddress);

			size_t		MemorySize() const	{ return fMemorySize; }
			size_t		Alignment() const	{ return fAlignment; }

			bool		IsStatic() const
							{ return fStaticOffset != kNoStaticOffset; }
			unsigned	StaticOffset() const	{ return fStaticOffset; }
			void		SetStaticOffset(unsigned offset)
							{ fStaticOffset = offset; }

			void		CopyInitImage(void* to) const;
			void		ForgetInitImage();

			TLSBlock	CreateBlock();

	static	const unsigned	kNoStaticOffset = unsigned(-1);

private:
			unsigned	fGeneration;
			unsigned	fStaticOffset;

			void*		fAddress;
			size_t		fFileSize;
			size_t		fMemorySize;
			size_t		fAlignment;
};

class TLSBlockTemplates {
public:
	static	TLSBlockTemplates&	Get();

			unsigned	Register(const TLSBlockTemplate& block);
			void		Unregister(unsigned dso);

			void		SetBaseAddress(unsigned dso, addr_t baseAddress);

			unsigned	GetGeneration(unsigned dso) const;

			TLSBlock	CreateBlock(unsigned dso);

			addr_t		RelocationModule(unsigned dso) const;
			bool		GetStaticOffset(unsigned dso,
							unsigned& _offset) const;
			void		FreezeStaticBlock()
							{ fStaticBlockFrozen = true; }
			void*		CreateStaticBlock();
	static	void		DeleteStaticBlock(void* block);

private:
	inline				TLSBlockTemplates();

			void		_AddToStaticBlock(TLSBlockTemplate& block);

	static	TLSBlockTemplates*	fInstance;

			unsigned	fGeneration;

			size_t		fStaticBlockSize;
			size_t		fStaticBlockAlignment;
			bool		fStaticBlockFrozen;

			utility::vector<TLSBlockTemplate>	fTemplates;
			utility::vector<unsigned>	fFreeDSOs;
};


TLSBlockTemplate::TLSBlockTemplate(void* address, size_t fileSize,
	size_t memorySize, size_t alignment)
	:
	fStaticOffset(kNoStaticOffset),
	fAddress(address),
	fFileSize(fileSize),
	fMemorySize(memorySize),
	fAlignment(alignment)
{
}


void*	get_tls_address(unsigned dso, addr_t offset);
void	destroy_thread_tls();
void	freeze_static_tls_block();


#endif	// ELF_TLS_H
