/*
 * Copyright 2009, Ingo Weinhold, ingo_weinhold@gmx.de.
 * Copyright 2013, Rene Gollent, rene@gollent.com.
 * Distributed under the terms of the MIT License.
 */
#ifndef COMPILATION_UNIT_H
#define COMPILATION_UNIT_H

#include <Array.h>
#include <ObjectList.h>
#include <String.h>

#include "BaseUnit.h"
#include "LineNumberProgram.h"
#include "Types.h"


class DIECompileUnitBase;
class TargetAddressRangeList;


class CompilationUnit : public BaseUnit {
public:
								CompilationUnit(off_t headerOffset,
									off_t contentOffset,
									off_t totalSize,
									off_t abbreviationOffset,
									uint8 addressSize, bool isBigEndian,
									bool isDwarf64, uint16 version);
	virtual						~CompilationUnit();

	inline	target_addr_t		MaxAddress() const;

			DIECompileUnitBase*	UnitEntry() const	{ return fUnitEntry; }
			void				SetUnitEntry(DIECompileUnitBase* entry);

			TargetAddressRangeList* AddressRanges() const
									{ return fAddressRanges; }
			void				SetAddressRanges(
									TargetAddressRangeList* ranges);

			target_addr_t		AddressRangeBase() const;

			LineNumberProgram&	GetLineNumberProgram()
									{ return fLineNumberProgram; }

			bool				AddDirectory(const char* directory);
			int32				CountDirectories() const;
			const char*			DirectoryAt(int32 index) const;

			bool				AddFile(const char* fileName, int32 dirIndex);
			int32				CountFiles() const;
			const char*			FileAt(int32 index,
									const char** _directory = NULL) const;

			// File numbers (DW_LNS_set_file, DW_AT_decl_file, ...) count from
			// 1 in DWARF 2-4 line number programs, where they are indices into
			// the file list (FileAt(number - 1)). A DWARF 5 line number
			// program header (which GCC also emits for DWARF 4 units) numbers
			// its file entries from 0, and the primary source file usually is
			// both entry 0 and 1: AddFileEntry() adds such an entry, sharing
			// the file of an earlier entry with the same name and directory.
			// FileIndex() maps a file number to the one-based index into the
			// file list (0, if there is no such file).
			bool				AddFileEntry(const char* fileName,
									int32 dirIndex);
			int32				FileIndex(uint64 fileNumber) const;

	virtual	dwarf_unit_kind		Kind() const;

private:
			struct File;
			typedef BObjectList<BString, true> DirectoryList;
			typedef BObjectList<File, true> FileList;

private:
			DIECompileUnitBase*	fUnitEntry;
			TargetAddressRangeList* fAddressRanges;
			DirectoryList		fDirectories;
			FileList			fFiles;
			Array<int32>		fFileEntries;
				// DWARF 5: file entry -> one-based index into fFiles
			LineNumberProgram	fLineNumberProgram;
};


target_addr_t
CompilationUnit::MaxAddress() const
{
	return AddressSize() == 4 ? 0xffffffffULL : 0xffffffffffffffffULL;
}


#endif	// COMPILATION_UNIT_H
