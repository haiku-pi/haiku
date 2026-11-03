/*
 * Copyright 2009, Ingo Weinhold, ingo_weinhold@gmx.de.
 * Copyright 2013, Rene Gollent, rene@gollent.com.
 * Distributed under the terms of the MIT License.
 */
#ifndef BASE_UNIT_H
#define BASE_UNIT_H


#include <String.h>

#include <Array.h>

#include "Types.h"


class AbbreviationTable;
class DebugInfoEntry;
struct SourceLanguageInfo;


enum dwarf_unit_kind {
	dwarf_unit_kind_compilation = 0,
	dwarf_unit_kind_type
};


class BaseUnit {
public:
								BaseUnit(off_t headerOffset,
									off_t contentOffset,
									off_t totalSize,
									off_t abbreviationOffset,
									uint8 addressSize, bool isBigEndian,
									bool isDwarf64, uint16 version);
	virtual						~BaseUnit();

			off_t				HeaderOffset() const { return fHeaderOffset; }
			off_t				ContentOffset() const { return fContentOffset; }
			off_t 				RelativeContentOffset() const
									{ return fContentOffset - fHeaderOffset; }
			off_t				TotalSize() const	{ return fTotalSize; }
			off_t				ContentSize() const
									{ return fTotalSize
										- RelativeContentOffset(); }
			off_t				AbbreviationOffset() const
									{ return fAbbreviationOffset; }

			bool				ContainsAbsoluteOffset(off_t offset) const;

			uint8				AddressSize() const	{ return fAddressSize; }
			bool				IsBigEndian() const { return fIsBigEndian; }
			bool				IsDwarf64() const	{ return fIsDwarf64; }
			uint16				Version() const		{ return fVersion; }

			// DWARF 5: the unit's contributions to the .debug_str_offsets,
			// .debug_addr, .debug_rnglists and .debug_loclists sections
			// (DW_AT_*_base of the unit entry); -1 if not given
			off_t				StrOffsetsBase() const
									{ return fStrOffsetsBase; }
			void				SetStrOffsetsBase(off_t base)
									{ fStrOffsetsBase = base; }
			off_t				AddrBase() const	{ return fAddrBase; }
			void				SetAddrBase(off_t base)
									{ fAddrBase = base; }
			off_t				RangeListsBase() const
									{ return fRangeListsBase; }
			void				SetRangeListsBase(off_t base)
									{ fRangeListsBase = base; }
			off_t				LocationListsBase() const
									{ return fLocationListsBase; }
			void				SetLocationListsBase(off_t base)
									{ fLocationListsBase = base; }

			AbbreviationTable*	GetAbbreviationTable() const
									{ return fAbbreviationTable; }
			void				SetAbbreviationTable(
									AbbreviationTable* abbreviationTable);

			const SourceLanguageInfo* SourceLanguage() const
									{ return fSourceLanguage; }
			void				SetSourceLanguage(
									const SourceLanguageInfo* language);

			status_t			AddDebugInfoEntry(DebugInfoEntry* entry,
									off_t offset);
			int					CountEntries() const;
			void				GetEntryAt(int index, DebugInfoEntry*& entry,
									off_t& offset) const;
			DebugInfoEntry*		EntryForOffset(off_t offset) const;

	virtual	dwarf_unit_kind		Kind() const = 0;

private:
			off_t				fHeaderOffset;
			off_t				fContentOffset;
			off_t				fTotalSize;
			off_t				fAbbreviationOffset;
			off_t				fStrOffsetsBase;
			off_t				fAddrBase;
			off_t				fRangeListsBase;
			off_t				fLocationListsBase;
			AbbreviationTable*	fAbbreviationTable;
			const SourceLanguageInfo* fSourceLanguage;
			Array<DebugInfoEntry*> fEntries;
			Array<off_t>		fEntryOffsets;
			uint8				fAddressSize;
			bool				fIsBigEndian;
			bool				fIsDwarf64;
			uint16				fVersion;
};


#endif	// BASE_UNIT_H
