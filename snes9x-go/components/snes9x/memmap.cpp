/*****************************************************************************\
     Snes9x - Portable Super Nintendo Entertainment System (TM) emulator.
                This file is licensed under the Snes9x License.
   For further information, consult the LICENSE file in the root directory.
\*****************************************************************************/

#include <string>
#include <numeric>
#include <assert.h>
#include <ctype.h>

#include "snes9x.h"
#include "memmap.h"
#include "apu/apu.h"
#include "controls.h"
#include "display.h"

#ifndef SET_UI_COLOR
#define SET_UI_COLOR(r, g, b) ;
#endif

#undef max
#define max(a, b) ({__typeof__(a) _a = (a); __typeof__(b) _b = (b);_a > _b ? _a : _b; })
#undef min
#define min(a, b) ({__typeof__(a) _a = (a); __typeof__(b) _b = (b);_a < _b ? _a : _b; })

extern uint32 crc32_le(uint32 crc, uint8 const * buf, uint32 len);

static int First512BytesCountZeroes(const uint8 *buf)
{
	int zeroCount = 0;
	for (int i = 0; i < 512; i++)
	{
		if (buf[i] == 0)
			zeroCount++;
	}
	return zeroCount;
}

static void sanitize(char *b, int size)
{
	for (int pos = 0; pos < size; pos++)
	{
		if (b[pos] == 0)
			continue;
		if (b[pos] < 32 || b[pos] > 126)
			b[pos] = '_';
	}
	b[size - 1] = 0;
}

static bool8 allASCII (uint8 *b, int size)
{
	for (int i = 0; i < size; i++)
	{
		if (b[i] < 32 || b[i] > 126)
			return (FALSE);
	}

	return (TRUE);
}

// allocation and deallocation

bool8 CMemory::Init (void)
{
	FillRAM = (uint8 *) calloc(1, 0x2800);
    RAM	 = (uint8 *) calloc(1, 0x20000);
    VRAM = (uint8 *) calloc(1, 0x10000);
    SRAM = (uint8 *) calloc(1, 0x20000);
    ROM  = (uint8 *) calloc(1, ROM_BUFFER_SIZE + 0x200);

	IPPU.TileCacheData = (uint8 *) calloc(4096, 64);

	if (!FillRAM || !RAM || !SRAM || !VRAM || !ROM || !IPPU.TileCacheData)
    {
		Deinit();
		return (FALSE);
    }

	// We never access 0x0000 - 0x2000 so we shift our offset to save 8K of ram
	FillRAM -= 0x2000;

	return (TRUE);
}

void CMemory::Deinit (void)
{
	if (FillRAM)
	{
		FillRAM += 0x2000;
		free(FillRAM);
		FillRAM = NULL;
	}

	free(RAM);
	free(SRAM);
	free(VRAM);
	free(ROM);
	free(IPPU.TileCacheData);

	RAM = NULL;
	SRAM = NULL;
	VRAM = NULL;
	ROM = NULL;
	IPPU.TileCacheData = NULL;
}

// file management and ROM detection

int CMemory::ScoreHiROM (bool8 skip_header, int32 romoff)
{
	uint8	*buf = ROM + 0xff00 + romoff + (skip_header ? 0x200 : 0);
	int		score = 0;

	if (buf[0xd5] & 0x1)
		score += 2;

	// Mode23 is SA-1
	if (buf[0xd5] == 0x23)
		score -= 2;

	if (buf[0xd4] == 0x20)
		score += 2;

	if ((buf[0xdc] + (buf[0xdd] << 8)) + (buf[0xde] + (buf[0xdf] << 8)) == 0xffff)
	{
		score += 2;
		if (0 != (buf[0xde] + (buf[0xdf] << 8)))
			score++;
	}

	if (buf[0xda] == 0x33)
		score += 2;

	if ((buf[0xd5] & 0xf) < 4)
		score += 2;

	if (!(buf[0xfd] & 0x80))
		score -= 6;

	if ((buf[0xfc] + (buf[0xfd] << 8)) > 0xffb0)
		score -= 2; // reduced after looking at a scan by Cowering

	if (CalculatedSize > 1024 * 1024 * 3)
		score += 4;

	if ((1 << (buf[0xd7] - 7)) > 48)
		score -= 1;

	if (!allASCII(&buf[0xb0], 6))
		score -= 1;

	if (!allASCII(&buf[0xc0], ROM_NAME_LEN - 1))
		score -= 1;

	return (score);
}

int CMemory::ScoreLoROM (bool8 skip_header, int32 romoff)
{
	uint8	*buf = ROM + 0x7f00 + romoff + (skip_header ? 0x200 : 0);
	int		score = 0;

	if (!(buf[0xd5] & 0x1))
		score += 3;

	// Mode23 is SA-1
	if (buf[0xd5] == 0x23)
		score += 2;

	if ((buf[0xdc] + (buf[0xdd] << 8)) + (buf[0xde] + (buf[0xdf] << 8)) == 0xffff)
	{
		score += 2;
		if (0 != (buf[0xde] + (buf[0xdf] << 8)))
			score++;
	}

	if (buf[0xda] == 0x33)
		score += 2;

	if ((buf[0xd5] & 0xf) < 4)
		score += 2;

	if (!(buf[0xfd] & 0x80))
		score -= 6;

	if ((buf[0xfc] + (buf[0xfd] << 8)) > 0xffb0)
		score -= 2; // reduced per Cowering suggestion

	if (CalculatedSize <= 1024 * 1024 * 16)
		score += 2;

	if ((1 << (buf[0xd7] - 7)) > 48)
		score -= 1;

	if (!allASCII(&buf[0xb0], 6))
		score -= 1;

	if (!allASCII(&buf[0xc0], ROM_NAME_LEN - 1))
		score -= 1;

	return (score);
}

bool8 CMemory::LoadROMMem (const uint8 *source, uint32 sourceSize)
{
	if (source == 0 || sourceSize > ROM_BUFFER_SIZE)
		return FALSE;

	memset(ROM, 0, ROM_BUFFER_SIZE);
	memcpy(ROM, source, sourceSize);

	ROM_SIZE = sourceSize;

	return InitROM();
}

bool8 CMemory::LoadROM (const char *filename)
{
	FILE *fp = fopen(filename, "rb");
	if (!fp)
		return (FALSE);

	fseek(fp, 0, SEEK_END);

	ROM_SIZE = ftell(fp);

	fseek(fp, 0, SEEK_SET);
	fread(ROM, ROM_BUFFER_SIZE + 0x200, 1, fp);

	fclose(fp);

	return InitROM();
}

bool8 CMemory::LoadROMBlock (int b)
{
	return FALSE;
}

bool8 CMemory::InitROM ()
{
	if (!ROM || ROM_SIZE < 0x200)
		return (FALSE);

	Settings.DisplayColor = BUILD_PIXEL(31, 31, 31);
	SET_UI_COLOR(255, 255, 255);

	if ((ROM_SIZE & 0x7FF) == 512 || First512BytesCountZeroes(ROM) > 400)
	{
		S9xMessage(S9X_INFO, S9X_HEADERS_INFO, "Found ROM file header (and ignored it).");
		memmove(ROM, ROM + 512, ROM_BUFFER_SIZE - 512);
		ROM_SIZE -= 512;
	}

	CalculatedSize = ((ROM_SIZE + 0x1fff) / 0x2000) * 0x2000;
	ExtendedFormat = NOPE;

	if (CalculatedSize > 0x400000 &&
		(ROM[0x7fd5] + (ROM[0x7fd6] << 8)) != 0x3423 && // exclude SA-1
		(ROM[0x7fd5] + (ROM[0x7fd6] << 8)) != 0x3523 &&
		(ROM[0x7fd5] + (ROM[0x7fd6] << 8)) != 0x4332 && // exclude S-DD1
		(ROM[0x7fd5] + (ROM[0x7fd6] << 8)) != 0x4532 &&
		(ROM[0xffd5] + (ROM[0xffd6] << 8)) != 0xF93a && // exclude SPC7110
		(ROM[0xffd5] + (ROM[0xffd6] << 8)) != 0xF53a)
		ExtendedFormat = YEAH;

	// CalculatedSize is now set, so rescore
	int hi_score = ScoreHiROM(FALSE);
	int lo_score = ScoreLoROM(FALSE);

	if (ExtendedFormat != NOPE)
	{
		int swappedhirom = ScoreHiROM(FALSE, 0x400000);
		int swappedlorom = ScoreLoROM(FALSE, 0x400000);

		// set swapped here
		if (max(swappedlorom, swappedhirom) >= max(lo_score, hi_score))
		{
			ExtendedFormat = BIGFIRST;
			hi_score = swappedhirom;
			lo_score = swappedlorom;
		}
		else
			ExtendedFormat = SMALLFIRST;
	}

	if (Settings.ForceLoROM || (!Settings.ForceHiROM && lo_score >= hi_score))
	{
		LoROM = TRUE;
		HiROM = FALSE;
	}
	else
	{
		LoROM = FALSE;
		HiROM = TRUE;
	}

	// this two games fail to be detected
	if (!Settings.ForceHiROM && !Settings.ForceLoROM)
	{
		if (strncmp((char *) &ROM[0x7fc0], "YUYU NO QUIZ DE GO!GO!", 22) == 0 ||
		   (strncmp((char *) &ROM[0xffc0], "BATMAN--REVENGE JOKER",  21) == 0))
		{
			LoROM = TRUE;
			HiROM = FALSE;
		}
	}

	if (ExtendedFormat == SMALLFIRST)
	{
		RG_PANIC("ExHiROM swapping not implemented yet");
		// uint8	*tmp = (uint8 *) malloc(CalculatedSize - 0x400000);
		// if (tmp)
		// {
		// 	S9xMessage(S9X_INFO, S9X_ROM_INTERLEAVED_INFO, "Fixing swapped ExHiROM...");
		// 	memmove(tmp, ROM, CalculatedSize - 0x400000);
		// 	memmove(ROM, ROM + CalculatedSize - 0x400000, 0x400000);
		// 	memmove(ROM + 0x400000, tmp, CalculatedSize - 0x400000);
		// 	free(tmp);
		// }
	}

	//// Parse ROM header and read ROM informatoin
	uint8	*RomHeader = ROM + 0x7FB0;

	if (ExtendedFormat == BIGFIRST)
		RomHeader += 0x400000;

	if (HiROM)
		RomHeader += 0x8000;

	ParseSNESHeader(RomHeader);

	//// Detect and initialize chips
	//// detection codes are compatible with NSRT

	// DSP 1 & 2
	if (ROMType == 0x03 || (ROMType == 0x05 && ROMSpeed != 0x20))
	{
		Settings.DSP = 1;
		if (HiROM)
		{
			DSP0.boundary = 0x7000;
			DSP0.maptype = M_DSP1_HIROM;
		}
		else
		if (CalculatedSize > 0x100000)
		{
			DSP0.boundary = 0x4000;
			DSP0.maptype = M_DSP1_LOROM_L;
		}
		else
		{
			DSP0.boundary = 0xc000;
			DSP0.maptype = M_DSP1_LOROM_S;
		}

		SetDSP = &DSP1SetByte;
		GetDSP = &DSP1GetByte;
	}
	else
	if (ROMType == 0x05 && ROMSpeed == 0x20)
	{
		Settings.DSP = 2;
		DSP0.boundary = 0x10000;
		DSP0.maptype = M_DSP2_LOROM;
		SetDSP = &DSP2SetByte;
		GetDSP = &DSP2GetByte;
	}
	else
	{
		Settings.DSP = 0;
		SetDSP = NULL;
		GetDSP = NULL;
	}


	//// Map memory and calculate checksum
	Map_Initialize();
	CalculatedChecksum = 0;

	if (HiROM)
    {
		if (ExtendedFormat != NOPE)
			Map_ExtendedHiROMMap();
		else
			Map_HiROMMap();
    }
    else
    {
		if (ExtendedFormat != NOPE)
			Map_JumboLoROMMap();
		else
		if (strncmp(ROMName, "WANDERERS FROM YS", 17) == 0)
			Map_NoMAD1LoROMMap();
		else
		if (strncmp(ROMName, "SOUND NOVEL-TCOOL", 17) == 0 ||
			strncmp(ROMName, "DERBY STALLION 96", 17) == 0)
			Map_ROM24MBSLoROMMap();
		else
		if (strncmp(ROMName, "THOROUGHBRED BREEDER3", 21) == 0 ||
			strncmp(ROMName, "RPG-TCOOL 2", 11) == 0)
			Map_SRAM512KLoROMMap();
		else
			Map_LoROMMap();
    }

	Checksum_Calculate();

	bool8 isChecksumOK = (ROMChecksum + ROMComplementChecksum == 0xffff) &
						 (ROMChecksum == CalculatedChecksum);

	//// Build more ROM information

	// CRC32
	ROMCRC32 = crc32_le(0, ROM, CalculatedSize);

	// NTSC / PAL
	Settings.PAL = (Settings.ForcePAL && !Settings.ForceNTSC)
						|| ((ROMRegion >= 2) && (ROMRegion <= 12));

	if (Settings.PAL)
	{
		Settings.FrameTime = Settings.FrameTimePAL;
		ROMFramesPerSecond = 50;
	}
	else
	{
		Settings.FrameTime = Settings.FrameTimeNTSC;
		ROMFramesPerSecond = 60;
	}

	// truncate cart name
	ROMName[ROM_NAME_LEN - 1] = 0;
	if (strlen(ROMName))
	{
		char *p = ROMName + strlen(ROMName);
		if (p > ROMName + 21 && ROMName[20] == ' ')
			p = ROMName + 21;
		while (p > ROMName && *(p - 1) == ' ')
			p--;
		*p = 0;
	}

	// SRAM size
	SRAMMask = SRAMSize ? ((1 << (SRAMSize + 3)) * 128) - 1 : 0;

	// checksum
	if (!isChecksumOK || ((uint32) CalculatedSize > (uint32) (((1 << (ROMSize - 7)) * 128) * 1024)))
	{
		Settings.DisplayColor = BUILD_PIXEL(31, 31, 0);
		SET_UI_COLOR(255, 255, 0);
	}

	// Use slight blue tint to indicate ROM was patched.
	if (Settings.IsPatched)
	{
		Settings.DisplayColor = BUILD_PIXEL(26, 26, 31);
		SET_UI_COLOR(216, 216, 255);
	}

	//// Initialize emulation
	Timings.H_Max_Master = SNES_CYCLES_PER_SCANLINE;
	Timings.H_Max        = Timings.H_Max_Master;
	Timings.HBlankStart  = SNES_HBLANK_START_HC;
	Timings.HBlankEnd    = SNES_HBLANK_END_HC;
	Timings.HDMAInit     = SNES_HDMA_INIT_HC;
	Timings.HDMAStart    = SNES_HDMA_START_HC;
	Timings.RenderPos    = SNES_RENDER_START_HC;
	Timings.V_Max_Master = Settings.PAL ? SNES_MAX_PAL_VCOUNTER : SNES_MAX_NTSC_VCOUNTER;
	Timings.V_Max        = Timings.V_Max_Master;
	/* From byuu: The total delay time for both the initial (H)DMA sync (to the DMA clock),
	   and the end (H)DMA sync (back to the last CPU cycle's mcycle rate (6, 8, or 12)) always takes between 12-24 mcycles.
	   Possible delays: { 12, 14, 16, 18, 20, 22, 24 }
	   XXX: Snes9x can't emulate this timing :( so let's use the average value... */
	Timings.DMACPUSync   = 18;
	/* If the CPU is halted (i.e. for DMA) while /NMI goes low, the NMI will trigger
	   after the DMA completes (even if /NMI goes high again before the DMA
	   completes). In this case, there is a 24-30 cycle delay between the end of DMA
	   and the NMI handler, time enough for an instruction or two. */
	// Wild Guns, Mighty Morphin Power Rangers - The Fighting Edition
	Timings.NMIDMADelay  = 24;

	IPPU.TotalEmulatedFrames = 0;

	//// Hack games
	ApplyROMFixes();

	//// Show ROM information
	sprintf(String, "\"%s\" [%s] %s, %s, %s, %s, SRAM:%s, ID:%s, CRC32:%08X",
		ROMName, isChecksumOK ? "checksum ok" : "bad checksum",
		MapType(), Size(), KartContents(), Settings.PAL ? "PAL" : "NTSC", StaticRAMSize(), ROMId, ROMCRC32);
	S9xMessage(S9X_INFO, S9X_ROM_INFO, String);

	// Reset system, then we're ready
	S9xReset();

    return (TRUE);
}

void CMemory::ClearSRAM (bool8 onlyNonSavedSRAM)
{
	memset(SRAM, Settings.SRAMInitialValue, 0x20000);
}

bool8 CMemory::LoadSRAM (const char *filename)
{
	FILE	*file;
	int		size, len;
	char	sramName[PATH_MAX + 1];

	strcpy(sramName, filename);

	ClearSRAM();

	size = SRAMSize ? (1 << (SRAMSize + 3)) * 128 : 0;
	if (size > 0x20000)
		size = 0x20000;

	if (size)
	{
		file = fopen(sramName, "rb");
		if (file)
		{
			len = fread((char *) SRAM, 1, 0x20000, file);
			fclose(file);
			if (len - size == 512)
				memmove(SRAM, SRAM + 512, size);

			return (TRUE);
		}

		return (FALSE);
	}

	return (TRUE);
}

bool8 CMemory::SaveSRAM (const char *filename)
{
	FILE	*file;
	int		size;
	char	sramName[PATH_MAX + 1];

	strcpy(sramName, filename);

    size = SRAMSize ? (1 << (SRAMSize + 3)) * 128 : 0;
	if (size > 0x20000)
		size = 0x20000;

	if (size)
	{
		file = fopen(sramName, "wb");
		if (file)
		{
			if (!fwrite((char *) SRAM, size, 1, file))
				printf ("Couldn't write to SRAM file.\n");
			fclose(file);

			return (TRUE);
		}
	}

	return (FALSE);
}

// initialization

void CMemory::ParseSNESHeader (uint8 *RomHeader)
{
	strncpy(ROMName, (char *) &RomHeader[0x10], ROM_NAME_LEN - 1);
	sanitize(ROMName, ROM_NAME_LEN);

	ROMSize   = RomHeader[0x27];
	SRAMSize  = RomHeader[0x28];
	ROMSpeed  = RomHeader[0x25];
	ROMType   = RomHeader[0x26];
	ROMRegion = RomHeader[0x29];

	ROMChecksum           = RomHeader[0x2E] + (RomHeader[0x2F] << 8);
	ROMComplementChecksum = RomHeader[0x2C] + (RomHeader[0x2D] << 8);

	memmove(ROMId, &RomHeader[0x02], 4);
	sanitize(ROMId, 4);

	CompanyId = -1;

	if (RomHeader[0x2A] != 0x33)
		CompanyId = ((RomHeader[0x2A] >> 4) & 0x0F) * 36 + (RomHeader[0x2A] & 0x0F);
	else
	if (isalnum(RomHeader[0x00]) && isalnum(RomHeader[0x01]))
	{
		int	l, r, l2, r2;
		l = toupper(RomHeader[0x00]);
		r = toupper(RomHeader[0x01]);
		l2 = (l > '9') ? l - '7' : l - '0';
		r2 = (r > '9') ? r - '7' : r - '0';
		CompanyId = l2 * 36 + r2;
	}
}

// memory map

uint32 CMemory::map_mirror (uint32 size, uint32 pos)
{
	// from bsnes
	if (size == 0)
		return (0);
	if (pos < size)
		return (pos);

	uint32	mask = 1 << 31;
	while (!(pos & mask))
		mask >>= 1;

	if (size <= (pos & mask))
		return (map_mirror(size, pos - mask));
	else
		return (mask + map_mirror(size - mask, pos - mask));
}

void CMemory::map_lorom (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, uint32 size)
{
	uint32	c, i, p, addr;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			addr = (c & 0x7f) * 0x8000;
			ReadMap[p] = ROM + map_mirror(size, addr) - (i & 0x8000);
		}
	}
}

void CMemory::map_hirom (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, uint32 size)
{
	uint32	c, i, p, addr;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			addr = c << 16;
			ReadMap[p] = ROM + map_mirror(size, addr);
		}
	}
}

void CMemory::map_lorom_offset (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, uint32 size, uint32 offset)
{
	uint32	c, i, p, addr;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			addr = ((c - bank_s) & 0x7f) * 0x8000;
			ReadMap[p] = ROM + offset + map_mirror(size, addr) - (i & 0x8000);
		}
	}
}

void CMemory::map_hirom_offset (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, uint32 size, uint32 offset)
{
	uint32	c, i, p, addr;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			addr = (c - bank_s) << 16;
			ReadMap[p] = ROM + offset + map_mirror(size, addr);
		}
	}
}

void CMemory::map_space (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, uint8 *data)
{
	uint32	c, i, p;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			ReadMap[p] = data;
		}
	}
}

void CMemory::map_index (uint32 bank_s, uint32 bank_e, uint32 addr_s, uint32 addr_e, int index, int type)
{
	uint32	c, i, p;

	for (c = bank_s; c <= bank_e; c++)
	{
		for (i = addr_s; i <= addr_e; i += 0x1000)
		{
			p = (c << 4) | (i >> 12);
			ReadMap[p] = (uint8 *) (pint) index;
		}
	}
}

void CMemory::map_System (void)
{
	// will be overwritten
	map_space(0x00, 0x3f, 0x0000, 0x1fff, RAM);
	map_index(0x00, 0x3f, 0x2000, 0x3fff, MAP_PPU, MAP_TYPE_I_O);
	map_index(0x00, 0x3f, 0x4000, 0x5fff, MAP_CPU, MAP_TYPE_I_O);
	map_space(0x80, 0xbf, 0x0000, 0x1fff, RAM);
	map_index(0x80, 0xbf, 0x2000, 0x3fff, MAP_PPU, MAP_TYPE_I_O);
	map_index(0x80, 0xbf, 0x4000, 0x5fff, MAP_CPU, MAP_TYPE_I_O);
}

void CMemory::map_WRAM (void)
{
	// will overwrite others
	map_space(0x7e, 0x7e, 0x0000, 0xffff, RAM);
	map_space(0x7e, 0x7e, 0x0000, 0xffff, RAM);
	map_space(0x7f, 0x7f, 0x0000, 0xffff, RAM + 0x10000);
}

void CMemory::map_LoROMSRAM (void)
{
	uint32 hi;

	if (SRAMSize == 0)
		return;

	if (ROMSize > 11 || SRAMSize > 5)
		hi = 0x7fff;
	else
		hi = 0xffff;

	map_index(0x70, 0x7d, 0x0000, hi, MAP_LOROM_SRAM, MAP_TYPE_RAM);
	map_index(0xf0, 0xff, 0x0000, hi, MAP_LOROM_SRAM, MAP_TYPE_RAM);
}

void CMemory::map_HiROMSRAM (void)
{
	map_index(0x20, 0x3f, 0x6000, 0x7fff, MAP_HIROM_SRAM, MAP_TYPE_RAM);
	map_index(0xa0, 0xbf, 0x6000, 0x7fff, MAP_HIROM_SRAM, MAP_TYPE_RAM);
}

void CMemory::map_DSP (void)
{
	switch (DSP0.maptype)
	{
		case M_DSP1_LOROM_S:
			map_index(0x20, 0x3f, 0x8000, 0xffff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0xa0, 0xbf, 0x8000, 0xffff, MAP_DSP, MAP_TYPE_I_O);
			break;

		case M_DSP1_LOROM_L:
			map_index(0x60, 0x6f, 0x0000, 0x7fff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0xe0, 0xef, 0x0000, 0x7fff, MAP_DSP, MAP_TYPE_I_O);
			break;

		case M_DSP1_HIROM:
			map_index(0x00, 0x1f, 0x6000, 0x7fff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0x80, 0x9f, 0x6000, 0x7fff, MAP_DSP, MAP_TYPE_I_O);
			break;

		case M_DSP2_LOROM:
			map_index(0x20, 0x3f, 0x6000, 0x6fff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0x20, 0x3f, 0x8000, 0xbfff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0xa0, 0xbf, 0x6000, 0x6fff, MAP_DSP, MAP_TYPE_I_O);
			map_index(0xa0, 0xbf, 0x8000, 0xbfff, MAP_DSP, MAP_TYPE_I_O);
			break;
	}
}

void CMemory::map_WriteProtectROM (void)
{
	memmove((void *) WriteMap, (void *) ReadMap, sizeof(ReadMap));

	for (int c = 0; c < MEMMAP_NUM_BLOCKS; c++)
	{
		if (WriteMap[c] >= ROM && WriteMap[c] <= (ROM + ROM_SIZE + 0x8000))
		{
			WriteMap[c] = (uint8 *) MAP_NONE;
		}
	}
}

void CMemory::Map_Initialize (void)
{
	for (int c = 0; c < MEMMAP_NUM_BLOCKS; c++)
	{
		ReadMap[c]  = (uint8 *) MAP_NONE;
		WriteMap[c] = (uint8 *) MAP_NONE;
	}
}

void CMemory::Map_LoROMMap (void)
{
	printf("Map_LoROMMap\n");
	map_System();

	map_lorom(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize);
	map_lorom(0x80, 0xbf, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0xc0, 0xff, 0x0000, 0xffff, CalculatedSize);

	if (Settings.DSP)
		map_DSP();

    map_LoROMSRAM();
	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_NoMAD1LoROMMap (void)
{
	printf("Map_NoMAD1LoROMMap\n");
	map_System();

	map_lorom(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize);
	map_lorom(0x80, 0xbf, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0xc0, 0xff, 0x0000, 0xffff, CalculatedSize);

	map_index(0x70, 0x7f, 0x0000, 0xffff, MAP_LOROM_SRAM, MAP_TYPE_RAM);
	map_index(0xf0, 0xff, 0x0000, 0xffff, MAP_LOROM_SRAM, MAP_TYPE_RAM);

	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_JumboLoROMMap (void)
{
	// XXX: Which game uses this?
	printf("Map_JumboLoROMMap\n");
	map_System();

	map_lorom_offset(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize - 0x400000, 0x400000);
	map_lorom_offset(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize - 0x600000, 0x600000);
	map_lorom_offset(0x80, 0xbf, 0x8000, 0xffff, 0x400000, 0);
	map_lorom_offset(0xc0, 0xff, 0x0000, 0xffff, 0x400000, 0x200000);

	map_LoROMSRAM();
	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_ROM24MBSLoROMMap (void)
{
	// PCB: BSC-1A5M-01, BSC-1A7M-10
	printf("Map_ROM24MBSLoROMMap\n");
	map_System();

	map_lorom_offset(0x00, 0x1f, 0x8000, 0xffff, 0x100000, 0);
	map_lorom_offset(0x20, 0x3f, 0x8000, 0xffff, 0x100000, 0x100000);
	map_lorom_offset(0x80, 0x9f, 0x8000, 0xffff, 0x100000, 0x200000);
	map_lorom_offset(0xa0, 0xbf, 0x8000, 0xffff, 0x100000, 0x100000);

	map_LoROMSRAM();
	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_SRAM512KLoROMMap (void)
{
	printf("Map_SRAM512KLoROMMap\n");
	map_System();

	map_lorom(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize);
	map_lorom(0x80, 0xbf, 0x8000, 0xffff, CalculatedSize);
	map_lorom(0xc0, 0xff, 0x0000, 0xffff, CalculatedSize);

	map_space(0x70, 0x70, 0x0000, 0xffff, SRAM);
	map_space(0x71, 0x71, 0x0000, 0xffff, SRAM + 0x8000);
	map_space(0x72, 0x72, 0x0000, 0xffff, SRAM + 0x10000);
	map_space(0x73, 0x73, 0x0000, 0xffff, SRAM + 0x18000);

	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_HiROMMap (void)
{
	printf("Map_HiROMMap\n");
	map_System();

	map_hirom(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize);
	map_hirom(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize);
	map_hirom(0x80, 0xbf, 0x8000, 0xffff, CalculatedSize);
	map_hirom(0xc0, 0xff, 0x0000, 0xffff, CalculatedSize);

	if (Settings.DSP)
		map_DSP();

	map_HiROMSRAM();
	map_WRAM();

	map_WriteProtectROM();
}

void CMemory::Map_ExtendedHiROMMap (void)
{
	printf("Map_ExtendedHiROMMap\n");
	map_System();

	map_hirom_offset(0x00, 0x3f, 0x8000, 0xffff, CalculatedSize - 0x400000, 0x400000);
	map_hirom_offset(0x40, 0x7f, 0x0000, 0xffff, CalculatedSize - 0x400000, 0x400000);
	map_hirom_offset(0x80, 0xbf, 0x8000, 0xffff, 0x400000, 0);
	map_hirom_offset(0xc0, 0xff, 0x0000, 0xffff, 0x400000, 0);

	map_HiROMSRAM();
	map_WRAM();

	map_WriteProtectROM();
}

// checksum

uint16 CMemory::checksum_calc_sum (uint8 *data, uint32 length)
{
	uint16	sum = 0;

	for (uint32 i = 0; i < length; i++)
		sum += data[i];

	return (sum);
}

uint16 CMemory::checksum_mirror_sum (uint8 *start, uint32 &length, uint32 mask)
{
	// from NSRT
	while (!(length & mask) && mask)
		mask >>= 1;

	uint16	part1 = checksum_calc_sum(start, mask);
	uint16	part2 = 0;

	uint32	next_length = length - mask;
	if (next_length)
	{
		part2 = checksum_mirror_sum(start + mask, next_length, mask >> 1);

		while (next_length < mask)
		{
			next_length += next_length;
			part2 += part2;
		}

		length = mask + mask;
	}

	return (part1 + part2);
}

void CMemory::Checksum_Calculate (void)
{
	// from NSRT
	uint16	sum = 0;

	if (CalculatedSize & 0x7fff)
		sum = checksum_calc_sum(ROM, CalculatedSize);
	else
	{
		uint32	length = CalculatedSize;
		sum = checksum_mirror_sum(ROM, length);
	}

	CalculatedChecksum = sum;
}

// information

const char * CMemory::MapType (void)
{
	return (HiROM ? ((ExtendedFormat != NOPE) ? "ExHiROM": "HiROM") : "LoROM");
}

const char * CMemory::StaticRAMSize (void)
{
	static char	str[20];

	if (SRAMSize > 16)
		strcpy(str, "Corrupt");
	else
		sprintf(str, "%dKbits", 8 * (SRAMMask + 1) / 1024);

	return (str);
}

const char * CMemory::Size (void)
{
	static char	str[20];

	if (ROMSize < 7 || ROMSize - 7 > 23)
		strcpy(str, "Corrupt");
	else
		sprintf(str, "%dMbits", 1 << (ROMSize - 7));

	return (str);
}

const char * CMemory::Revision (void)
{
	static char	str[20];

	sprintf(str, "1.%d", HiROM ? ((ExtendedFormat != NOPE) ? ROM[0x40ffdb] : ROM[0xffdb]) : ROM[0x7fdb]);

	return (str);
}

const char * CMemory::KartContents (void)
{
	static char	str[64];
	const char	*contents[3] = { "ROM", "ROM+RAM", "ROM+RAM+BAT" };

	strcpy(str, contents[(ROMType & 0xf) % 3]);

	if (Settings.DSP == 1)
		strcat(str, "+DSP-1");
	else if (Settings.DSP == 2)
		strcat(str, "+DSP-2");

	return (str);
}

const char * CMemory::Country (void)
{
	switch (ROMRegion)
	{
		case 0:		return("Japan");
		case 1:		return("USA and Canada");
		case 2:		return("Oceania, Europe and Asia");
		case 3:		return("Sweden");
		case 4:		return("Finland");
		case 5:		return("Denmark");
		case 6:		return("France");
		case 7:		return("Holland");
		case 8:		return("Spain");
		case 9:		return("Germany, Austria and Switzerland");
		case 10:	return("Italy");
		case 11:	return("Hong Kong and China");
		case 12:	return("Indonesia");
		case 13:	return("South Korea");
		default:	return("Unknown");
	}
}

const char * CMemory::PublishingCompany (void)
{
	return ("Unknown");
}

void CMemory::MakeRomInfoText (char *romtext)
{
	char	temp[256];

	romtext[0] = 0;

	sprintf(temp,   "            Cart Name: %s", ROMName);
	strcat(romtext, temp);
	sprintf(temp, "\n            Game Code: %s", ROMId);
	strcat(romtext, temp);
	sprintf(temp, "\n             Contents: %s", KartContents());
	strcat(romtext, temp);
	sprintf(temp, "\n                  Map: %s", MapType());
	strcat(romtext, temp);
	sprintf(temp, "\n                Speed: 0x%02X (%s)", ROMSpeed, (ROMSpeed & 0x10) ? "FastROM" : "SlowROM");
	strcat(romtext, temp);
	sprintf(temp, "\n                 Type: 0x%02X", ROMType);
	strcat(romtext, temp);
	sprintf(temp, "\n    Size (calculated): %dMbits", CalculatedSize / 0x20000);
	strcat(romtext, temp);
	sprintf(temp, "\n        Size (header): %s", Size());
	strcat(romtext, temp);
	sprintf(temp, "\n            SRAM size: %s", StaticRAMSize());
	strcat(romtext, temp);
	sprintf(temp, "\nChecksum (calculated): 0x%04X", CalculatedChecksum);
	strcat(romtext, temp);
	sprintf(temp, "\n    Checksum (header): 0x%04X", ROMChecksum);
	strcat(romtext, temp);
	sprintf(temp, "\n  Complement (header): 0x%04X", ROMComplementChecksum);
	strcat(romtext, temp);
	sprintf(temp, "\n         Video Output: %s", (ROMRegion > 12 || ROMRegion < 2) ? "NTSC 60Hz" : "PAL 50Hz");
	strcat(romtext, temp);
	sprintf(temp, "\n             Revision: %s", Revision());
	strcat(romtext, temp);
	sprintf(temp, "\n             Licensee: %s", PublishingCompany());
	strcat(romtext, temp);
	sprintf(temp, "\n               Region: %s", Country());
	strcat(romtext, temp);
	sprintf(temp, "\n                CRC32: 0x%08X", ROMCRC32);
	strcat(romtext, temp);
}

// hack

bool8 CMemory::match_na (const char *str)
{
	return (strcmp(ROMName, str) == 0);
}

bool8 CMemory::match_nn (const char *str)
{
	return (strncmp(ROMName, str, strlen(str)) == 0);
}

void CMemory::ApplyROMFixes (void)
{
	Settings.BlockInvalidVRAMAccess = true;
	Settings.UniracersHack = FALSE;
	Settings.SRAMInitialValue = 0x60;

	//// Warnings

	// Reject strange hacked games
	if ((ROMCRC32 == 0x6810aa95) ||
		(ROMCRC32 == 0x340f23e5) ||
		(ROMCRC32 == 0x77fd806a) ||
		(match_nn("HIGHWAY BATTLE 2")) ||
		(match_na("FX SKIING NINTENDO 96") && (ROM[0x7fda] == 0)) ||
		(match_nn("HONKAKUHA IGO GOSEI")   && (ROM[0xffd5] != 0x31)))
	{
		Settings.DisplayColor = BUILD_PIXEL(31, 0, 0);
		SET_UI_COLOR(255, 0, 0);
	}

	//// APU timing hacks :(

	Timings.APUSpeedup = 0;

	if (!Settings.DisableGameSpecificHacks)
	{
		if (match_na("CIRCUIT USA"))
			Timings.APUSpeedup = 3;
	}

	S9xAPUTimingSetSpeedup(Timings.APUSpeedup);

	//// Other timing hacks :(

	Timings.HDMAStart   = SNES_HDMA_START_HC + Settings.HDMATimingHack - 100;
	Timings.HBlankStart = SNES_HBLANK_START_HC + Timings.HDMAStart - SNES_HDMA_START_HC;
	Timings.IRQTriggerCycles = 14;

	if (!Settings.DisableGameSpecificHacks)
	{
		// The delay to sync CPU and DMA which Snes9x cannot emulate.
		// Some games need really severe delay timing...
		if (match_na("BATTLE GRANDPRIX")) // Battle Grandprix
		{
			Timings.DMACPUSync = 20;
			printf("DMA sync: %d\n", Timings.DMACPUSync);
		}
		else if (match_na("KORYU NO MIMI ENG")) // Koryu no Mimi translation by rpgone)
		{
			// An infinite loop reads $4210 and checks NMI flag. This only works if LDA instruction executes before the NMI triggers,
			// which doesn't work very well with s9x's default DMA timing.
			Timings.DMACPUSync = 20;
			printf("DMA sync: %d\n", Timings.DMACPUSync);
		}
	}

	//// SRAM initial value

	if (!Settings.DisableGameSpecificHacks)
	{
		if (match_na("HITOMI3"))
		{
			SRAMSize = 1;
			SRAMMask = ((1 << (SRAMSize + 3)) * 128) - 1;
		}

		// SRAM value fixes
		if (match_na("SUPER DRIFT OUT")      || // Super Drift Out
			match_na("SATAN IS OUR FATHER!") ||
			match_na("goemon 4"))               // Ganbare Goemon Kirakira Douchuu
			Settings.SRAMInitialValue = 0x00;

		// Additional game fixes by sanmaiwashi ...
		// XXX: unnecessary?
		if (match_na("SFX \xC5\xB2\xC4\xB6\xDE\xDD\xC0\xDE\xD1\xD3\xC9\xB6\xDE\xC0\xD8 1")) // SD Gundam Gaiden - Knight Gundam Monogatari
			Settings.SRAMInitialValue = 0x6b;

		// others: BS and ST-01x games are 0x00.
	}

	//// OAM hacks :(

	if (!Settings.DisableGameSpecificHacks)
	{
		// OAM hacks because we don't fully understand the behavior of the SNES.
		// Totally wacky display in 2P mode...
		// seems to need a disproven behavior, so we're definitely overlooking some other bug?
		if (match_nn("UNIRACERS")) // Uniracers
		{
			Settings.UniracersHack = TRUE;
			printf("Applied Uniracers hack.\n");
		}
	}
}

// BPS % UPS % IPS
#if 0

static long ReadInt (Stream *r, unsigned nbytes)
{
	long	v = 0;

	while (nbytes--)
	{
		int	c = r->get_char();
		if (c == EOF)
			return (-1);
		v = (v << 8) | (c & 0xFF);
	}

	return (v);
}

static bool8 ReadIPSPatch (Stream *r, long offset, int32 &rom_size)
{
	const int32	IPS_EOF = 0x00454F46l;
	int32		ofs;
	char		fname[6];

	fname[5] = 0;
	for (int i = 0; i < 5; i++)
	{
		int	c = r->get_char();
		if (c == EOF)
			return (0);
		fname[i] = (char) c;
	}

	if (strncmp(fname, "PATCH", 5))
		return (0);

	for (;;)
	{
		long	len, rlen;
		int		rchar;

		ofs = ReadInt(r, 3);
		if (ofs == -1)
			return (0);

		if (ofs == IPS_EOF)
			break;

		ofs -= offset;

		len = ReadInt(r, 2);
		if (len == -1)
			return (0);

		if (len)
		{
			if (ofs + len > CMemory::ROM_MAX_SIZE)
				return (0);

			while (len--)
			{
				rchar = r->get_char();
				if (rchar == EOF)
					return (0);
				Memory.ROM[ofs++] = (uint8) rchar;
			}

			if (ofs > rom_size)
				rom_size = ofs;
		}
		else
		{
			rlen = ReadInt(r, 2);
			if (rlen == -1)
				return (0);

			rchar = r->get_char();
			if (rchar == EOF)
				return (0);

			if (ofs + rlen > CMemory::ROM_MAX_SIZE)
				return (0);

			while (rlen--)
				Memory.ROM[ofs++] = (uint8) rchar;

			if (ofs > rom_size)
				rom_size = ofs;
		}
	}

	ofs = ReadInt(r, 3);
	if (ofs != -1 && ofs - offset < rom_size)
		rom_size = ofs - offset;

	Settings.IsPatched = 1;
	return (1);
}

void CMemory::CheckForAnyPatch (const char *rom_filename, bool8 header, int32 &rom_size)
{
	Settings.IsPatched = false;

	if (Settings.NoPatch)
		return;

	FSTREAM		patch_file  = NULL;
	uint32		i;
	long		offset = header ? 512 : 0;
	int			ret;
	bool		flag;
	char		dir[_MAX_DIR + 1], drive[_MAX_DRIVE + 1], name[_MAX_FNAME + 1], ext[_MAX_EXT + 1], ips[_MAX_EXT + 3], fname[PATH_MAX + 1];
	const char	*n;

	_splitpath(rom_filename, drive, dir, name, ext);

	// IPS

	_makepath(fname, drive, dir, name, "ips");

	if ((patch_file = OPEN_FSTREAM(fname, "rb")) != NULL)
	{
		printf("Using IPS patch %s", fname);

        Stream *s = new fStream(patch_file);
		ret = ReadIPSPatch(s, offset, rom_size);
        s->closeStream();

		if (ret)
		{
			printf("!\n");
			return;
		}
		else
			printf(" failed!\n");
	}

	if (_MAX_EXT > 6)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, 8, "%03d.ips", i);
			_makepath(fname, drive, dir, name, ips);

			if (!(patch_file = OPEN_FSTREAM(fname, "rb")))
				break;

			printf("Using IPS patch %s", fname);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i < 1000);

		if (flag)
			return;
	}

	if (_MAX_EXT > 3)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, _MAX_EXT + 2, "ips%d", i);
			if (strlen(ips) > _MAX_EXT)
				break;
			_makepath(fname, drive, dir, name, ips);

			if (!(patch_file = OPEN_FSTREAM(fname, "rb")))
				break;

			printf("Using IPS patch %s", fname);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i != 0);

		if (flag)
			return;
	}

	if (_MAX_EXT > 2)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, 4, "ip%d", i);
			_makepath(fname, drive, dir, name, ips);

			if (!(patch_file = OPEN_FSTREAM(fname, "rb")))
				break;

			printf("Using IPS patch %s", fname);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i < 10);

		if (flag)
			return;
	}

	n = S9xGetFilename(".ips", PATCH_DIR);

	if ((patch_file = OPEN_FSTREAM(n, "rb")) != NULL)
	{
		printf("Using IPS patch %s", n);

        Stream *s = new fStream(patch_file);
		ret = ReadIPSPatch(s, offset, rom_size);
        s->closeStream();

		if (ret)
		{
			printf("!\n");
			return;
		}
		else
			printf(" failed!\n");
	}

	if (_MAX_EXT > 6)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, 9, ".%03d.ips", i);
			n = S9xGetFilename(ips, PATCH_DIR);

			if (!(patch_file = OPEN_FSTREAM(n, "rb")))
				break;

			printf("Using IPS patch %s", n);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i < 1000);

		if (flag)
			return;
	}

	if (_MAX_EXT > 3)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, _MAX_EXT + 3, ".ips%d", i);
			if (strlen(ips) > _MAX_EXT + 1)
				break;
			n = S9xGetFilename(ips, PATCH_DIR);

			if (!(patch_file = OPEN_FSTREAM(n, "rb")))
				break;

			printf("Using IPS patch %s", n);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i != 0);

		if (flag)
			return;
	}

	if (_MAX_EXT > 2)
	{
		i = 0;
		flag = false;

		do
		{
			snprintf(ips, 5, ".ip%d", i);
			n = S9xGetFilename(ips, PATCH_DIR);

			if (!(patch_file = OPEN_FSTREAM(n, "rb")))
				break;

			printf("Using IPS patch %s", n);

            Stream *s = new fStream(patch_file);
			ret = ReadIPSPatch(s, offset, rom_size);
            s->closeStream();

			if (ret)
			{
				printf("!\n");
				flag = true;
			}
			else
			{
				printf(" failed!\n");
				break;
			}
		} while (++i < 10);

		if (flag)
			return;
	}
}

#endif