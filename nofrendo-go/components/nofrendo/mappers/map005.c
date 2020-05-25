/*
** Nofrendo (c) 1998-2000 Matthew Conte (matt@conte.com)
**
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of version 2 of the GNU Library General
** Public License as published by the Free Software Foundation.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** Library General Public License for more details.  To obtain a
** copy of the GNU Library General Public License, write to the Free
** Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**
** Any permitted reproduction of these routines, in whole or in part,
** must bear this legend.
**
**
** map005.c
**
** Mapper 5 (MMC5) interface
** Implementation by ducalex
**
*/

#include <string.h>
#include <nofrendo.h>
#include <nes_mmc.h>
#include <nes.h>

static uint8 prg_mode;
static uint8 chr_mode;
static uint8 nametable_mapping;
static uint8 multiplication[2];

static uint16 prg_banks[4];
static uint16 chr_banks[12];
static uint16 chr_upper_bits;

static nes_t *nes;

static int16 split_tile, split_tile_number, split_region;
static int16 scanline;

#define IN_FRAME    0x40
#define IRQ_PENDING 0x80
#define SPLIT_ENABLED 0x80
#define SPLIT_RIGHT 0x40
#define SPLIT_TILE 0x1F

static struct
{
   uint8 *data;
   uint8 mode;
} exram;

static struct
{
   uint8 *data;
   uint8 mode;
   uint8 protect1;
   uint8 protect2;
} prgram;

static struct
{
   uint8 enabled;
   uint8 rightside;
   uint8 delimiter;
   uint8 scroll;
   uint8 bank;
} vert_split;

static struct
{
   int16 scanline;
   uint8 enabled;
   uint8 status;
} irq;

static struct
{
   uint8 color;
   uint8 tile;
} fill_mode;


INLINE uint8 *get_nametable(int n)
{
   return nes->ppu->nametab + (0x400 * n);
}

static void prg_setbank(int size, uint32 address, int bank)
{
   bool rom = (bank & 0x80);

   bank &= 0x7F;

   if (size == 32) bank >>= 2;
   if (size == 16) bank >>= 1;

   if (rom)
   {
      mmc_bankrom(size, address, bank);
   }
   else
   {
      mmc_bankwram(size, address, bank);
   }
}

static void prg_update()
{
   switch (prg_mode)
   {
   case 0: /* 1x32K */
      prg_setbank(32, 0x8000, prg_banks[0]);
      break;

   case 1: /* 2x16K */
      prg_setbank(16, 0x8000, prg_banks[1]);
      prg_setbank(16, 0xC000, prg_banks[3]);
      break;

   case 2: /* 1x16K+2x8K */
      prg_setbank(16, 0x8000, prg_banks[1]);
      prg_setbank(8, 0xC000, prg_banks[2]);
      prg_setbank(8, 0xE000, prg_banks[3]);
      break;

   case 3: /* 4x8K */
      prg_setbank(8, 0x8000, prg_banks[0]);
      prg_setbank(8, 0xA000, prg_banks[1]);
      prg_setbank(8, 0xC000, prg_banks[2]);
      prg_setbank(8, 0xE000, prg_banks[3]);
      break;
   }
}

static void chr_update()
{
   bool large_spr = nes->ppu->obj_height == 16;

   switch (chr_mode)
   {
   case 0: /* 8K */
      break;

   case 1: /* 4K */
      break;

   case 2: /* 2K */
      break;

   case 3: /* 1K */
      mmc_bankvrom(1, 0 * 0x400, chr_banks[large_spr ? 8  : 0]);
      mmc_bankvrom(1, 1 * 0x400, chr_banks[large_spr ? 9  : 1]);
      mmc_bankvrom(1, 2 * 0x400, chr_banks[large_spr ? 10 : 2]);
      mmc_bankvrom(1, 3 * 0x400, chr_banks[large_spr ? 11 : 3]);
      mmc_bankvrom(1, 4 * 0x400, chr_banks[large_spr ? 8  : 4]);
      mmc_bankvrom(1, 5 * 0x400, chr_banks[large_spr ? 9  : 5]);
      mmc_bankvrom(1, 6 * 0x400, chr_banks[large_spr ? 10 : 6]);
      mmc_bankvrom(1, 7 * 0x400, chr_banks[large_spr ? 11 : 7]);
      break;
   }
}

INLINE void chr_switch(int reg, int value)
{
   chr_banks[reg] = value | chr_upper_bits;
   // chr_update();

   if (reg >= 8) {
      mmc_bankvrom(1, 0x1000 + (reg - 8) * 0x400, chr_banks[reg]);
   } else {
      mmc_bankvrom(1, reg * 0x400, chr_banks[reg]);
   }
}

INLINE void nametable_update(uint8 value)
{
   nametable_mapping = value;
   ppu_mirror(value & 3, (value >> 2) & 3, (value >> 4) & 3, value >> 6);
}

INLINE void nametable_fill()
{
   memset(get_nametable(3), fill_mode.tile, 32 * 30); // 32 tiles per row, 30 rows
   memset(get_nametable(3) + 32 * 30, (fill_mode.color | fill_mode.color << 2 | fill_mode.color << 4 | fill_mode.color << 6), 64);
}

static void map5_hblank(int _scanline)
{
   scanline = _scanline;

   if (scanline >= 241)
   {
      irq.status &= ~IN_FRAME; // (IN_FRAME|IRQ_PENDING)
   }
   else
   {
      irq.status |= IN_FRAME;

      if (scanline == irq.scanline)
      {
         if (irq.enabled) nes6502_irq();
         irq.status |= IRQ_PENDING;
      }
   }
}

static void map5_write(uint32 address, uint8 value)
{
   // MESSAGE_INFO("MMC5 write: $%02X to $%04X\n", value, address);

   switch (address)
   {
   case 0x5100:
      /* PRG mode */
      prg_mode = value & 3;
      prg_update();
      break;

   case 0x5101:
      /* CHR mode */
      chr_mode = value & 3;
      chr_update();
      break;

   case 0x5102:
      /* PRG RAM Protect 1 */
      /* Allow write = 0b10 */
      prgram.protect1 = value & 3;
      prg_update();
      break;

   case 0x5103:
      /* PRG RAM Protect 2 */
      /* Allow write = 0b01 */
      prgram.protect2 = value & 3;
      prg_update();
      break;

   case 0x5104:
      /* Extended RAM mode */
      exram.mode = value & 3;
      break;

   case 0x5105:
      /* Nametable mapping */
      nametable_update(value);
      break;

   case 0x5106:
      /* Fill-mode tile */
      fill_mode.tile = value;
      nametable_fill();
      break;

   case 0x5107:
      /* Fill-mode color */
      fill_mode.color = value;
      nametable_fill();
      break;

   case 0x5113:
      /* PRG Bankswitching */
      break;

   case 0x5114:
   case 0x5115:
   case 0x5116:
   case 0x5117:
      /* PRG Bankswitching */
      prg_banks[address - 0x5114] = value;
      prg_update();
      break;

   case 0x5120:
   case 0x5121:
   case 0x5122:
   case 0x5123:
   case 0x5124:
   case 0x5125:
   case 0x5126:
   case 0x5127:
   case 0x5128:
   case 0x5129:
   case 0x512A:
   case 0x512B:
      /* CHR Bankswitching */
      chr_switch(address - 0x5120, value);
      break;

   case 0x5130:
      /* Upper CHR Bank bits */
      chr_upper_bits = (uint16)(value & 3) << 8;
      break;

   case 0x5200:
      /* Vertical Split Mode */
      vert_split.enabled   = (value >> 7) & 1;
      vert_split.rightside = (value >> 6) & 1;
      vert_split.delimiter = (value & 0x1F);
      break;

   case 0x5201:
      /* Vertical Split Scroll */
      vert_split.scroll = value;
      break;

   case 0x5202:
      /* Vertical Split Bank */
      vert_split.bank = value % (nes->mmc->chr_banks * 2);
      break;

   case 0x5203:
      /* IRQ Scanline Compare Value */
      irq.scanline = value;
      break;

   case 0x5204:
      /* Scanline IRQ Status */
      irq.enabled = value >> 7;
      break;

   case 0x5205:
   case 0x5206:
      /* Unsigned u8*u8 to u16 multiplication */
      multiplication[address & 1] = value;
      break;

   default:
      MESSAGE_DEBUG("Invalid MMC5 write: $%02X to $%04X\n", value, address);
      break;
   }
}

static uint8 map5_read(uint32 address)
{
   switch(address)
   {
   case 0x5204:
      /* Scanline IRQ Status */
      {
         uint8 x = irq.status;
         irq.status &= ~IRQ_PENDING;
         return x;
      }

   case 0x5205:
      /* Unsigned 8x8 to 16 Multiplier (low byte) */
      return (multiplication[0] * multiplication[1]) & 0xFF;

   case 0x5206:
      /* Unsigned 8x8 to 16 Multiplier (high byte) */
      return (multiplication[0] * multiplication[1]) >> 8;

   default:
      MESSAGE_DEBUG("Invalid MMC5 read: $%04X", address);
      return 0xFF;
   }
}

static uint8 map5_vram_read(uint32 address, uint8 value)
{
   bool is_nt_read = false, is_nt_attr_read = false;

   if (address >= 0x2000 && address <= 0x2FFF)
   {
      is_nt_read = (address & 0x3FF) < (30 * 32);
      is_nt_attr_read = !is_nt_read;
   }


   if (exram.mode <= 1 && scanline < 240)
   {
      if (vert_split.enabled)
      {
         short scroll = (vert_split.scroll + scanline) % 240;
      }
   }

   return value;
}

static uint8 map5_exram_read(uint32 address)
{
   return exram.data[address - 0x5C00];
}

static void map5_exram_write(uint32 address, uint8 value)
{
   if (exram.mode != 3)
   {
      exram.data[address - 0x5C00] = value;
   }
}

static void map5_init(void)
{
   nes = nes_getptr();

   // if (nes->rominfo->sram)
   // {
   //    free(nes->rominfo->sram);
   // }

   // nes->rominfo->sram = malloc(0x10000);
   // mmc_bankwram(8, 0x6000, 0);

   exram.data = get_nametable(2);

   irq.scanline = irq.enabled = 0;
   irq.status = 0;

   multiplication[0] = 0xFF;
   multiplication[1] = 0xFF;

   fill_mode.color = 0xFF;
   fill_mode.tile = 0xFF;

   scanline = 241;

   prg_mode = 3;
   chr_mode = 3;

   chr_upper_bits = 0;

   for (int i = 0; i < 4; i++)
      prg_banks[i] = MMC_LASTBANK;

   for (int i = 0; i < 8; i++)
      chr_banks[i] = i;

   prg_update();
   chr_update();
   nametable_fill();
   nametable_update(0);

   ppu_setvreadfunc(map5_vram_read);
}

static void map5_getstate(void *state)
{
   //
}

static void map5_setstate(void *state)
{
   //
}

static mem_write_handler_t map5_memwrite[] =
{
   // { 0x5000, 0x50FF, map5_write }, // sound
   { 0x5100, 0x5BFF, map5_write },
   { 0x5C00, 0x5FFF, map5_exram_write },
   { 0x8000, 0xFFFF, map5_write },
   LAST_MEMORY_HANDLER
};

static mem_read_handler_t map5_memread[] =
{
   { 0x5100, 0x5BFF, map5_read },
   { 0x5C00, 0x5FFF, map5_exram_read },
   LAST_MEMORY_HANDLER
};

mapintf_t map5_intf =
{
   5,                /* mapper number */
   "MMC5",           /* mapper name */
   map5_init,        /* init routine */
   NULL,             /* vblank callback */
   map5_hblank,      /* hblank callback */
   map5_getstate,    /* get state (snss) */
   map5_setstate,    /* set state (snss) */
   map5_memread,     /* memory read structure */
   map5_memwrite,    /* memory write structure */
   NULL              /* external sound device */
};