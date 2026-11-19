#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "sys.h"

/*
 * STM32F407ZG internal flash helper (1 MB, single bank).
 *
 * The original WHEELTEC stmflash.c was written for the STM32F103 page model
 * and never compiled for this target: the STM32F4 has no FLASH->AR register
 * and erases 16 KB / 64 KB / 128 KB sectors selected by SNB.  This version
 * keeps the project's address convention (configuration record located at
 * 0x0800E000) and implements sector erase plus halfword programming for the
 * F4 controller.
 *
 * STM32F407ZG sector map:
 *   sector 0..3 : 0x08000000 .. 0x0800FFFF    16 KB each
 *   sector 4    : 0x08010000 .. 0x0801FFFF    64 KB
 *   sector 5..11: 0x08020000 .. 0x080FFFFF   128 KB each
 *
 * The C30D image ends well below 0x08008000, so 16 KB sector 3
 * (0x0800C000..0x0800FFFF) is reserved for the gimbal/parameter record and
 * is erased as a whole.  Keep the firmware image below 0x0800C000.
 */
#define STM32_FLASH_BASE            0x08000000u
#define STM32_FLASH_SIZE_KB         1024u

#define STMFLASH_NVM_ADDR           0x0800E000u
#define STMFLASH_NVM_SECTOR_ADDR    0x0800C000u
#define STMFLASH_NVM_SECTOR_SIZE    0x4000u

u8 STMFLASH_EraseSector(u32 addr);
u8 STMFLASH_Program(u32 addr, const u8 *data, u16 length);
void STMFLASH_Read(u32 addr, u8 *out, u16 length);

#endif
