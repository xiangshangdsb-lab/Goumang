#include "stmflash.h"

/*
 * FLASH_CR / FLASH_SR bit definitions itself come from stm32f4xx.h (pulled in
 * through sys.h) and stm32f4xx_flash.h (KEY1/KEY2).  Only the field positions
 * that CMSIS does not name explicitly are defined here.
 */
#define C30D_FLASH_CR_SNB_POS     3u
#define C30D_FLASH_CR_PSIZE_POS   8u
#define C30D_FLASH_SR_OPERR       ((uint32_t)0x00000002)
#define C30D_FLASH_SR_CLEAR       (FLASH_SR_EOP | C30D_FLASH_SR_OPERR |      \
                                   FLASH_SR_WRPERR | FLASH_SR_PGAERR |        \
                                   FLASH_SR_PGPERR | FLASH_SR_PGSERR)
#define C30D_FLASH_SR_ERRORS      (C30D_FLASH_SR_OPERR | FLASH_SR_WRPERR |   \
                                   FLASH_SR_PGAERR | FLASH_SR_PGPERR |       \
                                   FLASH_SR_PGSERR)
#define C30D_FLASH_WAIT_LOOPS     0x00200000u

static u8 flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u)
    {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        if ((FLASH->CR & FLASH_CR_LOCK) != 0u)
        {
            return 1u;
        }
    }
    return 0u;
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static u8 flash_wait(void)
{
    u32 timeout = C30D_FLASH_WAIT_LOOPS;

    while ((FLASH->SR & FLASH_SR_BSY) != 0u)
    {
        if (--timeout == 0u)
        {
            return 1u;
        }
    }
    return 0u;
}

static u8 flash_sector_of(u32 addr)
{
    u32 offset = addr - STM32_FLASH_BASE;

    if (offset < 0x00010000u)
    {
        return (u8)(offset / 0x4000u);                    /* sectors 0..3, 16 KB */
    }
    if (offset < 0x00020000u)
    {
        return 4u;                                        /* sector 4, 64 KB */
    }
    return (u8)(5u + (offset - 0x00020000u) / 0x00020000u);  /* sectors 5..11 */
}

static u8 flash_address_ok(u32 addr)
{
    return (addr >= STM32_FLASH_BASE &&
            addr < (STM32_FLASH_BASE + STM32_FLASH_SIZE_KB * 1024u)) ? 1u : 0u;
}

/* Erase the whole sector that contains addr. Returns 0 on success. */
u8 STMFLASH_EraseSector(u32 addr)
{
    u8 sector;
    u8 result;

    if (flash_address_ok(addr) == 0u)
    {
        return 1u;
    }
    if (flash_unlock() != 0u)
    {
        return 2u;
    }
    sector = flash_sector_of(addr);
    FLASH->SR = C30D_FLASH_SR_CLEAR;
    FLASH->CR &= ~(0x0Fu << C30D_FLASH_CR_SNB_POS);
    FLASH->CR |= ((u32)sector << C30D_FLASH_CR_SNB_POS) |
                 FLASH_CR_SER | FLASH_CR_STRT;
    result = flash_wait();
    FLASH->CR &= ~(FLASH_CR_SER | (0x0Fu << C30D_FLASH_CR_SNB_POS));
    if ((FLASH->SR & C30D_FLASH_SR_ERRORS) != 0u)
    {
        result = 3u;
    }
    FLASH->SR = C30D_FLASH_SR_CLEAR;
    flash_lock();

    /* Sector data may have been cached while scanning the position log. */
    u32 acr = FLASH->ACR;
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR = acr;
    return result;
}

/* Program data bytes at addr. The target halfwords must already be erased. */
u8 STMFLASH_Program(u32 addr, const u8 *data, u16 length)
{
    u16 index;
    u8 result = 0u;

    if (data == (const u8 *)0 || (addr & 1u) != 0u ||
        flash_address_ok(addr) == 0u)
    {
        return 1u;
    }
    if (flash_unlock() != 0u)
    {
        return 2u;
    }
    FLASH->SR = C30D_FLASH_SR_CLEAR;
    FLASH->CR &= ~(3u << C30D_FLASH_CR_PSIZE_POS);
    FLASH->CR |= (1u << C30D_FLASH_CR_PSIZE_POS);   /* PSIZE = x16 halfword */
    FLASH->CR |= FLASH_CR_PG;

    for (index = 0u; index < length; index += 2u)
    {
        u16 halfword = (u16)data[index];

        if ((u16)(index + 1u) < length)
        {
            halfword |= (u16)((u16)data[index + 1u] << 8);
        }
        else
        {
            halfword |= 0xFF00u;
        }
        *(volatile u16 *)(addr + index) = halfword;
        if (flash_wait() != 0u || (FLASH->SR & C30D_FLASH_SR_ERRORS) != 0u)
        {
            result = 3u;
            break;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->SR = C30D_FLASH_SR_CLEAR;
    flash_lock();
    return result;
}

void STMFLASH_Read(u32 addr, u8 *out, u16 length)
{
    u16 index;

    if (out == (u8 *)0)
    {
        return;
    }
    for (index = 0u; index < length; ++index)
    {
        out[index] = *(const volatile u8 *)(addr + index);
    }
}
