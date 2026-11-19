/*
 * ARMCC -> GCC 兼容宏
 *
 * Keil/ARMCC 把 __weak / __packed / __align 当作编译器关键字，
 * GCC 下没有这些名字，而工程里的 HAL-USB、USB host 库直接用了它们。
 * 用 -include gcc/gcc_compat.h 全局引入，避免逐个文件改。
 */

#ifndef GCC_COMPAT_H
#define GCC_COMPAT_H

#ifndef __weak
#define __weak          __attribute__((weak))
#endif

#ifndef __packed
#define __packed        __attribute__((packed))
#endif

#ifndef __align
#define __align(n)      __attribute__((aligned(n)))
#endif

#ifndef __forceinline
#define __forceinline   __attribute__((always_inline)) inline
#endif

#ifndef __nop
#define __nop()         __asm__ volatile ( "nop" )
#endif

#ifndef __UNUSED
#define __UNUSED        __attribute__((unused))
#endif

#endif /* GCC_COMPAT_H */
