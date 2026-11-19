/*
 * newlib 适配层（GCC 专用）
 *
 * Keil 版本靠 fputc + __use_no_semihosting 把 printf 引到 USART2；
 * arm-none-eabi + newlib 走的是 _write，所以这里补一份等价实现，
 * 另外补上 _sbrk 等最小系统调用，避免链到 semihosting。
 */

#include <errno.h>
#include <sys/stat.h>
#include "stm32f4xx.h"

/* 由链接脚本提供 */
extern char end;      /* .bss 之后的空闲 RAM 起点 */
extern char _estack;  /* RAM 顶端，即初始 MSP */

/*
 * 用 -nostartfiles 链接时不会带入 crti.o，而 newlib 的 __libc_init_array()
 * 会调用 _init/_fini，这里补两个空实现。工程里没有 C++ 全局构造函数，
 * 这两个函数保持为空即可。
 */
void _init(void) { }
void _fini(void) { }

/* printf / putchar 输出：USART2，与 Keil 版 fputc 完全一致 */
int _write(int file, char *ptr, int len)
{
	int i;

	(void)file;

	for (i = 0; i < len; i++)
	{
		while ((USART2->SR & 0x40) == 0) { }   /* 等 TXE */
		USART2->DR = (uint8_t)ptr[i];
	}

	return len;
}

void *_sbrk(int incr)
{
	static char *heap_end = 0;
	char *prev;
	char *limit = &_estack - 0x800;   /* 给主栈留 2KB */

	if (heap_end == 0)
	{
		heap_end = &end;
	}

	prev = heap_end;
	if (heap_end + incr > limit)
	{
		errno = ENOMEM;
		return (void *)-1;
	}

	heap_end += incr;
	return prev;
}

int _close(int file)
{
	(void)file;
	return -1;
}

int _fstat(int file, struct stat *st)
{
	(void)file;
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	(void)file;
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	(void)file;
	(void)ptr;
	(void)dir;
	return 0;
}

int _read(int file, char *ptr, int len)
{
	(void)file;
	(void)ptr;
	(void)len;
	return 0;
}

void _exit(int status)
{
	(void)status;
	for (;;) { }
}

int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig;
	errno = EINVAL;
	return -1;
}

int _getpid(void)
{
	return 1;
}
