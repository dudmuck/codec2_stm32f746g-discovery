/**
*****************************************************************************
**
**  File        : syscalls.c
**
**  Abstract    : System Workbench Minimal System calls file
**
** 		          For more information about which c-functions
**                need which of these lowlevel functions
**                please consult the Newlib libc-manual
**
**  Environment : System Workbench for MCU
**
**  Distribution: The file is distributed �as is,� without any warranty
**                of any kind.
**
**  (c)Copyright System Workbench for MCU.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. System Workbench for MCU permit registered System Workbench(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the System Workbench for MCU toolchain.
**
*****************************************************************************
*/

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


/* Variables */
//#undef errno
extern int errno;
#define FreeRTOS
#define MAX_STACK_SIZE 0x2000

extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

#ifndef FreeRTOS
  register char * stack_ptr asm("sp");
#endif


register char * stack_ptr asm("sp");

char *__env[1] = { 0 };
char **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

int _read (int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = __io_getchar();
	}

return len;
}

#include "stm32f7xx_hal.h"
UART_HandleTypeDef UartHandle;

/* Non-blocking UART TX with ring buffer using interrupt mode.
 * Data is copied to ring buffer, interrupt drains in background.
 * Only blocks if buffer fills up (back-pressure).
 * Uses interrupt mode instead of DMA to avoid DMA2_Stream7 conflict with audio.
 */
#define UART_TX_BUF_SIZE    2048  /* Must be power of 2 */
#define UART_TX_BUF_MASK    (UART_TX_BUF_SIZE - 1)

static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;  /* Write position */
static volatile uint16_t tx_tail = 0;  /* TX read position */
static volatile uint8_t tx_busy = 0;

/* Start interrupt transfer if data available and TX not busy */
static void uart_tx_start(void)
{
    if (tx_busy || tx_head == tx_tail)
        return;

    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    uint16_t len;

    if (head > tail) {
        /* Contiguous block from tail to head */
        len = head - tail;
    } else {
        /* Wrap-around: send from tail to end of buffer */
        len = UART_TX_BUF_SIZE - tail;
    }

    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&UartHandle, &uart_tx_buf[tail], len) != HAL_OK) {
        /* TX failed (HAL_BUSY/HAL_ERROR) - clear busy flag to avoid deadlock */
        tx_busy = 0;
    }
}

/* Called from HAL when interrupt transfer completes */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &UartHandle)
        return;

    /* Advance tail by amount just transmitted */
    uint16_t transmitted = huart->TxXferSize;
    tx_tail = (tx_tail + transmitted) & UART_TX_BUF_MASK;
    tx_busy = 0;

    /* Start next transfer if more data pending */
    uart_tx_start();
}

#define TX_TIMEOUT              100
int _write(int file, char *ptr, int len)
{
#if 1  /* Interrupt mode with ring buffer */
    int written = 0;

    while (written < len) {
        uint16_t head = tx_head;
        uint16_t next_head = (head + 1) & UART_TX_BUF_MASK;

        /* If buffer full, wait for TX to drain some */
        while (next_head == tx_tail) {
            /* Buffer full - ensure TX is running */
            if (!tx_busy)
                uart_tx_start();
            /* Brief wait then check again */
            for (volatile int i = 0; i < 100; i++) asm("nop");
        }

        uart_tx_buf[head] = ptr[written++];
        tx_head = next_head;
    }

    /* Kick off TX if not already running */
    if (!tx_busy)
        uart_tx_start();

    return len;
#else  /* Blocking polled TX */
    if (HAL_UART_Transmit(&UartHandle, (uint8_t*)ptr, len, TX_TIMEOUT) == HAL_OK)
        return len;
    else
        return -1;
#endif
}

caddr_t _sbrk(int incr)
{
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _close(int file)
{
	return -1;
}


int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}
