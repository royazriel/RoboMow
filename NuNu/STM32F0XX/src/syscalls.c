/*
 * syscalls.c
 *
 *  Created on: Apr 6, 2011
 *      Author: Ekawahyu Susilo
 */

#include "compiler.h"
#include "hardware_config.h"
#include <errno.h>
#undef errno
extern int errno;

#ifdef __caddr_t_required_
typedef char * caddr_t;
#endif

#ifdef __stat_struct_required_
struct stat
{
	unsigned long int st_mode;
};
#endif

#ifndef ENOMEM
#define	ENOMEM 12 /* Not enough core */
#endif

#ifndef EBADF
#define EBADF 9 /* Bad file number */
#endif

#ifndef EINVAL
#define EINVAL 22 /* Invalid argument */
#endif

#ifndef _IFCHR
#define _IFCHR 0020000 /* character special */
#endif

#ifndef S_IFCHR
#define S_IFCHR _IFCHR
#endif


/* ALL non reentrant function declaration start here
 *
 */

int _write(int fd, char *ptr, size_t len)
{
	size_t counter = len;

	if (fd == STDOUT_FILENO || fd == STDERR_FILENO){}
	else return -1;

	while(counter-- > 0) { // Send the character from the buffer to UART
		while (port_USART_DEBUG_busy_sending());
		port_USART_DEBUG_send_data(*ptr);

		ptr++;
	}

	return len;
}

void _exit(int status) {
	_write (STDERR_FILENO, "exit1", 5);
	// visual blinking indicator when application exits and terminated
	while (1)
	{
		Sleep(50);
	}
}


/* ALL reentrant function declaration start here
 *
 */

/* Return a clock that ticks at CLOCKS_PER_SEC Hz.  */
clock_t _times_r(void *reent, struct tms * tp)
{
  clock_t timeval = GetMiliSecondCount(); //time32_incr;

  if (tp)
    {
      tp->tms_utime  = timeval;	/* user time */
      tp->tms_stime  = 0;	/* system time */
      tp->tms_cutime = 0;	/* user time, children */
      tp->tms_cstime = 0;	/* system time, children */
    }

  return timeval;
};

int _read_r(int fd, char *ptr, size_t len)
{
	int i;
	size_t counter = 0;

	if(fd == STDIN_FILENO){}
	else return -1;
#if 0
	for (i = 0; i < len; i++) { // Get characters from the UART
		while (port_USARTx_no_data());
		*ptr++ = (char)port_USARTx_receive_data();
		counter++;
	}
#endif
	return counter;
}

int _close_r(int file)
{
	return -1;
}

int _fstat_r(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;

	return 0;
}

int _isatty_r(int file)
{
	switch (file)
	{
		case STDOUT_FILENO:
		case STDERR_FILENO:
		case STDIN_FILENO:
			return 1;
		default:
			//errno = ENOTTY;
			errno = EBADF;
			return 0;
	}
}

#define __MY_PID	1
int _getpid_r(void)
{
	return  __MY_PID; /* return any number as PID */
}

int _lseek_r(int file, int ptr, int dir)
{
	return 0;
}

int _kill_r(int pid, int sig)
{
	errno = EINVAL;

	return -1; /* Always fails */
}
