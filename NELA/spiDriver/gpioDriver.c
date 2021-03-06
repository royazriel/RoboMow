/*
 * gpioDriver.c
 *
 *  Created on: Oct 23, 2014
 *      Author: roy
 */


#include "gpioDriver.h"

int gDecawaveIrqFd;

#define MAX_BUF 64
#define SYSFS_GPIO_DIR "/sys/class/gpio"

#if 0
int GPIOExport(int pin) {

#define BUFFER_MAX 3

	char buffer[BUFFER_MAX];

	ssize_t bytes_written;

	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);

	if (-1 == fd) {

		fprintf(stderr, "Failed to open export for writing!\n");

		return (-1);

	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);

	write(fd, buffer, bytes_written);

	close(fd);

	return (0);
}

int GPIOUnexport(int pin)
{
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd)
	{
		fprintf(stderr, "Failed to open unexport for writing!\n");
		return (-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return (0);
}
#endif

int GPIOExport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int GPIOUnexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

int GPIODirection(int pin, int dir)
{
	static const char s_directions_str[]  = "in\0out";

#define DIRECTION_MAX 35
	char path[DIRECTION_MAX];
	int fd;

	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		return(-1);
	}

	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
		fprintf(stderr, "Failed to set direction!\n");
		return(-1);
	}

	close(fd);
	return(0);
}

int GPIORead(int pin)
{
#define VALUE_MAX 50
	char path[VALUE_MAX];
	char value_str[3];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for reading!\n");
		return(-1);
	}

	if (-1 == read(fd, value_str, 3)) {
		fprintf(stderr, "Failed to read value!\n");
		return(-1);
	}

	close(fd);

	return(atoi(value_str));
}

int GPIOReadByDescriptor(int fd)
{
	char buf[32];
	int retval;

	retval = lseek(fd, 0, SEEK_SET);
	if (retval < 0)
	{
		PINFO("Error: read: %s\n", strerror(errno));
		return 1;
	}

	retval = read(fd, buf, 10);
	if (retval < 0)
	{
		PINFO("Error: read: %s\n", strerror(errno));
		return 1;
	}

	//printf(" irq value %s\r\n", buf);

	return 0;
}

int GPIOWrite(int pin, int value)
{
	static const char s_values_str[] = "01";

	char path[VALUE_MAX];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for writing!\n");
		return(-1);
	}

	if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
		fprintf(stderr, "Failed to write value!\n");
		return(-1);
	}

	close(fd);
	return(0);
}

int GPIOSetEdge(int pin, int irq_type, int active_low)
{
	char path[VALUE_MAX];
	int fd;
	int retval;
	char* edge_str;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/edge", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd)
	{
		fprintf(stderr, "Failed to open gpio edge for writing!\n");
		return(-1);
	}

	switch(irq_type)
	{
		case GPIO_IRQ_EDGE_FALLING:
			edge_str = "falling";
			break;
		case GPIO_IRQ_EDGE_RISING:
			edge_str = "rising";
			break;
		case GPIO_IRQ_EDGE_BOTH:
		default:
			edge_str = "both";
			break;
	}

	retval = write( fd, edge_str, strlen(edge_str));
	if (retval < 0)
	{
		fprintf(stderr,"Error: write: %s\n", strerror(errno));
		return 1;
	}

	close(fd);

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/active_low", pin);
	fd = open(path, O_WRONLY);
	if (fd < 0)
	{
		printf("Error: open: %s\n", strerror(errno));
		return 1;
	}

	retval = write(fd, active_low ? "1" : "0", 1);
	if (retval < 0)
	{
		printf("Error: write: %s\n", strerror(errno));
		return 1;
	}

	close(fd);

	return 0;
}

int GPIOPoll(int pin, int openFile )
{
	char path[VALUE_MAX];
	struct pollfd pfd;
	int retval;

	if( openFile == 1 )
	{
			snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
			gDecawaveIrqFd = open(path, O_RDONLY);
			if (-1 == gDecawaveIrqFd) {
				fprintf(stderr, "Failed to open gpio value for writing!\n");
				return(-1);
			}
			GPIOSetEdge( pin, GPIO_IRQ_EDGE_RISING,0);
	}
	else
	{
		pfd.fd = gDecawaveIrqFd;
		pfd.events = POLLPRI | POLLERR;
		retval = poll(&pfd, 1, -1);
		if (retval < 0)
		{
			printf("Error: poll: %s\n", strerror(errno));
			return 1;
		}
		GPIOReadByDescriptor(gDecawaveIrqFd);
		dwt_isr();

	}
}

