/*
 * gpioDriver.h
 *
 *  Created on: Oct 23, 2014
 *      Author: roy
 */

#ifndef SPIDRIVER_GPIODRIVER_H_
#define SPIDRIVER_GPIODRIVER_H_

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define IN   0
#define OUT  1

#define LOW  0
#define HIGH 1

#define SYSFS_GPIO_DIR "/sys/class/gpio"

#define DW_EXT_ON		22			//header pin 15
#define DW_RESET_PIN 	23			//header pin 16
#define DW_WAKEUP_PIN 	24			//header pin 18
#define DW_IRQ_PIN		25			//header pin 22

int GPIOExport(int pin);
int GPIOUnexport(int pin);
int GPIODirection(int pin, int dir);
int GPIORead(int pin);
int GPIOWrite(int pin, int value);

#endif /* SPIDRIVER_GPIODRIVER_H_ */
