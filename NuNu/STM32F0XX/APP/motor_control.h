/*
 * motor_control.h
 *
 *  Created on: Dec 23, 2014
 *      Author: roy
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "hardware_config.h"
#define STOP_SPEED			500
#define LOW_SPEED 			400				//(SystemCoreClock/PWM_FREQUENCy) = 2730 ~90% 273
#define MID_SPEED			350
#define HIGH_SPEED			300

typedef enum{
	etSpeedStop			= 	STOP_SPEED	,
	etSpeedLow			=	LOW_SPEED 	,
	etSpeedMid			=	MID_SPEED	,
	etSpeedHigh			=	HIGH_SPEED
}Speeds;

typedef enum
{
	etDirForward 				= 	0	,
	etDirForwardRightFaster				,
	etDirForwardLeftFaster				,
	etDirReverse						,
	etDirCW								,
	etDirCCW
}Directions;

void MotorsControlDrive( uint16_t speed, Directions dir );

#endif /* MOTOR_CONTROL_H_ */
