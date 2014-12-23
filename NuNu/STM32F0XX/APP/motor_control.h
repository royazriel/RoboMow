/*
 * motor_control.h
 *
 *  Created on: Dec 23, 2014
 *      Author: roy
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "hardware_config.h"

#define LOW_SPEED 			300				//(SystemCoreClock/PWM_FREQUENCy) = 2730 ~90% 273
#define MID_SPEED			150
#define HIGH_SPEED			0

typedef enum{
	etStop 			= 	0				,
	etSpeedLow			=	LOW_SPEED 	,
	etSpeedMid			=	MID_SPEED	,
	etSpeedHigh			=	HIGH_SPEED
}Speeds;

typedef enum
{
	etDirForword 		= 	0			,
	etDirReverse						,
	etDirCW								,
	etDirCCW
}Directions;

void MotorsControlDrive( Speeds speed, Directions dir );

#endif /* MOTOR_CONTROL_H_ */
