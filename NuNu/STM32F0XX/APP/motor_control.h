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
#define LOW_SPEED 			380			//(SystemCoreClock/PWM_FREQUENCy) = 2730 ~90% 273
#define MID_SPEED			350
#define HIGH_SPEED			200
#define NUMBER_OF_MOTORS	2

typedef enum{
	etMotorLeft = 0,
	etMotorRight
}MotorSide;

typedef enum{
	etSpeedStop			= 	STOP_SPEED	,
	etSpeedLow			=	LOW_SPEED 	,
	etSpeedMid			=	MID_SPEED	,
	etSpeedHigh			=	HIGH_SPEED
}Speeds;

typedef enum
{
	etDirForward 				= 	0	,
	etDirStop,
	etDirReverse						,
	etDirCW								,
	etDirCCW
}Directions;

void MotorsControlDrive( double rightSpeed,double leftSpeed,Directions dir );
double MotorControlGetDistance( MotorSide side );
void MotorControlResetDistance();

#endif /* MOTOR_CONTROL_H_ */
