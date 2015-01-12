/*
 * motor_control.c
 *
 *  Created on: Dec 23, 2014
 *      Author: roy
 */

#include "motor_control.h"

void MotorsControlDrive( uint16_t speed, Directions dir )
{
	if( speed < HIGH_SPEED ) speed = HIGH_SPEED;
	if( speed > LOW_SPEED ) speed = LOW_SPEED;
	if( dir!= etDirForwardRightFaster && dir != etDirForwardLeftFaster)
	{
		MOTOR_PWM_L = MOTOR_PWM_R = speed;
	}

	//UsartPrintf( "speed %d dir %d\r\n",speed,dir);

	switch(dir)
	{
		case etDirForward:
			MOTOR_DIR_PORT->BSRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BSRR  |= MOTOR_DIR_L;
			break;
		case etDirForwardRightFaster:
			MOTOR_PWM_R = speed;
			break;
		case etDirForwardLeftFaster:
			MOTOR_PWM_L = speed;
			break;
		case etDirReverse:
			MOTOR_DIR_PORT->BRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BRR  |= MOTOR_DIR_L;
			break;
		case etDirCCW:
			MOTOR_DIR_PORT->BSRR |= MOTOR_DIR_L;
			MOTOR_DIR_PORT->BRR  |= MOTOR_DIR_R;
			break;
		case etDirCW:
			MOTOR_DIR_PORT->BRR |= MOTOR_DIR_L;
			MOTOR_DIR_PORT->BSRR  |= MOTOR_DIR_R;
			break;
		default:
			break;
	}
}

