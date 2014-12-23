/*
 * motor_control.c
 *
 *  Created on: Dec 23, 2014
 *      Author: roy
 */

#include "motor_control.h"

void MotorsControlDrive( Speeds speed, Directions dir )
{
	MOTOR_PWM_L = MOTOR_PWM_R = speed;

	UsartPrintf( "speed %d dir %d\r\n",speed,dir);

	switch(dir)
	{
		case etDirForword:

			MOTOR_DIR_PORT->BSRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BSRR  |= MOTOR_DIR_L;
			break;
		case etDirReverse:
			MOTOR_DIR_PORT->BRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BRR  |= MOTOR_DIR_L;
			break;
		case etDirCCW:
			MOTOR_DIR_PORT->BSRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BRR  |= MOTOR_DIR_L;
			break;
		case etDirCW:
			MOTOR_DIR_PORT->BRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BSRR  |= MOTOR_DIR_L;
			break;
		default:
			break;
	}
}

