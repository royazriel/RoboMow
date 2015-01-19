/*
 * motor_control.c
 *
 *  Created on: Dec 23, 2014
 *      Author: roy
 */

#include "motor_control.h"

static uint32_t sMotorEncoderSum[NUMBER_OF_MOTORS] = {};

void MotorsControlDrive( double rightSpeed,double leftSpeed,Directions dir )
{
	static uint32_t lastTime = 0;
	uint32_t requestedSpeedInPulses[NUMBER_OF_MOTORS];
	int32_t speedErrorInPulses[NUMBER_OF_MOTORS];
	uint16_t encoderVal[NUMBER_OF_MOTORS];
	uint16_t newPwmVal;

	requestedSpeedInPulses[etMotorRight]=rightSpeed*PULSE_PER_METER;
	requestedSpeedInPulses[etMotorLeft]=leftSpeed*PULSE_PER_METER;

	switch(dir)
	{
		case etDirForward:
			MOTOR_DIR_PORT->BSRR |= MOTOR_DIR_R;
			MOTOR_DIR_PORT->BSRR  |= MOTOR_DIR_L;
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
		case etDirStop:
			MOTOR_PWM_R = STOP_SPEED;
			MOTOR_PWM_L = STOP_SPEED;
			goto EXIT;
			break;
		default:
			break;
	}

	if(MOTOR_PWM_R == STOP_SPEED && MOTOR_PWM_L == STOP_SPEED)
	{
		lastTime = GetMiliSecondCount();
		MOTOR_PWM_R = LOW_SPEED;
		MOTOR_PWM_L = LOW_SPEED;
		TIM_SetCounter (ENCODER_R, 0);
		TIM_SetCounter (ENCODER_L, 0);
	}

	if(GetMiliSecondCount()-lastTime > CONTROL_LOOP_RESOLUTION_MILI)
	{
		encoderVal[etMotorRight] = TIM_GetCounter(ENCODER_R);
		TIM_SetCounter (ENCODER_R, 0);
		sMotorEncoderSum[etMotorRight] +=encoderVal[etMotorRight];
		speedErrorInPulses[etMotorRight] = ((double)encoderVal[etMotorRight]/(NORMALIZE_TO_SECOND)) - requestedSpeedInPulses[etMotorRight];
		lastTime= GetMiliSecondCount();

		newPwmVal = MOTOR_PWM_R/*LOW_SPEED*/+(speedErrorInPulses[etMotorRight] * MOTOR_CONTROL_GAIN);
		if(newPwmVal < HIGH_SPEED) newPwmVal = HIGH_SPEED;
		MOTOR_PWM_R = newPwmVal;

		encoderVal[etMotorLeft] = TIM_GetCounter(ENCODER_L);
		TIM_SetCounter (ENCODER_L, 0);
		sMotorEncoderSum[etMotorLeft] +=encoderVal[etMotorLeft];
		speedErrorInPulses[etMotorLeft] = ((double)encoderVal[etMotorLeft]/(NORMALIZE_TO_SECOND)) - requestedSpeedInPulses[etMotorLeft];
		lastTime= GetMiliSecondCount();

		newPwmVal = MOTOR_PWM_L/*LOW_SPEED*/+(speedErrorInPulses[etMotorLeft] * MOTOR_CONTROL_GAIN);
		if(newPwmVal < HIGH_SPEED) newPwmVal = HIGH_SPEED;

		MOTOR_PWM_L = newPwmVal;

		UsartPrintf("l_speed: %2.2f r_speed: %2.2f ", leftSpeed,rightSpeed);
		UsartPrintf("ERR_L: %d PWM_L %d  | ERR_R: %d PWM_R %d | DIS_L: %f  DIS_R: %f\r\n",speedErrorInPulses[etMotorLeft], MOTOR_PWM_L,
																							speedErrorInPulses[etMotorRight],MOTOR_PWM_R,
																							MotorControlGetDistance(etMotorLeft),MotorControlGetDistance(etMotorRight));
	}
EXIT:{}
}

double MotorControlGetDistance( MotorSide side )
{
	return sMotorEncoderSum[side]/PULSE_PER_METER;
}

void MotorControlResetDistance()
{
	sMotorEncoderSum[etMotorLeft] = 0;
	sMotorEncoderSum[etMotorRight] = 0;
}
