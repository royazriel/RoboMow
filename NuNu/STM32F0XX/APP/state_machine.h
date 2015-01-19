/*
 * state_machine.h
 *
 *  Created on: Jan 18, 2015
 *      Author: roy
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "hardware_config.h"
#include "motor_control.h"
#include "lis3dh_driver.h"

#define DEFAULT_SPEED 				0.03  						//5cm/sec
#define DEFAULT_SIDE_SPEED 			0.1							//10cm/sec
#define DRIVE_UP_DEFAULT_SPEED		0.15   						//15cm/sec
#define DRIVE_DOWN_DEFAULT_SPEED  	0.06		  				//meter/sec
#define DEFAULT_TIME_BETWEEN_STATES 300
#define TILT_ANGLE_TOLLERANCE 3.0f //deg
#define TILT_GAIN	0.005f
#define TILT_GAIN_SIDE_DRIVE	0.001f
#define REVERSE_3CM	0.03f
#define OVERLAP_COVERAGE_SIZE		(DISTANCE_BETWEEN_WHEELS/2)

typedef enum
{
	etTurnRight = 0,
	etTurnLeft =  1,
	etTurnUnknown
}Turns;

typedef enum
{
	etStatesWaitForInit,
	etStatesFaceLeft,
	etStatesFaceLeftDone,
	etStatesDriveLeft,
	etStatesDriveLeftDone,
	etStatesShortReverse,
	etStatesShortReverseDone,
	etStatesFaceUp,
	etStatesFaceUpDone,
	etStatesDriveUpDown,
	etStatesDriveUpDownDone,
	etStatesShortReverse1,
	etStatesShortReverse1Done,
	etStatesFaceRight,
	etStatesFaceRightDone,
	etStatesDriveRight,
	etStatesDriveRightDone,
	etStatesFaceDown,
	etStatesFaceDownDone,
	etStatesFaceLeft1,
	etStatesFaceLeft1Done,
	etStatesDriveLeft1,
	etStatesDriveLeft1Done,
	etStatesCoverageDone,
	etStatesLastState
}States;

typedef struct _StateString {
	States state;
	uint8_t name[30];
}Code2String;

void StateMachineHandleStates();

#endif /* STATE_MACHINE_H_ */
