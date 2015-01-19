/*
 * state_machine.c
 *
 *  Created on: Jan 11, 2015
 *      Author: roy
 */

#include "state_machine.h"

#define __DEBUG

#ifdef __DEBUG
Code2String StatesTable[etStatesLastState+1] =
{
	{etStatesWaitForInit       ,"etStatesWaitForInit       "},
	{etStatesFaceLeft          ,"etStatesFaceLeft          "},
	{etStatesFaceLeftDone      ,"etStatesFaceLeftDone      "},
	{etStatesDriveLeft         ,"etStatesDriveLeft         "},
	{etStatesDriveLeftDone     ,"etStatesDriveLeftDone     "},
	{etStatesShortReverse      ,"etStatesShortReverse      "},
	{etStatesShortReverseDone  ,"etStatesShortReverseDone  "},
	{etStatesFaceUp            ,"etStatesFaceUp            "},
	{etStatesFaceUpDone        ,"etStatesFaceUpDone        "},
	{etStatesDriveUpDown       ,"etStatesDriveUpDown       "},
	{etStatesDriveUpDownDone   ,"etStatesDriveUpDownDone   "},
	{etStatesShortReverse1     ,"etStatesShortReverse1     "},
	{etStatesShortReverse1Done ,"etStatesShortReverse1Done "},
	{etStatesFaceRight         ,"etStatesFaceRight         "},
	{etStatesFaceRightDone     ,"etStatesFaceRightDone     "},
    {etStatesDriveRight        ,"etStatesDriveRight        "},
	{etStatesDriveRightDone    ,"etStatesDriveRightDone    "},
    {etStatesFaceDown          ,"etStatesFaceDown          "},
    {etStatesFaceDownDone      ,"etStatesFaceDownDone      "},
    {etStatesFaceLeft1         ,"etStatesFaceLeft1         "},
    {etStatesFaceLeft1Done     ,"etStatesFaceLeft1Done     "},
    {etStatesDriveLeft1        ,"etStatesDriveLeft1        "},
    {etStatesDriveLeft1Done    ,"etStatesDriveLeft1Done    "},
    {etStatesCoverageDone      ,"etStatesCoverageDone      "},
    {etStatesLastState         ,"etStatesLastState         "}
};

uint8_t * GetCodeName( Code2String* table, uint8_t code )
{
	int i=0;
	while(table[i].state != etStatesLastState )
	{
		if( code == table[i].state)
		{
			break;
		}
		i++;
	}
	return table[i].name;
}
#endif

static States sCurrentState = etStatesWaitForInit;
static States sPrevState = etStatesLastState;
static uint32_t sCurrentStateStartTime = 0;
static uint8_t sWaitBeforeNextState = 0;
void StateMachineHandleStates()
{
	double tilt;
	TiltDirection dir;
	status_t res;
	States state;
	static Turns nextTurn = etTurnRight;

	if(sWaitBeforeNextState && GetMiliSecondCount()- sCurrentStateStartTime < DEFAULT_TIME_BETWEEN_STATES ) return;

	state = sCurrentState;
	switch( sCurrentState )
	{
		case etStatesWaitForInit:
			if(!BUMPER_FRONT_STATE)
			{
				MotorsControlDrive(DEFAULT_SPEED,DEFAULT_SPEED, etDirCCW);
				sCurrentState = etStatesFaceLeft;
			}
			break;
		case etStatesFaceLeft:
			MotorsControlDrive(DEFAULT_SPEED,DEFAULT_SPEED, etDirCCW);
			res = GetXAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS )
			{

				if( tilt  < -90.0f + TILT_ANGLE_TOLLERANCE)
				{
					UsartPrintf("tilt %f dir %s\r\n",tilt, dir == etTiltUp ? "UP  ":"DOWN");
					MotorsControlDrive( 0,0, etDirStop);
					sWaitBeforeNextState = 1;	//prevent inertia
					sCurrentState = etStatesFaceLeftDone;
				}
			}
			break;
		case etStatesFaceLeftDone:
				MotorsControlDrive( DEFAULT_SPEED,DEFAULT_SPEED, etDirForward);
				sWaitBeforeNextState = 0;
				sCurrentState = etStatesDriveLeft;
			break;
		case etStatesDriveLeft:
			res = GetYAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS && abs(tilt) < 90.0f)
			{
				UsartPrintf("tilt %2.2f dir %s ", tilt, dir == etTiltUp ? "UP  ":"DOWN");
				if(dir == etTiltUp)
				{
					MotorsControlDrive( DEFAULT_SIDE_SPEED-(tilt*TILT_GAIN), DEFAULT_SIDE_SPEED, etDirForward);
				}
				if(dir == etTiltDown)
				{
					MotorsControlDrive( DEFAULT_SIDE_SPEED, DEFAULT_SIDE_SPEED+(tilt*TILT_GAIN), etDirForward);
				}
			}
			if(!BUMPER_FRONT_STATE)
			{
				UsartPrintf("Front bummper - End of Drive Left\r\n");
				MotorsControlDrive( 0,0, etDirStop);
				sWaitBeforeNextState = 1;
				sCurrentState = etStatesDriveLeftDone;
			}
			break;
		case etStatesDriveLeftDone:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirReverse);
			sWaitBeforeNextState = 0;
			sCurrentState = etStatesShortReverse;
			MotorControlResetDistance();
			break;
		case etStatesShortReverse:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirReverse);
			if( MotorControlGetDistance(etMotorLeft)> REVERSE_3CM && MotorControlGetDistance(etMotorRight)> REVERSE_3CM)
			{
				UsartPrintf("R %f meter L %f meters  - End of Short Reverse\r\n", MotorControlGetDistance(etMotorRight), MotorControlGetDistance(etMotorLeft));
				MotorsControlDrive( 0,0, etDirStop);
				sWaitBeforeNextState = 1;
				sCurrentState = etStatesShortReverseDone;
			}
			break;

		case etStatesShortReverseDone:
			MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
			sWaitBeforeNextState = 0;
			sCurrentState = etStatesFaceUp;
			break;
		case etStatesFaceUp:
			if( sPrevState == etStatesShortReverseDone  )
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
			}
			if( sPrevState == etStatesDriveLeft1Done  )
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCCW);
			}
			res = GetXAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS )
			{

				if( dir == etTiltUp && abs(tilt) < TILT_ANGLE_TOLLERANCE )
				{
					UsartPrintf("tilt %f dir %s\r\n",tilt, dir == etTiltUp ? "UP":"DOWN");
					MotorsControlDrive( 0,0, etDirStop);
					sWaitBeforeNextState = 1;	//prevent inertia
					sCurrentState = etStatesFaceUpDone;
				}
			}
			break;
		case etStatesFaceUpDone:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirForward);
			sWaitBeforeNextState = 0;
			sCurrentState = etStatesDriveUpDown;
			//debugTimer = GetMiliSecondCount();
			break;
		case etStatesDriveUpDown:
			res = GetXAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS )
			{
				UsartPrintf("tilt %2.2f ", tilt);
				if( sPrevState == etStatesFaceUpDone)
				{
					if(tilt > 0)
					{
						MotorsControlDrive( DRIVE_UP_DEFAULT_SPEED+(tilt*TILT_GAIN), DRIVE_UP_DEFAULT_SPEED, etDirForward);
					}
					if(tilt < 0)
					{
						MotorsControlDrive( DRIVE_UP_DEFAULT_SPEED, DRIVE_UP_DEFAULT_SPEED-(tilt*TILT_GAIN), etDirForward);
					}
				}
				if( sPrevState == etStatesFaceDownDone)
				{
					if(tilt > 0)
					{
						MotorsControlDrive( DRIVE_DOWN_DEFAULT_SPEED, DRIVE_DOWN_DEFAULT_SPEED+(tilt*TILT_GAIN), etDirForward);
					}
					if(tilt < 0)
					{
						MotorsControlDrive( DRIVE_DOWN_DEFAULT_SPEED-(tilt*TILT_GAIN), DRIVE_DOWN_DEFAULT_SPEED, etDirForward);
					}
				}
			}
			if(!BUMPER_FRONT_STATE)
			{
				UsartPrintf("Front bumper - End of etStatesDriveUpDown\r\n");
				MotorsControlDrive( 0,0, etDirStop);
				sWaitBeforeNextState = 1;
				if(sPrevState == etStatesFaceUpDone)
				{
					nextTurn = etTurnRight;
				}
				if(sPrevState == etStatesFaceDownDone)
				{
					nextTurn = etTurnLeft;
				}
				sCurrentState = etStatesDriveUpDownDone;
			}
			break;
		case etStatesDriveUpDownDone:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirReverse);
			sWaitBeforeNextState = 0;
			sCurrentState = etStatesShortReverse1;
			MotorControlResetDistance();
			break;
		case etStatesShortReverse1:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirReverse);
			if( MotorControlGetDistance(etMotorLeft)> REVERSE_3CM && MotorControlGetDistance(etMotorRight)> REVERSE_3CM)  //5cm
			{
				UsartPrintf("R %f meter L %f meters  - End of Short Reverse1\r\n", MotorControlGetDistance(etMotorRight), MotorControlGetDistance(etMotorLeft));
				MotorsControlDrive( 0,0, etDirStop);
				sWaitBeforeNextState = 1;
				sCurrentState = etStatesShortReverse1Done;
			}
			break;
		case etStatesShortReverse1Done:
			if( nextTurn == etTurnRight)
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
				sWaitBeforeNextState = 0;
				sCurrentState = etStatesFaceRight;
			}
			if( nextTurn == etTurnLeft)
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCCW);
				sWaitBeforeNextState = 0;
				sCurrentState = etStatesFaceLeft1;
			}

			break;
		case etStatesFaceLeft1:
		case etStatesFaceRight:
			if( sCurrentState == etStatesFaceRight)
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
			}
			if( sCurrentState == etStatesFaceLeft1)
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCCW);
			}
			res = GetXAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS )
			{
				//UsartPrintf("tilt %f dir %s\r\n",tilt, dir == etTiltUp ? "UP":"DOWN");
				if( tilt > 90.0 - TILT_ANGLE_TOLLERANCE )
				{
					MotorsControlDrive( 0,0, etDirStop);
					sWaitBeforeNextState = 1;	//prevent inertia
					if( sCurrentState == etStatesFaceRight)
					{
						sCurrentState = etStatesFaceRightDone;
					}
					if( sCurrentState == etStatesFaceLeft1)
					{
						sCurrentState = etStatesFaceLeft1Done;
					}
				}
			}
			break;
		case etStatesFaceLeft1Done:
		case etStatesFaceRightDone:
			MotorsControlDrive( DEFAULT_SPEED, DEFAULT_SPEED, etDirForward);
			sWaitBeforeNextState = 0;
			if(sCurrentState == etStatesFaceRightDone)
				sCurrentState = etStatesDriveRight;
			if(sCurrentState == etStatesFaceLeft1Done)
				sCurrentState = etStatesDriveLeft1;
			MotorControlResetDistance();
			break;
		case etStatesDriveLeft1:
		case etStatesDriveRight:
			res = GetYAxisTilt( &tilt, & dir);
			if(sCurrentState == etStatesDriveLeft1 )
			{
				UsartPrintf("tilt %2.2f dir %s ", tilt, dir == etTiltUp ? "UP  ":"DOWN");
				if(res == MEMS_SUCCESS && abs(tilt) < 90.0f)
				{

					if(dir == etTiltUp)
					{
						MotorsControlDrive( DEFAULT_SIDE_SPEED+(tilt*TILT_GAIN), DEFAULT_SIDE_SPEED, etDirForward);
					}
					if(dir == etTiltDown)
					{
						MotorsControlDrive( DEFAULT_SIDE_SPEED, DEFAULT_SIDE_SPEED-(tilt*TILT_GAIN), etDirForward);
					}
				}
			}
			if(sCurrentState == etStatesDriveRight )
			{
				if(res == MEMS_SUCCESS && abs(tilt) < 90.0f)
				{
					if(dir == etTiltUp)
					{
						MotorsControlDrive( DEFAULT_SIDE_SPEED, DEFAULT_SIDE_SPEED-(tilt*TILT_GAIN), etDirForward);
					}
					if(dir == etTiltDown)
					{
						MotorsControlDrive( DEFAULT_SIDE_SPEED+(tilt*TILT_GAIN), DEFAULT_SIDE_SPEED, etDirForward);
					}
				}
			}
			if( MotorControlGetDistance(etMotorLeft)> (DISTANCE_BETWEEN_WHEELS-OVERLAP_COVERAGE_SIZE ) &&
					MotorControlGetDistance(etMotorRight) > DISTANCE_BETWEEN_WHEELS-OVERLAP_COVERAGE_SIZE)  //5cm
			{
				UsartPrintf("R %f meter L %f meters  - End of Drive To next leg\r\n", MotorControlGetDistance(etMotorRight), MotorControlGetDistance(etMotorLeft));
				MotorsControlDrive( 0,0, etDirStop);
				sWaitBeforeNextState = 1;
				if(sCurrentState == etStatesDriveRight )
					sCurrentState = etStatesDriveRightDone;
				if(sCurrentState == etStatesDriveLeft1 )
					sCurrentState = etStatesDriveLeft1Done;
			}
			break;
		case etStatesDriveLeft1Done:
		case etStatesDriveRightDone:
			sWaitBeforeNextState = 0;
			if( sCurrentState == etStatesDriveRightDone  )
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
				sCurrentState = etStatesFaceDown;
			}
			if( sCurrentState == etStatesDriveLeft1Done  )
			{
				MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCCW);
				sCurrentState = etStatesFaceUp;
			}
			break;
		case etStatesFaceDown:
			MotorsControlDrive(DEFAULT_SPEED, DEFAULT_SPEED, etDirCW);
			res = GetXAxisTilt( &tilt, & dir);
			if(res == MEMS_SUCCESS )
			{
				if( dir == etTiltDown && tilt < TILT_ANGLE_TOLLERANCE )
				{
					MotorsControlDrive( 0,0, etDirStop);
					sWaitBeforeNextState = 1;	//prevent inertia
					sCurrentState = etStatesFaceDownDone;
				}
			}
			break;
		case etStatesFaceDownDone:
			MotorsControlDrive( DEFAULT_SPEED,DEFAULT_SPEED, etDirForward);
			sWaitBeforeNextState = 0;
			sCurrentState = etStatesDriveUpDown;
			break;
		default:
			UsartPrintf("unexpected state\r\n");
			break;
	}
	if(state != sCurrentState)
	{
		sPrevState = state;
		sCurrentStateStartTime = GetMiliSecondCount();
		MotorControlResetDistance();
#ifdef __DEBUG
		UsartPrintf("Current State: %s\r\n",GetCodeName(StatesTable, sCurrentState));
#endif
	}
}

