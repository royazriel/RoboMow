//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "compiler.h"
#include "diag/Trace.h"
#include "hardware_config.h"
#include "lis3dh_driver.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via ITM).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
	char buffer[100];
	double tilt;
	status_t response;


	InitSystemTick();
	InitGpio();
	InitUsartDebug();
	InitI2c();
	InitSpi();
	InitPwm();

	LIS3DH_Configure();

	while (1)
	{
		if(PB1_STATE)
		{
			//get Acceleration Raw data
			response = GetOneAxisTilt( &tilt );
			if(response==MEMS_SUCCESS)
			{
				//print data values
				UsartPrintf( "tilt %2.2f \r\n",tilt);
			}
			/*******************************************************/
			if(PB1_STATE)
			{
				if(GPIO_ReadOutputDataBit(LED_PORT,LED1))
				{
					LED_PORT->BRR |= LED1;
				}else
				{
					LED_PORT->BSRR |= LED1;
				}
			}
			usleep(500000);
			/*******************************************************/
		}

	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
