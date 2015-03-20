//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "compiler.h"
#include "diag/Trace.h"
#include "hardware_config.h"
#include "lis3dh_driver.h"
#include "motor_control.h"
#include "flash_spi.h"
#include "wav_player.h"
#include "state_machine.h"
#include "tsl_types.h"
#include "tsl_user.h"

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


void MyLinRots_ErrorStateProcess(void)
{
  /* Add here your own processing when a sensor is in Error state */
  TSL_linrot_SetStateOff();
}

void MyLinRots_OffStateProcess(void)
{
  /* Add here your own processing when a sensor is in Off state */
}

int
main(int argc, char* argv[])
{
	//uint8_t string[SPI_FLASH_SECTOR_SIZE] = {0,};
	int count = 0;
	float temp, press;

	InitGpio();
	InitSystemTick();
	InitUsartDebug();
	InitI2c();
	InitSpi();
	InitPwm();
	InitEncoders();
	InitAdc();
	InitDac();



	 /* Init STMTouch driver */
	//TSL_user_Init();

	UsartPrintf("\033cNUNU Project!!\r\n");
	UsartPrintf("starting pressure sensor\r\n");
	LPS25H_Init();
	UsartPrintf("id of pressure sensor is: 0x%x\r\n",LPS25H_ReadID());

	while(1)
	{
		if(PB1_STATE)
		{
			LPS25H_GetPressure(&press);
			LPS25H_GetTemperature(&temp);
			UsartPrintf("pressure %2.2f temp %2.2f\r\n", press,temp);

#if 0
		    if (TSL_user_Action() == TSL_STATUS_OK)
		    {

				if ((MyLinRots[0].p_Data->StateId == TSL_STATEID_DETECT) ||
					(MyLinRots[0].p_Data->StateId == TSL_STATEID_DEB_RELEASE_DETECT))
				{
				  if (MyLinRots[0].p_Data->Position > 0)
				  {
				  }
				}
		    }
#endif
			WavPlayerPlaySound(count);
			count++;
			if(count==20)count =0;
			usleep(500000);
		}
	}




#if 0
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

				ErrorCode err = WavPlayerLoad( 0 );
				if ( err != Valid_WAVE_File)
				{
					UsartPrintf("file not valid %d\r\n", err );
				}
				WavPlayerPlaySound(0);
//				while(1)
//				{
//					WavPlayerLoadNextBuffer();
//				}

				//UsartPrintf("%d %d %d %d %d %d\r\n",GetAdcData()[0],GetAdcData()[1],GetAdcData()[2],GetAdcData()[3],GetAdcData()[4],GetAdcData()[5]);

				usleep(500000);
			}
#endif
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
