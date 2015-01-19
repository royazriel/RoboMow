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
#include "state_machine.h"
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

 void flashUnitTest()
 {
	 uint8_t string1[10] = "hello\0";
	 uint8_t string2[10] = "bye\0";
	 uint8_t string3[10] = {0,};

	 FlashSpiEraseSector(0);
	 FlashSpiWriteBuffer(string1, 0, 6);
	 UsartPrintf(string1);
	 UsartPrintf("\r\n");
	 FlashSpiReadBuffer(string3,0,6);
	 UsartPrintf(string3);
	 UsartPrintf("\r\n");
	 if(strncmp(string1,string3,6)==0)
	 {
		 UsartPrintf("erasing writing reading to first sector success\r\n");
	 }


	 FlashSpiEraseSector(SPI_FLASH_SECTOR_SIZE* 127); //erase last sector
	 FlashSpiWriteBuffer(string2, 127*SPI_FLASH_SECTOR_SIZE, 4);
	 UsartPrintf(string2);
	 UsartPrintf("\r\n");
	 FlashSpiReadBuffer(string3,127*SPI_FLASH_SECTOR_SIZE,4);
	 UsartPrintf(string3);
	 UsartPrintf("\r\n");
	 if(strncmp(string2,string3,4)==0)
	 {
		 UsartPrintf("erasing writing reading to last sector success\r\n");
	 }
 }

int
main(int argc, char* argv[])
{
	uint8_t string[SPI_FLASH_SECTOR_SIZE] = {0,};
	int count = 0;

	InitGpio();
	InitSystemTick();
	InitUsartDebug();
	InitI2c();
	InitSpi();
	InitPwm();
	InitEncoders();
	InitAdc();
	InitDac();

	UsartPrintf("\033cNUNU Project!!\r\n");
//	LIS3DH_Configure();
//
//	while(1)
//	{
//		StateMachineHandleStates();
//	}
	while(1)
	{
		if(PB1_STATE)
		{
			WavPlayerLoad(0);
			WavPlayerPlaySound(0);
//			FlashSpiReadBuffer(string,0,SPI_FLASH_SECTOR_SIZE);
//			for(int i=0;i<SPI_FLASH_SECTOR_SIZE; i++)
//			{
//				UsartPrintf("%x ",string[i]);
//			}
//			UsartPrintf("\r\n");
//			UsartPrintf("before\r\n");
//			WavPlayerPlaySound(0);
//			UsartPrintf("after\r\n");
			usleep(500000);
//			count++;
//			switch(count)
//			{
//				case 2:
//					flashUnitTest();
//					break;
//				case 1:
//					FlashSpiReadBuffer(string,0,6);
//					UsartPrintf(string);
//					UsartPrintf("\r\n");
//					break;
//			}
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
