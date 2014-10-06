/*
 * main.c
 *
 *  Created on: Oct 3, 2014
 *      Author: roy
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#include "deca_device_api.h"
#include "compiler.h"
#include "spiDriver.h"

#include "instance.h"

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
					//mode 1 - S1: 7 off, 6 off, 5 off
					{
						2,              // channel
						DWT_PRF_16M,    // prf
						DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,	// preambleLength
                        DWT_PAC32,		// pacSize
                        1		// non-standard SFD
                    },
                    //mode 2
					{
						2,              // channel
						DWT_PRF_16M,    // prf
						DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,	// preambleLength
                        DWT_PAC8,		// pacSize
                        0		// non-standard SFD
                    },
                    //mode 3
					{
						2,              // channel
						DWT_PRF_64M,    // prf
						DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,	// preambleLength
                        DWT_PAC32,		// pacSize
                        1		// non-standard SFD
                    },
                    //mode 4
					{
						2,              // channel
						DWT_PRF_64M,    // prf
						DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,	// preambleLength
                        DWT_PAC8,		// pacSize
                        0		// non-standard SFD
                    },
                    //mode 5
					{
						5,              // channel
						DWT_PRF_16M,    // prf
						DWT_BR_110K,    // datarate
						3,             // preambleCode
						DWT_PLEN_1024,	// preambleLength
						DWT_PAC32,		// pacSize
						1		// non-standard SFD
					},
					//mode 6
					{
						5,              // channel
						DWT_PRF_16M,    // prf
						DWT_BR_6M8,    // datarate
						3,             // preambleCode
						DWT_PLEN_128,	// preambleLength
						DWT_PAC8,		// pacSize
						0		// non-standard SFD
					},
					//mode 7
					{
						5,              // channel
						DWT_PRF_64M,    // prf
						DWT_BR_110K,    // datarate
						9,             // preambleCode
						DWT_PLEN_1024,	// preambleLength
						DWT_PAC32,		// pacSize
						1		// non-standard SFD
					},
					//mode 8
					{
						5,              // channel
						DWT_PRF_64M,    // prf
						DWT_BR_6M8,    // datarate
						9,             // preambleCode
						DWT_PLEN_128,	// preambleLength
						DWT_PAC8,		// pacSize
						0		// non-standard SFD
					}
};

#if (DR_DISCOVERY == 0)
//Tag address list
uint64 tagAddressList[3] =
{
	 0xDECA010000001001,         // First tag
     0xDECA010000000002,         // Second tag
     0xDECA010000000003          // Third tag
} ;

//Anchor address list
uint64 anchorAddressList[ANCHOR_LIST_SIZE] =
{
     0xDECA020000000001 ,       // First anchor
     0xDECA020000000002 ,       // Second anchor
     0xDECA020000000003 ,       // Third anchor
     0xDECA020000000004         // Fourth anchor
} ;

//ToF Report Forwarding Address
uint64 forwardingAddress[1] =
{
	 0xDECA030000000001
} ;
// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(void)
{
    instanceAddressConfig_t ipc ;

    ipc.forwardToFRAddress = forwardingAddress[0];
    ipc.anchorAddress = anchorAddressList[instance_anchaddr];
    ipc.anchorAddressList = anchorAddressList;
    ipc.anchorListSize = ANCHOR_LIST_SIZE ;
    ipc.anchorPollMask = 0x1; //0x7;              // anchor poll mask

    ipc.sendReport = 1 ;  //1 => anchor sends TOF report to tag
    //ipc.sendReport = 2 ;  //2 => anchor sends TOF report to listener

    instancesetaddresses(&ipc);
}
#endif


static SpiConfig s_spi;
int instance_anchaddr = 0;
int dr_mode = 0;
int instance_mode = ANCHOR;

uint32 inittestapplication( int mode, int replyDelayState, int blinkDelayState );


void reset_DW1000(void)
{
#ifdef ST_MC
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);
#endif
}


int decarangingmode(void)
{
	int mode = 0;  //ch2 110kb/s

#ifdef ST_MC
	if(is_switch_on(TA_SW1_5))
	{
		mode = 1;
	}

	if(is_switch_on(TA_SW1_6))
	{
		mode = mode + 2;
	}

	if(is_switch_on(TA_SW1_7))
	{
		mode = mode + 4;
	}
#endif
	return mode;
}
// Restart and re-configure
void restartinstance(void)
{
	closespi();
	openspi( &s_spi );
    instance_close() ;                          //shut down instance, PHY, SPI close, etc.

    inittestapplication(instance_mode, 1, 1 ) ;                     //re-initialise instance/device
} // end restartinstance()

//TODO check if supported
#ifdef ST_MC
void process_deca_irq(void)
{
    do{

    	instance_process_irq(0);

    }while(port_CheckIRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}
#endif

void pabort(const char *s)
{
	perror(s);
	abort();
}

void hexDump( const uint8 * buf, uint32 length )
{
   char format[MAX_PATH];
   unsigned int i = 0;
   int size = length;
   int indexSize = 0;
   int rawLength = SCREEN_RAW_LENGTH;
   while ( size > 0 ) {
       indexSize++;
       size = size / 10;
   }
   sprintf( format, " [%%0%dd]:0x%%02x", indexSize );
   rawLength = rawLength/strlen( format );
   for ( ; i < length; i++ ) {
       printf( format, i, buf[i] );
       if ( ( ( i + 1 ) % rawLength ) == 0 ) {
           printf( "\n");
       }
   }
   printf( "\n" );
}

int main(int argc, char *argv[])
{
	int i = 0;
	uint32 status = 0;
	uint32 states = 0;

	SpiConfig spi;

	s_spi.bits = 8;
	s_spi.speed = 4500000;
	s_spi.delay = 0;
	s_spi.mode = 0;
	s_spi.toggle_cs = 1;
	s_spi.device = "/dev/spidev0.0";

	openspi( &s_spi );

	PINFO( "spi initialized");

    if(inittestapplication(instance_mode, 1, 1) == (uint32)-1)
	    {
	        PERROR("init failed");
	        return 0; //error
	    }

		sleep(5);
		i = 0;


		if(instance_mode == TAG)
		{
			PINFO( "AWAITING RESPONSE");
		}
		else
		{
			PINFO( "AWAITING POLL");
		}
#ifdef ST_MC
		port_EnableIRQ(); //enable ScenSor IRQ before starting
#endif
    // main loop
    while(1)
    {
    	//DEBUG
#if 1
    	if(i==1)
		{
			states = 0;
			uint32 addr = 0;
			uint32 offset = 0;
			uint32 value = 0;

			status = dwt_read32bitoffsetreg(0xF, 0x0);
			states = dwt_read32bitoffsetreg(0x19, 0x0);

			states = dwt_read32bitoffsetreg(0x19, 0x1);

			dwt_write32bitoffsetreg(addr, offset, value);
		}
#endif

		//system_services();
		instance_run();

		if(instancenewrange())
		{
			//send the new range information to Location Engine
			double range_result = 0;
			double avg_result = 0;

			range_result = instance_get_idist();
			avg_result = instance_get_adist();

			PINFO("LAST: %4.2f m AVG8: %4.2f m", range_result, avg_result);
		}
    }
}


uint32 inittestapplication( int mode, int replyDelayState, int blinkDelayState )
{
    uint32 devID ;
    instanceConfig_t instConfig;
    int i , result;
#ifdef ST_MC
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_16);  //max SPI before PLLs configured is ~4M
#endif
    i = 10;

	//this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of devide ID fails, the DW1000 could be asleep
    {
    	PERROR("dev id problem");
#ifdef ST_MC
    	port_SPIx_clear_chip_select();	//CS low
    	Sleep(1);	//200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    	port_SPIx_set_chip_select();  //CS high
    	Sleep(7);
#endif-    	devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
    	if(DWT_DEVICE_ID != devID)
    		return(-1) ;
    	//clear the sleep bit - so that after the hard reset below the DW does not go into sleep
    	dwt_softreset();
    }

	//reset the DW1000 by driving the RSTn line low
	reset_DW1000();

    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred
#ifdef ST_MC
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
#endif
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        PERROR("dev id problem");
		return(-1) ;
    }

    PINFO("DEVID = 0x%x", devID);

	if(mode == 0)
	{
		instance_mode = TAG;
		PINFO( "MODE = TAG");
	}
	else
	{
		instance_mode = ANCHOR;
		PINFO( "MODE = ANCHOR");
	}

    instancesetrole(instance_mode) ;     // Set this instance role

    dr_mode = decarangingmode();

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;

    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    instance_config(&instConfig) ;                  // Set operating channel etc

#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
#endif
    instancesettagsleepdelay(400); //set the Tag sleep time

    if( replyDelayState == 0 )
    {
    	instancesetreplydelay(FIXED_REPLY_DELAY);
    	PINFO( "REPLY DELAY = FIXED");
    }
    else
    {
    	instancesetreplydelay(FIXED_LONG_REPLY_DELAY);
    	PINFO( "REPLY DELAY = LONG");
    }
    //use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times)
    if(blinkDelayState == 1 )
    	instancesetblinkreplydelay(FIXED_LONG_BLINK_RESPONSE_DELAY);

    return devID;
}

