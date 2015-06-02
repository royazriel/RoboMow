// -------------------------------------------------------------------------------------------------------------------
//
//  File: main.c -
//
//  Copyright 2011 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Zoran Skrba, March 2012
//
// -------------------------------------------------------------------------------------------------------------------

/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(void);
extern void send_usbmessage(uint8*, int);

#if (DR_DISCOVERY == 0)
#define SOFTWARE_VER_STRING    "Version 2.37 Mx " //
#else
#define SOFTWARE_VER_STRING    "Version 2.37    " //
#endif

#define COM_TIMEOUT_TO_RESET 200

int instance_anchaddr = 0; //0 = 0xDECA020000000001; 1 = 0xDECA020000000002; 2 = 0xDECA020000000003
int dr_mode = 0;
//if instance_mode = TAG_TDOA then the device cannot be selected as anchor
int instance_mode = ANCHOR;
//int instance_mode = TAG;
//int instance_mode = TAG_TDOA;
//int instance_mode = LISTENER;
int paused = 0;
uint32 startTime;
uint64 lastCommunication = 0;


typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        3,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        9,             // preambleCode
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
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


uint8 getanchorpollmask(uint8 switches)
{
	uint8 value = 0;
	switch(switches)
	{
	case 0: //poll just one anchor 0xDECA020000000001
		value = 0x1;
		break;
	case 1: //poll two anchors 0xDECA020000000001 and 0xDECA020000000002
		value = 0x3;
		break;
	case 2: //poll three anchors 0xDECA020000000001, 0xDECA020000000002 and 0xDECA020000000003
		value = 0x7;
		break;
	case 3: //poll all 4 anchors
	default:
		value = 0xf;
		break;
	}

	return value;
}


// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(void)
{
    instanceAddressConfig_t ipc ;

    ipc.forwardToFRAddress = forwardingAddress[0];
    //ipc.anchorAddress = anchorAddressList[instance_anchaddr];
    instance_anchaddr = 0;
    ipc.anchorAddress = anchorAddressList[instance_anchaddr];
    ipc.anchorAddressList = anchorAddressList;
    ipc.anchorListSize = ANCHOR_LIST_SIZE ; //max 4 for this application

    // anchor poll mask is set by switches 7 and 8
    ipc.anchorPollMask = getanchorpollmask(0);

    ipc.sendReport = 1 ;  //1 => anchor sends TOF report to tag
    //ipc.sendReport = 2 ;  //2 => anchor sends TOF report to listener (see forwardingAddress above)

    instancesetaddresses(&ipc);
}
#endif

uint32 inittestapplication(void);

// Restart and re-configure
void restartinstance(void)
{
    instance_close() ;                          //shut down instance, PHY, SPI close, etc.

    spi_peripheral_init();                      //re initialise SPI...

    inittestapplication() ;                     //re-initialise instance/device
} // end restartinstance()



int decarangingmode(void)
{
    int mode = 0;
#ifdef DW1000_EVK
    if(is_switch_on(TA_SW1_5))
    {
        mode = 1;
    }

    if(is_switch_on(TA_SW1_6))
    {
        mode = mode + 2;
    }
#if (DR_DISCOVERY == 1)
    if(is_switch_on(TA_SW1_7))
    {
        mode = mode + 4;
    }
#endif
#endif
    return mode;
}

uint32 inittestapplication(void)
{
    unsigned int devID ;
    instanceConfig_t instConfig;
    char buf[100];
    int i , result;

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_2);  //max SPI before PLLs configured is ~4M

    i = 10;

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
    sprintf(buf,"dev id %x", devID);
    printUSART(buf); //send some data
    if(DWT_DEVICE_ID != devID) //if the read of devide ID fails, the DW1000 could be asleep
    {
        port_SPIx_clear_chip_select();  //CS low
        Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
        port_SPIx_set_chip_select();  //CS high
        Sleep(7);
        devID = instancereaddeviceid() ;
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

    SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    devID = instancereaddeviceid() ;
    sprintf(buf,"dev id %x", devID);
    printUSART(buf); //send some data
    if (DWT_DEVICE_ID != devID)   // Means it is NOT MP device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }


	instance_mode = ANCHOR;
#if (DR_DISCOVERY == 1)
	led_on(LED_PC6);
#else
	if(anchorAddressList[instance_anchaddr] & 0x1)
	{
		led_on(LED_PC8);
		led_off(LED_PC9);
	}
	if(anchorAddressList[instance_anchaddr] & 0x2)
	{
		led_on(LED_PC8);
		led_on(LED_PC9);
	}
#endif

    instancesetrole(instance_mode) ;     // Set this instance role
	instance_init_s(instance_mode);
	dr_mode = decarangingmode();

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    sprintf(buf,"ch selection: %d ch=%d preamble=%d prf=%d pacSize=%d nsSFD=%d datarate=%d preambleLength=%d",
        		dr_mode, instConfig.channelNumber, instConfig.preambleCode, instConfig.pulseRepFreq,
    			instConfig.pacSize, instConfig.nsSFD, instConfig.dataRate, instConfig.preambleLen);
    printUSART(buf);
    instance_config(&instConfig) ;                  // Set operating channel etc

#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time
#endif

	// NOTE: this is the delay between receiving the blink and sending the ranging init message
	// The anchor ranging init response delay has to match the delay the tag expects
	// the tag will then use the ranging response delay as specified in the ranging init message
	// use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times != 150ms)
	instancesetblinkreplydelay(FIXED_REPLY_DELAY);

#if (DR_DISCOVERY == 1)
        //set the default response delays
        instancesetreplydelay(FIXED_REPLY_DELAY, 0);
#else
        //set the default response delays
        instancesetreplydelay(FIXED_REPLY_DELAY_MULTI, 0);
#endif

    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void process_dwRSTn_irq(void)
{
    instance_notify_DW1000_inIDLE(1);
}

void process_deca_irq(void)
{
    do{

        instance_process_irq(0);

    }while(port_CheckEXT_IRQ() == 1); //while IRS line active (ARM can only do edge sensitive interrupts)

}

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

/*
 * @brief switch_mask  - bitmask of testing switches (currently 7 switches)
 * 		  switchbuff[] - switch name to test
 * 		  *switch_fn[]() - corresponded to switch test function
**/
#define switch_mask   (0x7F)

#ifdef DW1000_EVK
const uint8	switchbuf[]={0, TA_SW1_3 , TA_SW1_4 , TA_SW1_5 , TA_SW1_6 , TA_SW1_7 , TA_SW1_8 };
const int (* switch_fn[])(uint16)={ &is_button_low, \
								&is_switch_on, &is_switch_on, &is_switch_on,\
							    &is_switch_on, &is_switch_on, &is_switch_on };
#endif

/*
 * @fn test_application_run
 * @brief	test application for production pre-test procedure
**/
void test_application_run(void)
{
    char  dataseq[2][40];
    uint8 j, switchStateOn, switchStateOff;

    }

/*
 * @fn 		main()
 * @brief	main entry point
**/
int main(void)
{
    int i = 0;
    int toggle = 1;
    int ranging = 0;
    uint8 dataseq[40];
	double range_result = 0;
	double avg_result = 0;

	static uint32 periodicLedTime;

    peripherals_init();

    spi_peripheral_init();

	lastCommunication = portGetTickCount();

    printUSART( "DECAWAVE");
    printUSART( SOFTWARE_VER_STRING );
	GPIO_SetBits(GPIOA,DECAWAVE_POWER);
	Sleep(100);
	GPIO_ResetBits(GPIOA,DECAWAVE_POWER);
	Sleep(1000);
	GPIO_SetBits(GPIOA,DECAWAVE_POWER);
    /* Enable the peripheral clock of GPIOA */
     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
#if 0
    //Enable GPIO used for User button
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit( &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

    port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device

    printUSART("DECAWAVE  RANGE");

    led_off(LED_ALL);

    startTime = portGetTickCount();
	if(inittestapplication() == (uint32)-1)
	{
		led_on(LED_ALL); //to display error....
		printUSART("  INIT FAIL ");
	}

	//sleep for 5 seconds displaying "Decawave"
	i=5;
	while(i--)
	{
		if (i & 1) led_off(LED_ALL);
		else    led_on(LED_ALL);
	
		Sleep(200);
	}
	i = 0;
	led_off(LED_ALL);

	instance_mode = ANCHOR;
#if (DR_DISCOVERY == 1)
            led_on(LED_PC6);
#else
	if(anchorAddressList[instance_anchaddr] & 0x1)
	{
		led_on(LED_PC8);
		led_off(LED_PC9);
	}
	if(anchorAddressList[instance_anchaddr] & 0x2)
	{
		led_on(LED_PC8);
		led_on(LED_PC9);
	}
#endif

	if(instance_mode == TAG)
	{
		//if TA_SW1_2 is on use fast ranging (fast 2wr)
		if( 0 == S1_SWITCH_ON)
		{
			printUSART(" Fast Tag  Ranging ");
		}
		else
		{
			printUSART("TAG BLINK");
		}
	}
	else
	{
		printUSART("AWAITING POLL ");
	}

    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

	if( GPIO_ReadInputDataBit( ANCHOR_ID_SELECTION_PORT,ANCHOR_ID_SELECTION))
	{
		instance_anchaddr = 1;
		if(instance_anchaddr==ANCHOR_LIST_SIZE) instance_anchaddr = 0;
		restartinstance();
	}
	sprintf( dataseq, "ANCHOR ID IS %d", instance_anchaddr + 1);
	printUSART(dataseq);

	WDT_Configuration();

	periodicLedTime = portGetTickCnt();
    // main loop
    while(1)
    {
#if 0
    	if( GPIO_ReadInputDataBit( USER_BUTTON_PORT,USER_BUTTON))
    	{
    		instance_anchaddr++;
    		if(instance_anchaddr==ANCHOR_LIST_SIZE) instance_anchaddr = 0;
    		sprintf( dataseq, "ANCHOR ID CHANGED to %d", instance_anchaddr);
    		printUSART(dataseq);
    		restartinstance();
    		sleep(1);
    	}
#endif
        instance_run();
        IWDG_ReloadCounter();
        if( portGetTickCnt()- periodicLedTime > 1000)
        {
        	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))
        		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        	else
        		GPIO_SetBits(GPIOC, GPIO_Pin_9);
        	periodicLedTime = portGetTickCnt();
        }

		if( portGetTickCount() >  lastCommunication + COM_TIMEOUT_TO_RESET )
		{
			printUSART("no Communication resetting");
			restartinstance();
			lastCommunication = portGetTickCount();
		}

        if(instancenewrange())
        {
        	ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist();
#if (DR_DISCOVERY == 0)
            if(instance_mode == ANCHOR)
#endif
            avg_result = instance_get_adist();
            sprintf((char*)&dataseq[0], "LAST: %4.2f m", range_result);
            printUSART(dataseq); //send some data

#if (DR_DISCOVERY == 0)
            if(instance_mode == ANCHOR)
                sprintf((char*)&dataseq[0], "AVG8: %4.2f m", avg_result);
            else
                sprintf((char*)&dataseq[0], "%llx", instance_get_anchaddr());
#else
            sprintf((char*)&dataseq[0], "AVG8: %4.2f m", avg_result);
#endif
            printUSART(dataseq); //send some data
        }

        if(ranging == 0)
        {
        	if(instance_mode != ANCHOR)
        	{
        		if(instancesleeping())
				{
					dataseq[0] = 0x2 ;  //return cursor home
					writetoLCD( 1, 0,  dataseq);
					if(toggle)
					{
						toggle = 0;
						printUSART("AWAITING RESPONSE");
					}
					else
					{
						toggle = 1;
						printUSART("   TAG BLINK    ");
						sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
						printUSART(dataseq);
					}

				}

        		if(instanceanchorwaiting() == 2)
				{
        			ranging = 1;
					memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
					printUSART(dataseq); //send some data
					sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
					printUSART(dataseq);
				}
        	}
        	else
        	{
				if(instanceanchorwaiting())
				{
					toggle+=2;

					if(toggle > 300000)
					{
						if(toggle & 0x1)
						{
							toggle = 0;
							printUSART("AWAITING POLL");
						}
						else
						{
							toggle = 1;
	#if (DR_DISCOVERY == 1)
							memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
	#else
							memcpy(&dataseq[0], (const uint8 *) " NON DISCOVERY  ", 16);
#endif
							//printUSART( dataseq); //send some data
							//sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
							//printUSART( dataseq); //send some data
						}
					}

				}
				else if(instanceanchorwaiting() == 2)
				{
					memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
					printUSART( dataseq); //send some data
					sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
					printUSART( dataseq); //send some data
				}
        	}
        }
    }
    return 0;
}



