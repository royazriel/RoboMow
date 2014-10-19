// -------------------------------------------------------------------------------------------------------------------
//
//  File: instance_common.c - application level message exchange for ranging demo
//
//  Copyright 2014 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Zoran Skrba, Billy Verso, December 2014
//
// -------------------------------------------------------------------------------------------------------------------

#include "compiler.h"
#include "deca_device_api.h"
#include "deca_spinit.h"
#include "spiDriver.h"
#include "deca_util.h"
#include "instance.h"


//uint32 ptime = 0;
extern uint32 getmstime();
extern uint32 startTime;
// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
extern instance_data_t instance_data[NUM_INST] ;
// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------
//
// function to select the destination address (e.g. the address of the next anchor to poll)
//
// -------------------------------------------------------------------------------------------------------------------
//
instance_data_t* getInstanceData(int index)
{
	return &instance_data[index];
}

int instaddtagtolist(instance_data_t *inst, uint8 *tagAddr)
{
    uint8 i;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    inst->blinkRXcount++ ;

    //add the new Tag to the list, if not already there and there is space
    for(i=0; i<TAG_LIST_SIZE; i++)
    {
        if(memcmp(&inst->tagList[i][0], &tagAddr[0], 8) != 0)
        {
            if(memcmp(&inst->tagList[i][0], &blank[0], 8) == 0) //blank entry
            {
            	PINFO("*****************adding tag to list of found tags*********************************");
                memcpy(&inst->tagList[i][0], &tagAddr[0], 8) ;
                inst->tagListLen = i + 1 ;
                break;
            }
        }
        else
        {
            break; //we already have this Tag in the list
        }
    }

    return 0;
}


// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else

void instcleartaglist(void)
{
	int instance = 0 ;

	memset((uint8 *) &instance_data[instance].tagList[0], 0, TAG_LIST_SIZE * sizeof(instance_data[instance].tagList[0]));
    instance_data[instance].tagListLen = 0 ;
    instance_data[instance].blinkRXcount = 0 ;
	instance_data[instance].tagToRangeWith = TAG_LIST_SIZE;
}

int instgettaglistlen(void)
{
	int instance = 0 ;
    return instance_data[instance].tagListLen ;
}

uint8* instgettaglist(int i)
{
	int instance = 0 ;
	
	if(i<instance_data[instance].tagListLen)
		return &instance_data[instance].tagList[i][0] ;
	else
		return NULL;
}

uint8 instgettaglistlength(void)
{
	int instance = 0 ;
    return instance_data[instance].tagListLen ;
}

void instsettagtorangewith(int tagID)
{
	int instance = 0 ;

	instance_data[instance].tagToRangeWith = tagID ;
}

uint32 instgetblinkrxcount(void)
{
	int instance = 0 ;
    return instance_data[instance].blinkRXcount ;
}

int instancegetRxCount(void)
{
	return instance_data[0].rxmsgcount;
}

int instancegetRngCount(void)
{
	return instance_data[0].longTermRangeCount;
}

int instancesetdeepsleep(int inst_mode)
{
	if(instance_data[0].testAppState == TA_PAUSED)
	{
		// assume instance 0, for this
		instance_data[0].tagDeepSleep =  inst_mode;                   // set the role
		//this is platform dependent - only program if DW EVK/EVB - this function enables GPIOs 2 and 3 which are connected to LED3 and LED4 on EVB1000
		dwt_setleds(1);

		//MP bug - TX antenna delay needs reprogramming as it is not preserved
		dwt_settxantennadelay(instance_data[0].txantennaDelay) ;

		dwt_entersleepaftertx(0);

		dwt_configeventcounters(1); //enable and clear

		return 0; 
	}
	else
		return 1;
}

int instancegetdeepsleep(void)
{
    return instance_data[0].tagDeepSleep;
}



// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  inst_mode;                   // set the role
}

int instancegetrole(void)
{
    return instance_data[0].mode;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;
	instance_data[instance].responseTimeouts = 0 ;

    instance_data[instance].frame_sn = 0;
    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

    instance_data[instance].idistmax = 0;
    instance_data[instance].idistmin = 1000;

    instance_data[instance].payloadCounter = 0 ;

    dwt_configeventcounters(1); //enable and clear

    instance_data[instance].frame_sn = 0;
    instance_data[instance].lastReportSN = 0xff;
#if DECA_LOG_ENABLE==1
	clearreportTOF(&instance_data[instance].statusrep);
#endif

	instance_data[instance].txmsgcount = 0;
	instance_data[instance].rxmsgcount = 0;
	instance_data[instance].lateTX = 0;
	instance_data[instance].lateRX = 0;

} // end instanceclearcounts()

// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error

int instance_init(accBuff_t *buf) 
{
    int instance = 0 ;
    int result, i, j ;
	//uint16 temp = 0;

    instance_data[instance].mode = ANCHOR ;                                // assume listener,
    instance_data[instance].testAppState = TA_INIT ;
    instance_data[instance].prevStateDebug = TA_LAST_STATE;

    instance_data[instance].anchorListIndex = 0 ;
    instance_data[instance].instToSleep = 0;

    instance_data[instance].sentSN = 0;
	
	instance_data[instance].tofstdevindex = 0;
	instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;

	for (i = 0 ; i < NUM_STAT_LINES ; i++)
    {
        for (j = 0 ; j < STAT_LINE_LENGTH ; j++)
        {
            instance_data[instance].statusrep.scr[i][j] = '.' ;
        }
    }
	for (i = 0 ; i < NUM_MSGBUFF_LINES ; i++)
    {
        for (j = 0 ; j < STAT_LINE_LONG_LENGTH ; j++)
        {
            instance_data[instance].statusrep.lmsg_scrbuf[i][j] = ' ' ;
        }
    }
    instance_data[instance].statusrep.lmsg_last_index = 0 ;
#if (DECA_ACCUM_LOG_SUPPORT==1)
    clearlogmsgbuffer() ;
#endif


    instance_data[instance].statusrep.changed = 0 ;
    instance_data[instance].last_update = -1 ;           // detect changes to status report

	// Reset the IC (might be needed if not getting here from POWER ON)
    dwt_softreset();

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
	uint16 blinktime  = 0xf; //e.g. blink time to be used for the Tag auto wake up
	{
		double t;
		uint16 lp_osc_cal = dwt_calibratesleepcnt(); //calibrate low power oscillator
		//the lp_osc_cal value is number of XTAL/2 cycles in one cycle of LP OSC
		//to convert into seconds (38.4MHz/2 = 19.2MHz (XTAL/2) => 1/19.2MHz ns)
		//so to get a sleep time of 5s we need to program 5 / period and then >> 12 as the register holds upper 16-bits of 28-bit counter
		t = ((double)5/((double)lp_osc_cal/19.2e6));
		blinktime = (int) t;
		blinktime >>= 12;
					
		dwt_configuresleepcnt(blinktime);//configure sleep time

	}
#endif

	//we can enable any configuration loding from OTP/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM) ;
	
	//temp = dwt_readtempvbat();
	//  temp formula is: 1.13 * reading - 113.0
	// Temperature (�C )= (SAR_LTEMP - (OTP_READ(Vtemp @ 23 �C )) x 1.14) + 23
	//  volt formula is: 0.0057 * reading + 2.3
	// Voltage (volts) = (SAR_LVBAT- (OTP_READ(Vmeas @ 3.3 V )) /173) + 3.3
	//printf("Vbat = %d (0x%02x) %1.2fV\tVtemp = %d  (0x%02x) %2.2fC\n",temp&0xFF,temp&0xFF, ((0.0057 * (temp&0xFF)) + 2.3), (temp>>8)&0xFF,(temp>>8)&0xFF, ((1.13 * ((temp>>8)&0xFF)) - 113.0));
	
	// if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
	// other errors which need to be checked (as they disable receiver) are
	if(getSpiHandle() < 0) //this is as Cheetah does not work with interrupts
	{
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 0);
	}
	else
	{
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
	}
	//this is platform dependant - only program if DW EVK/EVB
    dwt_setleds(2) ;	//configure the GPIOs which control the leds on EVBs

	//this is set though the instance_data[instance].configData.smartPowerEn in the instance_config function
	//dwt_setsmarttxpower(0); //disable smart TX power

	instancesetspibitrate(DW_FAST_SPI); 

    if (DWT_SUCCESS != result)
    {
    	PINFO("dwt_initialise - failed");
    	return (-1) ;	// device initialise has failed
    }

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

    instanceclearcounts() ;

    instance_data[instance].panid = 0xdeca ;

    instance_data[instance].sendTOFR2Tag = NO_TOF_REPORT;

    instance_data[instance].tag2rxReport = 0; //Tag is not expecting the report

	instance_data[instance].tagBlinkSleepTime_ms = BLINK_SLEEP_DELAY ;

	instance_data[instance].newReportSent = 0; //clear the flag
	instance_data[instance].wait4ack = 0;
    instance_data[instance].ackexpected = 0;
	instance_data[instance].stoptimer = 0;
	instance_data[instance].instancetimer_en = 0;

	instance_clearevents();

	instance_data[instance].deviceissleeping = 0;
#if (DECA_ACCUM_LOG_SUPPORT==1)
	instance_data[instance].dwacclogdata.newAccumData = 0;

	instance_data[instance].dwacclogdata.buff.accumData = buf;
#endif
	memset((uint8 *) &instance_data[instance].dwlogdata.diag, 0, sizeof(instance_data[instance].dwlogdata.diag));

	instance_data[instance].dwlogdata.icid = dwt_getpartid();

	instance_data[instance].doublebufferon = DOUBLE_RX_BUFFER;

	if(instance_data[instance].doublebufferon)
		instance_data[instance].rxautoreenable = 1;
	//sample test calibration functions
	//xtalcalibration();
	//powertest();

	dwt_geteui(instance_data[instance].eui64);
	printf("device EUI = %02x%02x%02x%02x%02x%02x%02x%02x\n", instance_data[instance].eui64[7], instance_data[instance].eui64[6], 
								instance_data[instance].eui64[5], instance_data[instance].eui64[4],
								instance_data[instance].eui64[3], instance_data[instance].eui64[2],
								instance_data[instance].eui64[1], instance_data[instance].eui64[0]);

	instance_data[instance].tagDeepSleep = DEEP_SLEEP ;
    return DWT_SUCCESS ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}




// -------------------------------------------------------------------------------------------------------------------
// function to set the antenna delay
//
void instancesetantennadelays(double fdelay)        // delay in nanoseconds
{
	int instance = 0 ;

    uint16 antennaDelay = (uint16)((fdelay / 2.0) * 1e-9 / DWT_TIME_UNITS) ; // half delay figure in each, converting from ns to time units.
	 
	dwt_setrxantennadelay(antennaDelay);
	dwt_settxantennadelay(antennaDelay);

	instance_data[instance].txantennaDelay = antennaDelay ;

} // end instancesetantennadelays()

// -------------------------------------------------------------------------------------------------------------------
// function to read the antenna delay stored in the DW1000 and convert to nano seconds
//
double instancegetantennadelay(int prf)        // returns delay in nanoseconds
{
	//int instance = 0 ;

    uint16 antennaDelay = dwt_readantennadelay(prf);
		
	double totaldelay = antennaDelay * DWT_TIME_UNITS * 1e9; // covert to nanoseconds, this is the total delay
	 
	return totaldelay ;
} // end instancegetantennadelay()


// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
	int use_otpdata =  DWT_LOADANTDLY | DWT_LOADXTALTRIM ;
	uint32 power = 0;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
	instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
	instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = DWT_SFDTOC_DEF ;//config->sfdTO;
	
    //enable gating gain for 6.81Mbps data rate
    if(instance_data[instance].configData.dataRate == DWT_BR_6M8)
    	instance_data[instance].configData.smartPowerEn = 1;
    else
		instance_data[instance].configData.smartPowerEn = 0;

	//configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_otpdata) ; 
	
	instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

	//firstly check if there are calibrated TX power value in the DW1000 OTP
	power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);
	
	if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
	{
		power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
	}
	
	//Configure TX power
	//if smart power is used then the value as read from OTP is used directly
	//if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
	if(instance_data[instance].configData.smartPowerEn == 1)
	{
		instance_data[instance].configTX.power = power;
	}
	else //if the smart power is not used, then the low byte value (repeated) is used for the whole TX power register
	{
		uint8 pow = power & 0xFF ;
		instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
	}
	dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);

	//configure the tx spectrum parameters (power and PG delay)
	dwt_configuretxrf(&instance_data[instance].configTX);

	//check if to use the antenna delay calibration values as read from the OTP
	if((use_otpdata & DWT_LOADANTDLY) == 0)
	{
		instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
		// -------------------------------------------------------------------------------------------------------------------
		// set the antenna delay, we assume that the RX is the same as TX.
		dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
		dwt_settxantennadelay(instance_data[instance].txantennaDelay);
	}
	else
	{
        //get the antenna delay that was read from the OTP calibration area
		instance_data[instance].txantennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;
		
        // if nothing was actually programmed then set a reasonable value anyway
		if (instance_data[instance].txantennaDelay == 0) 
		{
			instance_data[instance].txantennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
			// -------------------------------------------------------------------------------------------------------------------
			// set the antenna delay, we assume that the RX is the same as TX.
			dwt_setrxantennadelay(instance_data[instance].txantennaDelay);
			dwt_settxantennadelay(instance_data[instance].txantennaDelay);
		}


	}

	//config is needed before reply delays are configured
	//instancesetreplydelay(FIXED_REPLY_DELAY, 0);

#if (REG_DUMP == 1)
#define REG_BUF_SIZE    (200*30)
{	
	char regDumpBuffer[REG_BUF_SIZE] ;
	dwt_dumpregisters(regDumpBuffer, REG_BUF_SIZE);

	printf("%s", regDumpBuffer);
}
#endif
}


void inst_processrxtimeout(instance_data_t *inst)
{

	inst->responseTimeouts ++ ;
    inst->rxTimeouts ++ ;
    inst->done = INST_NOT_DONE_YET;

    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
    {
#if (ENABLE_AUTO_ACK == 1)
		if((inst->newReportSent) && (inst->newReportSent < MAX_NUMBER_OF_REPORT_RETRYS)) //no ACK send another report
		{
			inst->testAppState = TA_TXREPORT_WAIT_SEND ;
			//printf("Timeout while waiting for ACK -> send next report\n");
		}	
		else //finished sending reports - wait for next poll message (can also get here if no ACK to a Ranging Init message, just wait for a Poll or next Blink)
#endif
		{
			//only enable receiver when not using double buffering
			inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
			dwt_setrxtimeout(0);
            inst->ackexpected = 0 ; //timeout... so no acks expected anymore
		}
    }
	else //if(inst->mode == TAG)
    {
		// initiate the re-transmission of the poll that was not responded to
		inst->testAppState = TA_TXE_WAIT ; 
#if (DR_DISCOVERY == 1)						
		if((inst->mode == TAG) && (inst->responseTimeouts >= MAX_NUMBER_OF_POLL_RETRYS)) //if no response after sending 20 Polls - go back to blink mode
		{
			inst->mode = TAG_TDOA ;
		}
#endif
		if(inst->mode == TAG)
		{
			inst->nextState = TA_TXPOLL_WAIT_SEND ;
		}
#if (DR_DISCOVERY == 1)
		else //TAG_TDOA
		{
			inst->nextState = TA_TXBLINK_WAIT_SEND ;
		}
#endif
    }

    //timeout - disable the radio (if using SW timeout the rx will not be off)
    //dwt_forcetrxoff() ;
}


void inst_processackmsg(instance_data_t *inst, uint8 seqNum)
{
    if(inst->ackexpected) //used to ignore unexpected ACK frames
    {
		if(seqNum == inst->sentSN) //this is the ACK we expect
		{
			//we got the ACK for the last sent frame
			if(inst->previousState == TA_TXREPORT_WAIT_SEND)
			{
				//we got the ACK for the report, wait for next poll message
				//printf("we got the ACK for the report\n");
				dwt_setrxtimeout(0);
			}
			else if(inst->previousState == TA_TXRANGINGINIT_WAIT_SEND) //the tag ACKed our ranging request
			{
				dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //we are starting ranging - enable the filter....
                inst->frameFilteringEnabled = 1 ;

				//dwt_setrxtimeout(0); - don't need to disable as it is not set in TA_TXRANGINGINIT_WAIT_SEND
			}

			inst->ackexpected = 0 ;
		}
    }
}


void instance_txcallback(const dwt_callback_data_t *txd)
{
	int instance = 0;
	uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
	uint32 temp = 0;
	uint8 txevent = txd->event;
	event_data_t dw_event;

	//printf("tx call back %d %d\n", txevent, instance_data[instance].ackreq);
	if(instance_data[instance].ackreq) //the ACK has been requested in the last RX frame - we got TX event, this means the ACK has been sent
	{
		txevent = DWT_SIG_TX_AA_DONE;
		instance_data[instance].ackreq = 0;
        //add the pending event to the event queue, so that the received frame can be processed
        instance_putevent(instance_getsavedevent());

		//printf("states after auto ACK has been sent %08X %08X\n", dwt_read32bitreg(0x19), dwt_read32bitreg(0x0f));
	}

	if(txevent == DWT_SIG_TX_DONE)
	{
		//uint64 txtimestamp = 0;

		//NOTE - we can only get TX good (done) while here
		//dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);
	    	
		dwt_readtxtimestamp(txTimeStamp) ;
		temp = txTimeStamp[0] + (txTimeStamp[1] << 8) + (txTimeStamp[2] << 16) + (txTimeStamp[3] << 24);
		dw_event.timeStamp = txTimeStamp[4];
	    dw_event.timeStamp <<= 32;
	    dw_event.timeStamp += temp;

		instance_data[instance].stoptimer = 0;

		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_DONE ;

		instance_putevent(dw_event);
			
#if (DEEP_SLEEP == 1)	
		if(instance_data[instance].tagDeepSleep)
			instance_data[instance].txmsgcount++;
#endif
		//printf("TX time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
		//printf("TX Timestamp: %4.15e\n", convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp));
#if (DECA_SUPPORT_SOUNDING==1)
	#if DECA_ACCUM_LOG_SUPPORT==1
		if ((instance_data[instance].dwlogdata.accumLogging == LOG_ALL_ACCUM ) || (instance_data[instance].dwlogdata.accumLogging == LOG_ALL_NOACCUM ))
		{
			uint32 hi32 = dw_event.timeStamp >> 32;
			uint32 low32 = dw_event.timeStamp & 0xffffffff;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"\nTX Frame TimeStamp Raw  = %02X %02X%02X%02X%02X\n",txTimeStamp[4], txTimeStamp[3], txTimeStamp[2], txTimeStamp[1], txTimeStamp[0]) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"   Adding Antenna Delay = %04X %08X\n", hi32, low32 ) ;
			fprintf(instance_data[instance].dwlogdata.accumLogFile,"%02X Tx time = %4.15e\n", instance_data[instance].msg.seqNum, convertdevicetimetosecu(dw_event.timeStamp)) ;
		}
    #endif
#endif
	}
	else if(txevent == DWT_SIG_TX_AA_DONE)
	{
		//auto ACK confirmation
		dw_event.rxLength = 0;
		dw_event.type2 = dw_event.type = DWT_SIG_TX_AA_DONE ;

		instance_putevent(dw_event);

		//printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
	}
}

void instance_rxcallback(const dwt_callback_data_t *rxd)
{
	int instance = 0;
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
	//int blink = 0;
    uint32 rxTSlow32 = 0;
    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	event_data_t dw_event;


	//if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
	{

		//ACK request bit?
		instance_data[instance].ackreq = rxd->aatset;

        //if(instance_data[instance].ackreq) //this indicates that the ACK request has been set
        //{
            //need to hold onto the received frame until the ACK has been sent
            //configuration and sending of ACK frame can also be done here when not using auto-ACK feature.
			//	printf("ACK request\n");
        //}

        rxd_event = DWT_SIG_RX_OKAY;

		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
        switch(rxd->fctrl[0])
	    {
			//blink type frame
		    case 0xC5:
				if(rxd->datalength == 12)
				{
					rxd_event = SIG_RX_BLINK;
				}
				else if(rxd->datalength == 18)//blink with Temperature and Battery level indication
				{
					rxd_event = SIG_RX_BLINK;
				}
				else
					rxd_event = SIG_RX_UNKNOWN;
					break;

			//ACK type frame
			case 0x02:
				if(rxd->datalength == 5)
					rxd_event = SIG_RX_ACK;
			    break;
			
			//data type frames (with/without ACK request) - assume PIDC is on.
			case 0x41:
			case 0x61:
				//read the frame
				if(rxd->datalength > STANDARD_FRAME_SIZE)
					rxd_event = SIG_RX_UNKNOWN;

				//need to check the destination/source address mode
				if((rxd->fctrl[1] & 0xCC) == 0x88) //dest & src short (16 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
				}
				else if((rxd->fctrl[1] & 0xCC) == 0xCC) //dest & src long (64 bits)
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_L; //function code is in first byte after source address
				}
				else //using one short/one long
				{
					fcode_index = FRAME_CRTL_AND_ADDRESS_LS; //function code is in first byte after source address
				}
				break;

			//any other frame types are not supported by this application
			default:
				rxd_event = SIG_RX_UNKNOWN;
				break;
		}

	#if (DEEP_SLEEP == 1)	
		if(instance_data[instance].tagDeepSleep)
			instance_data[instance].rxmsgcount++;
	#endif

		//read rx timestamp
		if((rxd_event == SIG_RX_ACK) || (rxd_event == SIG_RX_BLINK) || (rxd_event == DWT_SIG_RX_OKAY))
		{
			dwt_readrxtimestamp(rxTimeStamp) ;
			rxTSlow32 =  rxTimeStamp[0] + (rxTimeStamp[1] << 8) + (rxTimeStamp[2] << 16) + (rxTimeStamp[3] << 24);
			dw_event.timeStamp = rxTimeStamp[4];
			dw_event.timeStamp <<= 32;
			dw_event.timeStamp += rxTSlow32;	

			dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
			dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			if(dwt_checkoverrun()) //the overrun has occured while we were reading the data - dump the frame/data
			{
				rxd_event = DWT_SIG_RX_ERROR ;
			}
		}
		
		dw_event.type2 = dw_event.type = rxd_event;
		
		//printf("rx call back %d %d FI:%08X %02x%02x event%d\n", rxd_event, instance_data[instance].ackreq, dwt_read32bitreg(0x10), rxd->fctrl[0], rxd->fctrl[1], instance_data[instance].dweventCnt);

		//Process good frames
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			if(instance_data[instance].ackreq == 0) //only notify there is event if no ACK pending
	    		instance_putevent(dw_event);
			else
				instance_saveevent(dw_event);

			if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
#if DECA_LOG_ENABLE==1
			instance_calculatepower();
#endif
	    	instance_data[instance].stoptimer = 1;

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			//printf("RX OK %d %x\n",instance_data[instance].testAppState, instance_data[instance].rxmsg.messageData[FCODE]);
			//printf("RX OK %d ", instance_data[instance].testAppState);
			//printf("RX time %f ecount %d\n",convertdevicetimetosecu(dw_event.timeStamp), instance_data[instance].dweventCnt);

		}
		else if (rxd_event == SIG_RX_ACK)
		{
			//printf("RX ACK %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].dweventCnt);
			instance_putevent(dw_event);
			if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
#if DECA_LOG_ENABLE==1
			instance_calculatepower();
#endif
		}
		else if (rxd_event == SIG_RX_BLINK)
		{
			instance_putevent(dw_event);
			if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window
#if DECA_LOG_ENABLE==1
			instance_calculatepower();
#endif
			//printf("RX BLINK %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].dweventCnt);

			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, 0xC5, dw_event.msgu.rxblinkmsg.seqNum, &dw_event);
			#endif

		}
		//PINFO("instance_data[instance].mode=%d",instance_data[instance].mode);
		if(instance_data[instance].mode == LISTENER) //print out the message bytes when in Listener mode
		{
			int i;
			uint8 buffer[1024];
			dwt_readrxdata(buffer, rxd->datalength, 0);  // Read Data Frame
			buffer[1023] = 0;
#if DECA_LOG_ENABLE==1
			instancelogrxdata(&instance_data[instance], buffer, rxd->datalength);
#endif

			printf("RX data(%d): ", rxd->datalength);
			for(i=0; i<rxd->datalength; i++)
			{
				printf("%02x", buffer[i]);
			}
			printf("\n");
		}

		if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx
		{
			if(rxd->dblbuff == 0) instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window

			dwt_readdignostics(&instance_data[instance].dwlogdata.diag);
#if DECA_LOG_ENABLE==1
			instance_calculatepower();
#endif
			#if DECA_LOG_ENABLE==1
			#if DECA_KEEP_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = DWT_SIG_RX_NOERR ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(DWT_SIG_RX_NOERR, dw_event.msgu.frame[fcode_index], dw_event.msgu.frame[2], &dw_event);
			#endif

			if(instance_data[instance].doublebufferon == 0)
				instancerxon(&instance_data[instance], 0, 0); //immediate enable
		}
	}
	else if (rxd->event == DWT_SIG_RX_TIMEOUT)
	{
		dw_event.type2 = dw_event.type = DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;

		instance_putevent(dw_event);
		//printf("RX timeout while in %d\n", instance_data[instance].testAppState);
	}	
	else //assume other events are errors
	{
		//printf("RX error %d \n", instance_data[instance].testAppState);
		if(instance_data[instance].rxautoreenable == 0)
		{
			//re-enable the receiver
#if (DECA_BADF_ACCUMULATOR == 1)
			instance_readaccumulatordata();

			dwt_readdignostics(&instance_data[instance].dwlogdata.diag);

			instance_calculatepower();
#endif
#if (DECA_ERROR_LOGGING == 1)
			#if DECA_LOG_ENABLE==1
			#if DECA_BADF_ACCUMULATOR==1
			{
				instance_data[instance].dwacclogdata.newAccumData = 1 ;
				instance_data[instance].dwacclogdata.erroredFrame = rxd->event ;	//no error
				processSoundingData();
			}
			#endif
				logSoundingData(rxd->event, 0, 0, &dw_event);
			#endif
#endif

			//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
			if((instance_data[instance].mode == TAG) || (instance_data[instance].mode == TAG_TDOA))
			{
				dw_event.type = DWT_SIG_RX_TIMEOUT;
				dw_event.type2 = 0x40 | DWT_SIG_RX_TIMEOUT;
				dw_event.rxLength = 0;

				instance_putevent(dw_event);
			}
			else
			{
				instancerxon(&instance_data[instance], 0, 0); //immediate enable if anchor or listener
			}

		}
	}
}


int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_saveevent(event_data_t newevent)
{
	int instance = 0;

	instance_data[instance].saved_dwevent = newevent;
}

event_data_t instance_getsavedevent(void)
{
	int instance = 0;

	return instance_data[instance].saved_dwevent;
}

void instance_putevent(event_data_t newevent)
{
	int instance = 0;
	uint8 etype = newevent.type;
	newevent.type = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventCntIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventCntIn].type = etype;

	instance_data[instance].dweventCntIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventCntIn)
		instance_data[instance].dweventCntIn = 0;

	//printf("put %d - in %d out %d \n", etype, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut);
}

event_data_t instance_getevent(void)
{
	int instance = 0;

	event_data_t dw_event = instance_data[instance].dwevent[instance_data[instance].dweventCntOut]; //this holds any TX/RX events - at the moment this is an array of 2 but should be changed to a queue

	if(instance_data[instance].dwevent[instance_data[instance].dweventCntOut].type == 0) //exit with "no event"
	{
		instance_data[instance].dwevent[instance_data[instance].dweventCntOut].type2 = 0;
		return dw_event;
	}

	instance_data[instance].dwevent[instance_data[instance].dweventCntOut++].type = 0; //clear the event
	
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventCntOut) //wrap the counter
		instance_data[instance].dweventCntOut = 0;
	
	instance_data[instance].dweventPeek = instance_data[instance].dweventCntOut; //set the new peek value

	//if(dw_event.type) printf("get %d - in %d out %d \n", dw_event.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut);

	return dw_event;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventCntIn = 0;
	instance_data[instance].dweventCntOut = 0;
	instance_data[instance].dweventPeek = 0;
}

int instance_run(uint32 time)
{
    int instance = 0 ;
	int done = INST_NOT_DONE_YET;
    int update ;
	int message = 0 ;

	//if(instance_data[instance].pauseRequest == 0) 
    //{
		dwt_deviceentcnts_t itemp = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		if(!instance_data[instance].deviceissleeping) //if sleeping - don't access SPI
		{
			instancereadevents(&itemp);
		}

#if (DEEP_SLEEP == 1)	
		if(instance_data[instance].tagDeepSleep)
		{
			if((instance_data[instance].mode == TAG) || (instance_data[instance].mode == TAG_TDOA))
			{
				itemp.CRCG = instance_data[instance].rxmsgcount ;
				itemp.TXF = instance_data[instance].txmsgcount ;
			}
		}
		else
		{
			instance_data[instance].rxmsgcount = itemp.CRCG;
			instance_data[instance].txmsgcount = itemp.TXF;
		}
#else
		instance_data[instance].rxmsgcount = itemp.CRCG;
#endif

		//ptime = time;
	if(instance_data[instance].pauseRequest == 0) 
    {
        if(!instance_data[instance].deviceissleeping) //if status reg is read when device is in deep sleep it will wake up the device!
        {
			if(getSpiHandle() > 0) //this is as Cheetah does not work with interrupts
			{
				while(dwt_checkIRQ()) // check if IRQS bit is active and process any new events
				{
					PINFO("dwt_isr received time %u",getmstime()-startTime);
					dwt_isr() ;  
				}

			}
			else
			{
				dwt_isr() ;  
			}
		}

		message = instance_peekevent(); //get any of the received events from ISR

	}
		while(done == INST_NOT_DONE_YET)
		{
			//int state = instance_data[instance].testAppState;
			done = testapprun(&instance_data[instance], message, time) ;                                               // run the communications application
			//we've processed message
			message = 0;
		}

        update = itemp.TXF + itemp.CRCG +
#if DECA_ERROR_LOGGING==1
                 itemp.CRCB + itemp.PHE + itemp.SFDTO + itemp.RSL +
#endif
                 instance_data[instance].rxTimeouts ;
		
        if (instance_data[instance].last_update != update) // something has changed
        {
			instance_data[instance].last_update = update ;
			//instancedisplaynewstatusdata(&instance_data[instance], &itemp);
            instance_data[instance].statusrep.changed = 1 ;
        }

    //}

	if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
        if(instance_data[instance].mode == TAG) //Tag start timer
        {
			instance_data[instance].instancetimer = time + instance_data[instance].tagSleepTime_ms; //start timer
			instance_data[instance].instancetimer_en = 1;
			//printf("sleep timer on %d prev:%d next:%d @ %d\n", instance_data[instance].testAppState, instance_data[instance].previousState, instance_data[instance].nextState, time);
        }
		if(instance_data[instance].mode == TAG_TDOA)
		{
			instance_data[instance].instancetimer = time + instance_data[instance].tagBlinkSleepTime_ms; //start timer
			instance_data[instance].instancetimer_en = 1;
		}
        instance_data[instance].stoptimer = 0 ; //clear the flag - timer can run if instancetimer_en set (set above)
		instance_data[instance].done = INST_NOT_DONE_YET;
    }

    //check if timer has expired
    if((instance_data[instance].instancetimer_en == 1) && (instance_data[instance].stoptimer == 0))
    {
		if(instance_data[instance].instancetimer < time)
		{
			event_data_t dw_event;
    		instance_data[instance].instancetimer_en = 0;
			dw_event.rxLength = 0;
			dw_event.type = DWT_SIG_RX_TIMEOUT;
			dw_event.type2 = 0x80 | DWT_SIG_RX_TIMEOUT;
			//printf("PC timeout DWT_SIG_RX_TIMEOUT\n");
			instance_putevent(dw_event);
		}
	}

    if (instance_data[instance].statusrep.changed)
    {
        instance_data[instance].statusrep.changed = 0 ;

        return 1 ;
    }
    return 0 ;
}

void instance_close(void)
{
	//int instance = 0 ;
	uint8 buffer[1500];

    // close/tidy up any device functionality - i.e. wake it up if in sleep mode
	if(dwt_spicswakeup(buffer, 1500) == DWT_ERROR)
	{
		printf("FAILED to WAKE UP\n");
	}

	dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

	dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts
	
	instancesetspibitrate(DW_DEFAULT_SPI);

	dwt_forcetrxoff();

	closespi(); //close the SPI
}

// -------------------------------------------------------------------------------------------------------------------
//
// Soft Reset, called after verifying device ID, and before doing anything else
//
void instancedevicesoftreset(void)
{
	//revert SPI back to default (e.g. <= 3M)
	instancesetspibitrate(DW_DEFAULT_SPI);
	//reset the DW1000 device
	dwt_softreset() ;
}
#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
