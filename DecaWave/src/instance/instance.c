// -------------------------------------------------------------------------------------------------------------------
//
//  File: instance.c - application level message exchange for ranging demo
//
//  Copyright 2008 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Billy Verso, December 2008
//
// -------------------------------------------------------------------------------------------------------------------

#include "compiler.h"
#include "deca_device_api.h"
#include "spiDriver.h"

#include "instance.h"

// -------------------------------------------------------------------------------------------------------------------

#define INST_DONE_WAIT_FOR_NEXT_EVENT   1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               0   //this signifies that the instance is still processing the current event

//function codes
#define RTLS_DEMO_MSG_RNG_INIT				(0x20)			// Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)
#define RTLS_DEMO_MSG_ANCH_TOFR             (0x2A)          // Anchor TOF Report message


//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
#define TOFR                                1
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option paramter 0x00 (1) - used to notify Tag that the report is coming
#define RES_R3                              3               // Response option paramter 0x00 (1),
#define RES_T1                              3               // Ranging request response delay low byte
#define RES_T2                              4               // Ranging request response delay high byte
#define POLL_TEMP                           1               // Poll message TEMP octet
#define POLL_VOLT                           2               // Poll message Voltage octet


#define ACK_REQUESTED                       (1)             // Request an ACK frame

enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,    //4
    TA_TXREPORT_WAIT_SEND,      //5
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP,					//9
    TA_SLEEP_DONE,				//10
	TA_TXBLINK_WAIT_SEND,		//11
    TA_TXRANGINGINIT_WAIT_SEND  //12
} ;

typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

//The table below specifies the default TX spectrum configuration parameters... this has been tuned for DW EVK hardware units
const tx_struct txSpectrumConfig[8] =
{
    //Channel 0 ----- this is just a place holder so the next array element is channel 1
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 1
    {
            0xc9,   //PG_DELAY
            {
                    0x75757575, //16M prf power
                    0x67676767 //64M prf power
            }

    },
    //Channel 2
    {
            0xc2,   //PG_DELAY
            {
                    0x75757575, //16M prf power
                    0x67676767 //64M prf power
            }
    },
    //Channel 3
    {
            0xc5,   //PG_DELAY
            {
                    0x6f6f6f6f, //16M prf power
                    0x8b8b8b8b //64M prf power
            }
    },
    //Channel 4
    {
            0x95,   //PG_DELAY
            {
                    0x5f5f5f5f, //16M prf power
                    0x9a9a9a9a //64M prf power
            }
    },
    //Channel 5
    {
            0xc0,   //PG_DELAY
            {
                    0x48484848, //16M prf power
                    0x85858585 //64M prf power
            }
    },
    //Channel 6 ----- this is just a place holder so the next array element is channel 7
    {
            0x0,   //0
            {
                    0x0, //0
                    0x0 //0
            }
    },
    //Channel 7
    {
            0x93,   //PG_DELAY
            {
                    0x92929292, //16M prf power
                    0xd1d1d1d1 //64M prf power
            }
    }
};

//these are default antenna delays for EVB1000, these can be used if there is no calibration data in the DW1000,
//or instead of the calibration data
const uint16 rfDelays[2] = {
        (uint16) ((DWT_PRF_16M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS),//PRF 16
        (uint16) ((DWT_PRF_64M_RFDLY/ 2.0) * 1e-9 / DWT_TIME_UNITS)
};


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------
#define MAX_NUMBER_OF_REPORT_RETRYS (3)         // max number of times to send/re-send the report message

#define FIXED_REPORT_DELAY                  0 //15 //ms             //tx delay when sending the (ToF) report
#define RX_ON_TIME                          2 //ms                  //the time RX is turned on before the reply from the other end
#define RX_FWTO_TIME                        (RX_ON_TIME + 5) //ms //total "RX on" time
#define BLINK_SLEEP_DELAY					1000 //ms

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// -------------------------------------------------------------------------------------------------------------------
int stateCount = 0;
double inst_idist = 0;
double inst_adist = 0;
double inst_ldist = 0;
instance_data_t instance_data[NUM_INST] ;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
double convertdevicetimetosec8(uint8* dt)
{
    double f = 0;

    uint32 lo = 0;
    int8 hi = 0;

    memcpy(&lo, dt, 4);
    hi = dt[4] ;

    f = ((hi * 65536.00 * 65536.00) + lo) * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

double convertdevicetimetosec(int64 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}


void reportTOF(instance_data_t *inst)
{
        double distance ;
        double tof ;
        double ltave;
        int64 tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = (int64) inst->tof ;                          // make it signed
        if (tofi > 0x007FFFFFFFFF)                          // MP counter is 40 bits,  close up TOF may be negative
        {
            tofi -= 0x010000000000 ;                       // subtract fill 40 bit range to mak it negative
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi) * 0.25;          //this is divided by 4 to get single time of flight
        distance = tof * SPEED_OF_LIGHT;

        if ((distance < -0.5) || (distance > 20000.000))    // discard any results less than <50 cm or >20km
            return;

#if (CORRECT_RANGE_BIAS == 1)
        distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance, inst->configData.prf);
#endif

        distance = fabs(distance) ;                         // make any (small) negatives positive.

        inst_idist = distance;

        inst->longTermRangeSum+= distance ;
        inst->longTermRangeCount++ ;                          // for computing a long term average
        ltave = inst->longTermRangeSum / inst->longTermRangeCount ;

        inst_ldist = ltave ;

        inst->adist[inst->tofindex++] = distance;

        if(distance < inst->idistmin)
            inst->idistmin = distance;

        if(distance > inst->idistmax)
            inst->idistmax = distance;

        if(inst->tofindex == RTD_MED_SZ) inst->tofindex = 0;

        if(inst->tofcount == RTD_MED_SZ)
        {
            int i;
            double  avg;

            avg = 0;

            for(i = 0; i < inst->tofcount; i++)
            {
                avg += inst->adist[i];
            }
            avg /= inst->tofcount;

            inst_adist = avg ;

        }
        else
            inst->tofcount++;
    return ;
}// end of reportTOF

// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigframeheader(instance_data_t *inst, int ackrequest)
{
    inst->msg.panID[0] = (inst->panid) & 0xff;
    inst->msg.panID[1] = inst->panid >> 8;

    //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    inst->msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
    inst->msg.frameCtrl[0] |= (ackrequest ? 0x20 : 0x00);
#if (USING_64BIT_ADDR==1)
    //source/dest addressing modes and frame version
    inst->msg.frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
    inst->msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif
    inst->msg.seqNum = inst->frame_sn++;

}

// -------------------------------------------------------------------------------------------------------------------
//
// function to select the destination address (e.g. the address of the next anchor to poll)
//
// -------------------------------------------------------------------------------------------------------------------
//
int instaddtagtolist(instance_data_t *inst, uint64 tagAddr)
{
	int i;

	//add the new Tag to the list, if not already there and there is space
	for(i=0; i<TAG_LIST_SIZE; i++)
	{
		if(inst->tagList[i] != tagAddr)
		{
			if(inst->tagList[i] == 0)
			{
				inst->tagList[i] = tagAddr ;
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

void instcleartaglist(void)
{
	int instance = 0 ;

	memset((uint8 *) &instance_data[instance].tagList[0], 0, TAG_LIST_SIZE * sizeof(instance_data[instance].tagList[0]));

	instance_data[instance].tagToRangeWith = TAG_LIST_SIZE;
}

uint64* instgettaglist(void)
{
	int instance = 0 ;

    return instance_data[instance].tagList ;
}

void instsettagtorangewith(int tagID)
{
	int instance = 0 ;

	instance_data[instance].tagToRangeWith = tagID ;
}

#if (DR_DISCOVERY == 0)
int destaddress(instance_data_t *inst)
{
    int getnext = 1;
    int tempanchorListIndex = 0;
    //set destination address (Tag will cycle though the list of anchor addresses)

    if(inst->payload.anchorPollMask == 0)
        return 1; //error - list is empty

    while(getnext)
    {
        if((0x1 << inst->anchorListIndex) & inst->payload.anchorPollMask)
        {
            memcpy(&inst->msg.destAddr, &inst->payload.anchorAddressList[inst->anchorListIndex], ADDR_BYTE_SIZE);
            getnext = 0;
        }

        inst->anchorListIndex++ ;
    }

    tempanchorListIndex = inst->anchorListIndex;

    while(tempanchorListIndex <= inst->payload.anchorListSize)
        {
        //check if there are any anchors left to poll
        if((0x1 << tempanchorListIndex) & inst->payload.anchorPollMask)
        {
            return 0;
        }
        tempanchorListIndex++;
    }

    //if we got this far means that we are just about to poll the last anchor in the list
    inst->instToSleep = 1; //we'll sleep after this poll
    inst->anchorListIndex = 0; //start from the first anchor in the list after sleep finishes

    return 0;
}
#endif


// -------------------------------------------------------------------------------------------------------------------
//
// function to configure the mac frame data, prior to issuing the PD_DATA_REQUEST
//
// -------------------------------------------------------------------------------------------------------------------
//
void setupmacframedata(instance_data_t *inst, int len, int fcode, int ack)
{
    inst->macdata_msdu[FCODE] = fcode; //message function code (specifies if message is a poll, response or other...)

    if(len)
        memcpy(inst->msg.messageData, inst->macdata_msdu, len); //copy application data

    inst->psduLength = len + FRAME_CRTL_AND_ADDRESS + FRAME_CRC;

    //inst->psduLength = adduserpayload(inst, inst->psduLength, len); //add any user data to the message payload

    instanceconfigframeheader(inst, ack); //set up frame header (with/without ack request)

    if(ack == ACK_REQUESTED)
        inst->wait4ack = DWT_RESPONSE_EXPECTED;

    inst->ackexpected = ack ; //used to ignore unexpected ACK frames
}

// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
void instancerxon(int delayed, uint64 delayedReceiveTime)
{
    if (delayed)
    {
        uint32 dtime;
        dtime =  (uint32) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

#if (SNIFF_MODE == 1)
    dwt_setrxmode(RX_SNIFF, 0, 0x02, 0xFF); //Off time 0xFF, on time 0x2
#endif

    dwt_rxenable(delayed) ;               // turn receiver on, immediate/delayed

} // end instancerxon()


int instancesendpacket(instance_data_t *inst, int delayedTx)
{
    int result = 0;

    dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg, 0) ;   // write the frame data
    dwt_writetxfctrl(inst->psduLength, 0);
    if(delayedTx)
    {
        uint32 dtime;
        dtime = (uint32) (inst->delayedReplyTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    if(inst->wait4ack)
    {
        //if the ACK is requested there is a 5ms timeout to stop RX if no ACK coming
        dwt_setrxtimeout(5000);  //units are us - wait for 5ms after RX on
    }

    //begin delayed TX of frame
    if (dwt_starttx(delayedTx | inst->wait4ack))  // delayed start was too late
    {
        result = 1; //late/error
    }


    return result;                                              // state changes
    // after sending we should return to TX ON STATE ?
}

int powertest(void)
{
    dwt_config_t    configData ;
    dwt_txconfig_t  configTx ;
    uint8 msg[127] = "The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the lazy dog. The quick brown fox jumps over the l";

#ifdef ST_MC
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_16); //reduce the SPI speed before putting device into low power mode
#endif
    //
    //  reset device
    //
    dwt_softreset();

    //
    //  configure channel paramters
    //
    configData.chan = 2 ;
    configData.rxCode =  9 ;
    configData.txCode = 9 ;
    configData.prf = DWT_PRF_64M ;
    configData.dataRate = DWT_BR_110K ;
    configData.txPreambLength = DWT_PLEN_2048 ;
    configData.rxPAC = DWT_PAC64 ;
    configData.nsSFD = 1 ;
    configData.smartPowerEn = 0;

    dwt_configure(&configData, DWT_LOADANTDLY | DWT_LOADXTALTRIM) ;

    configTx.PGdly = txSpectrumConfig[configData.chan].PGdelay ;
    configTx.power = txSpectrumConfig[configData.chan].txPwr[configData.prf - DWT_PRF_16M];

    dwt_configuretxrf(&configTx);

    // the value here 0x1000 gives a period of 32.82 �s
    //this is setting 0x1000 as frame period (125MHz clock cycles) (time from Tx en - to next - Tx en)
    dwt_configcontinuousframemode(0x1000);

    dwt_writetxdata(127, (uint8 *)  msg, 0) ;
    dwt_writetxfctrl(127, 0);

    //to start the first frame - set TXSTRT
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    //measure the power
    //Spectrum Analyser set:
    //FREQ to be channel default e.g. 3.9936 GHz for channel 2
    //SPAN to 1GHz
    //SWEEP TIME 1s
    //RBW and VBW 1MHz
    //measure channel power

    return DWT_SUCCESS ;
}


void xtalcalibration(void)
{
    int i;
    uint8 chan = 2 ;
    uint8 prf = DWT_PRF_16M ;
    dwt_txconfig_t  configTx ;
#ifdef ST_MC
    SPI_ConfigFastRate(SPI_BaudRatePrescaler_16); //reduce the SPI speed before putting device into low power mode
#endif

    //
    //  reset device
    //
    dwt_softreset();

    //
    //  configure TX channel parameters
    //

    configTx.PGdly = txSpectrumConfig[chan].PGdelay ;
    configTx.power = txSpectrumConfig[chan].txPwr[prf - DWT_PRF_16M];

    dwt_configuretxrf(&configTx);

    dwt_configcwmode(chan);

    for(i=0; i<=0x1F; i++)
    {
        dwt_xtaltrim(i);
        //measure the frequency
        //Spectrum Analyser set:
        //FREQ to be channel default e.g. 3.9936 GHz for channel 2
        //SPAN to 10MHz
        //PEAK SEARCH
    }

    return;
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag, Anchor or Listener use the same statemachine....)
//
// -------------------------------------------------------------------------------------------------------------------
//
int testapprun(instance_data_t *inst, int message)
{

    switch (inst->testAppState)
    {
        case TA_INIT :
            // printf("TA_INIT") ;
            switch (inst->mode)
            {
                case TAG:
                {
					dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
                    dwt_setpanid(inst->panid);
					dwt_seteui(inst->eui64);
#if (USING_64BIT_ADDR==0)
                    {
                        uint16 addr = inst->payload.tagAddress & 0xFFFF;
                        dwt_setaddress16(addr);
                    }
#endif
                    //set source address into the message structure
					memcpy(&inst->msg.sourceAddr, inst->eui64, ADDR_BYTE_SIZE);
                    //change to next state - send a Poll message to 1st anchor in the list
#if (DR_DISCOVERY == 1)
					inst->mode = TAG_TDOA ;
                    inst->testAppState = TA_TXBLINK_WAIT_SEND;
					memcpy(inst->blinkmsg.tagID, inst->eui64, ADDR_BYTE_SIZE);
#else
                    inst->testAppState = TA_TXPOLL_WAIT_SEND;
#endif

                    dwt_setautorxreenable(inst->rxautoreenable); //not necessary to auto RX re-enable as the receiver is on for a short time (Tag knows when the response is coming)
#if (DOUBLE_RX_BUFFER == 1)
                    dwt_setdblrxbuffmode(0); //disable double RX buffer
#endif
#if (ENABLE_AUTO_ACK == 1)
                    dwt_enableautoack(ACK_RESPONSE_TIME); //wait for 100 symbols before replying with the ACK
#endif

#if (DEEP_SLEEP == 1)
#if (DEEP_SLEEP_AUTOWAKEUP == 1)
					dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_SLPCNT|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)		
#else
					//NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the DW1000 into full DEEPSLEEP mode as XTAL is kept on
#if (DEEP_SLEEP_XTAL_ON == 1)
					dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_CS|DWT_SLP_EN|DWT_XTAL_EN); //configure the on wake parameters (upload the IC config settings)		
#else
					dwt_configuresleep(DWT_LOADUCODE|DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV, DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)		
#endif
#endif
#endif
                }
                break;
                case ANCHOR:
                {
#if (DR_DISCOVERY == 0)
					uint8 eui64[8] ;
                    memcpy(eui64, &inst->payload.anchorAddress, sizeof(uint64));
                    
                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ack frames;
					dwt_seteui(eui64);
#else
					dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
					dwt_seteui(inst->eui64);
#endif
                    dwt_setpanid(inst->panid);
					
					
#if (USING_64BIT_ADDR==0)
                    {
                        uint16 addr = inst->payload.anchorAddress & 0xFFFF;
                        dwt_setaddress16(addr);
                    }
#endif

#if (DR_DISCOVERY == 0)
                    //set source address into the message structure
                    memcpy(&inst->msg.sourceAddr, &inst->payload.anchorAddress, ADDR_BYTE_SIZE);
#else
					//set source address into the message structure
					memcpy(&inst->msg.sourceAddr, inst->eui64, ADDR_BYTE_SIZE);
#endif
                    // First time anchor listens we don't do a delayed RX
                    inst->shouldDoDelayedRx = FALSE ;
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;
#if (ENABLE_AUTO_ACK == 1)
                    dwt_setrxaftertxdelay(WAIT_FOR_RESPONSE_DLY); //set the RX after TX delay time
#endif

#if (DECA_BADF_ACCUMULATOR == 0) //can use RX auto re-enable when not logging/plotting errored frames
                    inst->rxautoreenable = 1;
#endif
                    dwt_setautorxreenable(inst->rxautoreenable);
#if (DOUBLE_RX_BUFFER == 1)
					dwt_setdblrxbuffmode(0); //enable double RX buffer 
#endif
                    dwt_setrxtimeout(0);

                }
                break;
                case LISTENER:
                {
                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
                    // First time anchor listens we don't do a delayed RX
                    inst->shouldDoDelayedRx = FALSE ;
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

#if (DECA_BADF_ACCUMULATOR == 0) //can use RX auto re-enable when not logging/plotting errored frames
                    inst->rxautoreenable = 1;
#endif
                    dwt_setautorxreenable(inst->rxautoreenable);
#if (DOUBLE_RX_BUFFER == 1)
                    dwt_setdblrxbuffmode(1); //enable double RX buffer
#endif


                    dwt_setrxtimeout(0);

                }
                break ; // end case TA_INIT
                default:
                break;
            }
            break; // end case TA_INIT

        case TA_SLEEP_DONE :

            if(message != DWT_SIG_RX_TIMEOUT)
            {
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                break;
            }

            inst->done = INST_NOT_DONE_YET;
            inst->instToSleep = 0;
            inst->testAppState = inst->nextState;
            inst->nextState = 0; //clear
#if (DEEP_SLEEP == 1)
            {
#ifdef _MSC_VER
                uint8 buffer[1500];
                //wake up device from low power mode
				//1000 takes 400us, 1300 takes 520us, 1500 takes 600us (SPI @ 20MHz)
                //then the function will wait 5ms to make sure the XTAL has stabilised
				if(dwt_spicswakeup(buffer, 1500) == DWT_ERROR) //1000 takes 400us, 1300 takes 520us, 1500 takes 600us (SPI @ 20MHz)
                {
                    printf("FAILED to WAKE UP\n");
                }
#else
                //wake up device from low power mode
                //NOTE - in the ARM  code just drop chip select for 200us
//TODO check how to wake in linux
#ifdef ST_MC
                port_SPIx_clear_chip_select();  //CS low
                Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
                port_SPIx_set_chip_select();  //CS high
                Sleep(5);
#endif
                //add Sleep(50) to stabilise the XTAL
                Sleep(50);
#endif
                //this is platform dependent - only program if DW EVK/EVB
                dwt_setleds(1);

                //MP bug - TX antenna delay needs reprogramming as it is not preserved
                dwt_settxantennadelay(inst->txantennaDelay) ;

                //set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
				/*if((inst->mode == TAG) || (inst->mode == TAG_TDOA))
                {
                    dwt_setpanid(inst->panid);
					dwt_seteui(inst->eui64);
				}*/
                dwt_entersleepaftertx(0);
                dwt_setinterrupt(DWT_INT_TFRS, 1); //re-enable the TX/RX interrupts
            }
#endif
            break;

        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
            // printf("TA_TXE_WAIT") ;
            //if we are scheduled to go to sleep before next sending then sleep first.
			if(((inst->nextState == TA_TXPOLL_WAIT_SEND)
				|| (inst->nextState == TA_TXBLINK_WAIT_SEND))
                    && (inst->instToSleep)  //go to sleep before sending the next poll
                    )
            {
                //the app should put chip into low power state and wake up in tagSleepTime_ms time...
                //the app could go to *_IDLE state and wait for uP to wake it up...
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the TagTimeoutTimer (instancetimer)
                inst->testAppState = TA_SLEEP_DONE;
#if (DEEP_SLEEP == 1)
                //put device into low power mode
                dwt_entersleep(); //go to sleep
#endif
            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT

        case TA_TXBLINK_WAIT_SEND :
            {
                //blink frames with IEEE EUI-64 tag ID
                inst->blinkmsg.frameCtrl = 0xC5 ;
                inst->blinkmsg.seqNum = inst->frame_sn++;

                dwt_writetxdata((BLINK_FRAME_CRTL_AND_ADDRESS + FRAME_CRC), (uint8 *)  (&inst->blinkmsg), 0) ;	// write the frame data
				dwt_writetxfctrl((BLINK_FRAME_CRTL_AND_ADDRESS + FRAME_CRC), 0);

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
#if (DEEP_SLEEP == 1)
                dwt_entersleepaftertx(1);
                dwt_setinterrupt(DWT_INT_TFRS, 0);
#endif
#endif
                dwt_starttx(DWT_START_TX_IMMEDIATE); //always using immediate TX

#if (DEEP_SLEEP_AUTOWAKEUP == 0)
				stateCount = 0;
				inst->instToSleep = 1;
                inst->testAppState = TA_TX_WAIT_CONF ; // wait confirmation
                inst->previousState = TA_TXBLINK_WAIT_SEND ;
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)
				dwt_setrxtimeout((RX_FWTO_TIME) * 1000);  //units are us - wait for 5ms after RX on
#else
				while(1);		//Tag will continue blinking at defined bilnk rate... 
#endif
            }
            break ; // end case TA_TXBLINK_WAIT_SEND

		case TA_TXRANGINGINIT_WAIT_SEND :
                {
                //tell Tag what it's address will be for the ranging exchange
                inst->macdata_msdu[RES_R1] = inst->tagShortAdd & 0xFF;
                inst->macdata_msdu[RES_R2] = (inst->tagShortAdd >> 8) & 0xFF; //
                inst->macdata_msdu[RES_T1] = ((int) (inst->fixedReplyDelay_ms)) & 0xFF;
                inst->macdata_msdu[RES_T2] = (((int) (inst->fixedReplyDelay_ms)) >> 8) & 0xFF; //
                //set destination address
                memcpy(&inst->msg.destAddr, &inst->tagList[inst->tagToRangeWith], ADDR_BYTE_SIZE);

                setupmacframedata(inst, RANGINGINIT_MSG_LEN, RTLS_DEMO_MSG_RNG_INIT, !ACK_REQUESTED);

                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;

				//if(instancesendpacket(inst, DWT_START_TX_IMMEDIATE))
				if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    //error - TX FAILED
                    inst->txu.txTimeStamp = 0;
                    inst->shouldDoDelayedRx = FALSE ;   // no delay in turning on RX
					inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new blink or poll message	
                }
                else
                {
                    stateCount = 0;
                    inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
					inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;
                	inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout 
                }

				//anchor has this enabled by default
				//dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //we are starting ranging - enable the filter....
            }
            break;

        case TA_TXPOLL_WAIT_SEND :
            {
				
#if (DR_DISCOVERY == 1)
				//NOTE the anchor address is set after receiving the ranging initialisation message
            	inst->instToSleep = 1; //we'll sleep after this poll
#else
				//set destination address 
                if(destaddress(inst))
                {
                    break;
                }
#endif
#if (PUT_TEMP_VOLTAGE_INTO_POLL == 1)
                {
                    inst->macdata_msdu[POLL_TEMP] = dwt_readwakeuptemp() ;   // Temperature value set sampled at wakeup
                    inst->macdata_msdu[POLL_VOLT] = dwt_readwakeupvbat() ;   // (Battery) Voltage value set sampled at wakeup
                }
#else
					inst->macdata_msdu[POLL_TEMP] = inst->macdata_msdu[POLL_VOLT] = 0;
#endif
                setupmacframedata(inst, TAG_POLL_MSG_LEN, RTLS_DEMO_MSG_TAG_POLL, !ACK_REQUESTED);

                
				instancesendpacket(inst, DWT_START_TX_IMMEDIATE);
                
				stateCount = 0;
                inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

                dwt_setrxtimeout((RX_FWTO_TIME) * 1000);  //units are us - wait for 5ms after RX on

            }
            break;

        case TA_TXRESPONSE_WAIT_SEND :
        {
                //program option octet and parameters (not used currently)
                inst->macdata_msdu[RES_R1] = 0x2; // "activity"
                inst->macdata_msdu[RES_R2] = inst->sendTOFR2Tag; //0x0; (this tells the Tag that the ToF report will be sent to it)
                inst->macdata_msdu[RES_R3] = 0x0;

                //set destination address
                memcpy(&inst->msg.destAddr, &inst->relpyAddress, ADDR_BYTE_SIZE);

                setupmacframedata(inst, ANCH_RESPONSE_MSG_LEN, RTLS_DEMO_MSG_ANCH_RESP, !ACK_REQUESTED);

                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXRESPONSE_WAIT_SEND ;

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    //error - TX FAILED
                    inst->txu.txTimeStamp = 0;
                    inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new poll
                    inst->shouldDoDelayedRx = FALSE ;   // no delay in turning on RX
                }
                else
                {
                    stateCount = 0;
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout
                }
            }
            break;

        case TA_TXFINAL_WAIT_SEND :
            {
                uint64 tagCalculatedFinalTxTime ;

                // Embbed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
                // Write Poll TX time field of Final message
                memcpy(&(inst->macdata_msdu[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);

                // Write Response RX time field of Final message
                memcpy(&(inst->macdata_msdu[RRXT]), (uint8 *)&inst->rxu.anchorRespRxTime, 5);

                // Calculate Time Final message will be sent and write this field of Final message
                // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
                // zeroing its low 9 bits, and then having the TX antenna delay added
                tagCalculatedFinalTxTime = inst->delayedReplyTime & MASK_TXDTS; // 9 lower bits mask

                // getting antenna delay from the device and add it to the Calculated TX Time
                tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txantennaDelay;

                tagCalculatedFinalTxTime &= MASK_40BIT;

                // Write Calculated TX time field of Final message
                memcpy(&(inst->macdata_msdu[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);

                //set destination address
                memcpy(&inst->msg.destAddr, &inst->relpyAddress, ADDR_BYTE_SIZE);

                setupmacframedata(inst, TAG_FINAL_MSG_LEN, RTLS_DEMO_MSG_TAG_FINAL, !ACK_REQUESTED);

#if (DEEP_SLEEP == 1)
                if(inst->tag2rxReport==0) //if not going to wait for report, go to sleep after TX is complete
                {
                    dwt_entersleepaftertx(1);
                    dwt_setinterrupt(DWT_INT_TFRS, 0); //disable all the interrupts (wont be able to enter sleep if interrupts are pending)
                }
#endif

                if(instancesendpacket(inst, DWT_START_TX_DELAYED))
                {
                    //error - TX FAILED
                    inst->txu.txTimeStamp = 0;

                    // initiate the re-transmission
                    inst->testAppState = TA_TXE_WAIT ;
                    inst->nextState = TA_TXPOLL_WAIT_SEND ;
#if (DEEP_SLEEP == 1)
                    dwt_entersleepaftertx(0);
#endif
                    break; //exit this switch case...
                }
                else
                {
                    stateCount = 0;
                    inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                    inst->previousState = TA_TXFINAL_WAIT_SEND;
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out  (set below)
                }

                if(inst->tag2rxReport) //if waiting for report - set timeout to be same as a Sleep timer... if no report coming time out and send another poll
                {
                    dwt_setrxtimeout(0);
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO;
                }
                else //if Tag is not waiting for report - it will go to sleep automatically after the final is sent
                {
#if (DEEP_SLEEP == 1)
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //kick off the TagTimeoutTimer (instancetimer) to initiate wakeup
					inst->nextState = TA_TXPOLL_WAIT_SEND;
                    inst->testAppState = TA_SLEEP_DONE; //we are going automatically to sleep so no TX confirm interrupt (next state will be TA_SLEEP_DONE)
                    inst->txmsgcount ++;
#endif
                }

            }
            break;

        case TA_TXREPORT_WAIT_SEND :
            {
                int delayed = DWT_START_TX_DELAYED ;
                if(inst->newReportSent == 0) //keep the same message if re-sending the same report
                {
                    // Write calculated TOF into report message
                    memcpy(&(inst->macdata_msdu[TOFR]), &inst->tof, 5);

                    //set destination address
#if (DR_DISCOVERY == 0)
                    if(inst->sendTOFR2Tag)
                        memcpy(&inst->msg.destAddr, &inst->relpyAddress, ADDR_BYTE_SIZE);
                    else
                        memcpy(&inst->msg.destAddr, &inst->payload.forwardToFRAddress, ADDR_BYTE_SIZE);
#else
					if(inst->sendTOFR2Tag)
						memcpy(&inst->msg.destAddr, &inst->relpyAddress, ADDR_BYTE_SIZE);
#endif
                }
                else
                {
                    //re-send the old report
                    inst->msg.seqNum--;
                }

#if (ENABLE_AUTO_ACK == 1)
                setupmacframedata(inst, TOF_REPORT_MSG_LEN, RTLS_DEMO_MSG_ANCH_TOFR, ACK_REQUESTED);
#else
                setupmacframedata(inst, TOF_REPORT_MSG_LEN, RTLS_DEMO_MSG_ANCH_TOFR, !ACK_REQUESTED);
#endif

                if(inst->delayedReplyTime == 0)
                {
                    delayed = DWT_START_TX_IMMEDIATE ;
                }

                if(instancesendpacket(inst, delayed))
                {
                    //error - TX FAILED
                    inst->txu.txTimeStamp = 0;
                    inst->delayedReplyTime = 0;
                    //if fails to send, go back to receive mode and wait to receive a new poll
                    if(inst->newReportSent < MAX_NUMBER_OF_REPORT_RETRYS) //re-try
                    {
                        //stay in this state
                        inst->testAppState = TA_TXREPORT_WAIT_SEND ;
                    }
                    else
                    {
                        inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new poll
                    }
                }
                else
                {
                    inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                    inst->previousState = TA_TXREPORT_WAIT_SEND ;

                    inst->newReportSent++;
                    inst->delayedReplyTime = 0;
#if (ENABLE_AUTO_ACK == 1)
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
#else
                    //use anchor report timeout to timeout and re-send the ToF report - timeout and send next report (no RX on)
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO;
                    inst->wait4ack = RESPONSE_EXPECTED; //this is to skip turning the RX on (we'll just timeout and send the next report)
#endif
                }
            }
            break;


        case TA_TX_WAIT_CONF :
           //printf("TA_TX_WAIT_CONF") ;
            {
                if(message == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
                {
                    //printf("RX timeout in TA_TX_WAIT_CONF (%d)\n", inst->previousState);

                    //if we got no ACKs after sending MAX_NUMBER_OF_REPORT_RETRYS reports go to wait for RX state
                    if(inst->newReportSent && (inst->newReportSent >= MAX_NUMBER_OF_REPORT_RETRYS))
                    {
                        //don't change state - next event will the TX confirm from the sent report, so will just fall through to RX
                        inst->shouldDoDelayedRx = FALSE ;
                        inst->wait4ack = 0 ; //clear the flag as the ACK has been received
                        dwt_setrxtimeout(0);
                    }
                    else
                    {
                        //got timeout before TX confirm
                        inst->testAppState = TA_TXE_WAIT;
                        inst->nextState = inst->previousState ; // send poll / response / final / report (with ACK request)
                    }
                    break;
                }

                //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
                //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
                //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
                if(message != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
                    if(message == SIG_RX_ACK)
                    {
                        /*
                        ack_msg *rxmsg = &inst->rxackmsg;

                        if(rxmsg->seqNum == inst->msg.seqNum)
                        {
                            printf("got the expecting ACK frame\n");
                        }
                        */
                        inst->wait4ack = 0 ; //clear the flag as the ACK has been received
                        dwt_setrxtimeout(0); //clear/disable timeout
                        //printf("RX ACK in TA_TX_WAIT_CONF... wait for TX confirm before changing state\n");
                    }

                    if(stateCount++ > 50)
                    {
                        //printf("NO TX done??? while waiting for TX done ?\n");
                    }

                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                        break;

                }

                message = 0; //finished with the message

                //printf("got TX done stateCount %d\n", stateCount);
                stateCount = 0;  //ARM counts about 165 times...
                inst->done = INST_NOT_DONE_YET;

                if((inst->previousState == TA_TXFINAL_WAIT_SEND) //tag will do immediate receive when waiting for report (as anchor sends it without delay)
                        && (inst->tag2rxReport == 0)) //anchor is not sending the report to tag
                {
                    inst->testAppState = TA_TXE_WAIT ;
                    inst->nextState = TA_TXPOLL_WAIT_SEND ;
                    //inst->previousState = 0;
                    break;
                }
                else
                {

                    switch(inst->previousState)
                    {
                    	case TA_TXBLINK_WAIT_SEND:
                    	case TA_TXPOLL_WAIT_SEND:
                        case TA_TXRESPONSE_WAIT_SEND:
                        inst->shouldDoDelayedRx = TRUE ;                        // response is delayed so we can delay turning on RX
                            break;

                        case TA_TXREPORT_WAIT_SEND:
                        case TA_TXFINAL_WAIT_SEND: // if expecting a report don't delay turning on RX
						case TA_TXRANGINGINIT_WAIT_SEND:
                        default:
                        inst->shouldDoDelayedRx = FALSE ;
                    break;
                    }


                    inst->testAppState = TA_RXE_WAIT ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll
                    //fall into the next case (turn on the RX)

                }

            }

            //break ; // end case TA_TX_WAIT_CONF


        case TA_RXE_WAIT :
        // printf("TA_RXE_WAIT") ;
        {

            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
                uint64 delayedReceiveTime = 0;

                if (inst->shouldDoDelayedRx) //we don't need to turn receiver on immediately, as we know the response will come in a while
                {
                    delayedReceiveTime = (inst->txu.txTimeStamp + inst->rxOnDelay) & MASK_40BIT;

                    if(inst->previousState == TA_TXBLINK_WAIT_SEND) //check if we need to use long response delay for the blink response
                    {
                    	if(inst->fixedReplyDelay_ms > FIXED_LONG_REPLY_DELAY)
                    	{
                    		delayedReceiveTime = (delayedReceiveTime + (DELAY_MULTIPLE*inst->fixedReplyDelay)) & MASK_40BIT;
                    	}
                    }
                }

                //turn RX on
                instancerxon(inst->shouldDoDelayedRx, delayedReceiveTime) ;   // turn RX on, with/without delay
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

            inst->shouldDoDelayedRx = FALSE ; //clear the flag

            if (inst->mode != LISTENER)
            {
                if (inst->previousState != TA_TXREPORT_WAIT_SEND) //we are going to use anchor timeout and re-send the report
                    inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
            }

            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) break;
        }

        case TA_RX_WAIT_DATA :                                                                     // Wait RX data
           //printf("TA_RX_WAIT_DATA") ;

            switch (message)
            {

				case SIG_RX_BLINK : 
				{
					uint64 tagaddr = 0;
					memcpy(&tagaddr, &(inst->blinku.rxblinkmsg.tagID), ADDR_BYTE_SIZE);

					//printf("we got blink message from %08X\n", ( tagaddr& 0xFFFF));
					if((inst->mode == LISTENER) || (inst->mode == ANCHOR))
					{
						//add this Tag to the list of Tags we know about
						instaddtagtolist(inst, tagaddr);

						//initiate ranging message
						if(inst->tagToRangeWith < TAG_LIST_SIZE)
						{
							//initiate ranging message this is a Blink from the Tag we would like to range to
							if(inst->tagList[inst->tagToRangeWith] == tagaddr)
							{
								inst->tagShortAdd = (dwt_getpartid() & 0xFF); 
								inst->tagShortAdd =  (inst->tagShortAdd << 8) + (tagaddr & 0xFF) ;

								if(inst->fixedReplyDelay_ms > FIXED_LONG_REPLY_DELAY)
								{
									inst->delayedReplyTime = inst->rxu.rxTimeStamp + ((DELAY_MULTIPLE + 1) * inst->fixedReplyDelay) ;  // time we should send the blink response
								}
								else
								{
									inst->delayedReplyTime = inst->rxu.rxTimeStamp + inst->fixedReplyDelay ;  // time we should send the blink response
								}
								inst->delayedReplyTime &= MASK_40BIT ;

								inst->relpyAddress = tagaddr; //remember who to send the reply to

								inst->testAppState = TA_TXE_WAIT;
								inst->nextState = TA_TXRANGINGINIT_WAIT_SEND ; 

								break;
							}

							//else stay in RX
						}
					}
					
					//else //not initiating ranging - continue to receive 
					{
						inst->testAppState = TA_RXE_WAIT ;
						inst->done = INST_NOT_DONE_YET;
					}

				}
				break;



                case SIG_RX_ACK :
                {
                    ack_msg *rxmsg = &inst->rxackmsg;

                    if(inst->ackexpected) //used to ignore unexpected ACK frames
                    {
						if(rxmsg->seqNum == inst->msg.seqNum) //this is the ACK we expect
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
							}

							inst->ackexpected = 0 ;
						}
                    }
                    //else we did not expect this ACK turn the RX on again
#if (DOUBLE_RX_BUFFER == 0)
                    inst->testAppState = TA_RXE_WAIT ;
#endif
                    inst->done = INST_NOT_DONE_YET;
                }
                break;

                case DWT_SIG_RX_OKAY :
                {
                    srd_msg *rxmsg = &inst->rxmsg;
                    uint64  srcAddr;
                    //int non_user_payload_len = 0;
                    //int uplen = 0;
					int fcode = 0;
                    inst->stoptimer = 0; //clear the flag

                    // 16 or 64 bit addresses
                    memcpy(&srcAddr, &(rxmsg->sourceAddr), ADDR_BYTE_SIZE);

                    {
#if (DR_DISCOVERY == 1)
						if(inst->mode == ANCHOR)
						{
							if(inst->tagList[inst->tagToRangeWith] == srcAddr) //if the Tag's address does not match (ignore the message)
							{
								fcode = rxmsg->messageData[FCODE];
							}
						}
						else // LISTENER or TAG
						{
							fcode = rxmsg->messageData[FCODE];
						}
#else
						fcode = rxmsg->messageData[FCODE];
#endif

                        switch(fcode)
                        {
							case RTLS_DEMO_MSG_RNG_INIT:
							{
								if(inst->mode == TAG_TDOA) //only start ranging with someone if not ranging already
								{
									double delay = rxmsg->messageData[RES_T1] + (rxmsg->messageData[RES_T2] << 8); //in ms
                            		inst->testAppState = TA_TXE_WAIT;
									inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll
									inst->relpyAddress = srcAddr; //remember the anchor address
									memcpy(&inst->msg.destAddr, &inst->relpyAddress, ADDR_BYTE_SIZE);

									inst->tagShortAdd = rxmsg->messageData[RES_R1] + (rxmsg->messageData[RES_R2] << 8) ;
									instancesetreplydelay(delay); //

									inst->mode = TAG ;
									inst->rxTimeouts = 0; //reset timeout count
								}
								//non_user_payload_len = RANGINGINIT_MSG_LEN;
                            }
                            break; //RTLS_DEMO_MSG_TAG_POLL

                            case RTLS_DEMO_MSG_TAG_POLL:
                            {
                                inst->tagPollRxTime = inst->rxu.rxTimeStamp ; //Poll's Rx time

                                inst->delayedReplyTime = inst->tagPollRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

                                //printf("PollRx Timestamp: %4.15e\n", convertdevicetimetosecu(inst->tagPollRxTime));
                                //printf("Delay: %4.15e\n", convertdevicetimetosecu(inst->delayedReplyTime));

                                inst->testAppState = TA_TXRESPONSE_WAIT_SEND ; // send our response

                                inst->relpyAddress = srcAddr; //remember who to send the reply to
                                //non_user_payload_len = TAG_POLL_MSG_LEN;
                            }
                            break; //RTLS_DEMO_MSG_TAG_POLL

                            case RTLS_DEMO_MSG_ANCH_RESP:
                            {
                                inst->tag2rxReport = rxmsg->messageData[RES_R2]; //check if the anchor is going to send the report
                                                                                 //if no report coming, go to sleep before sending the next poll

                                inst->rxu.anchorRespRxTime = inst->rxu.rxTimeStamp ; //Response's Rx time

                                inst->delayedReplyTime = inst->rxu.anchorRespRxTime + inst->fixedReplyDelay ;  // time we should send the response
                                inst->delayedReplyTime &= MASK_TXDTS ;

                                //printf("RespRx Timestamp: %4.15e\n", convertdevicetimetosecu(inst->rxu.anchorRespRxTime));
                                //printf("Delay: %4.15e\n", convertdevicetimetosecu(inst->delayedReplyTime));

                                inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final

                                inst->relpyAddress = srcAddr; //remember who to send the reply to
                                //non_user_payload_len = ANCH_RESPONSE_MSG_LEN;
                            }
                            break; //RTLS_DEMO_MSG_ANCH_RESP

                            case RTLS_DEMO_MSG_ANCH_TOFR:
                            {
                                    memcpy(&inst->tof, &(rxmsg->messageData[TOFR]), 5);

                                    if(rxmsg->seqNum != inst->lastReportSN)
                                    {
                                        reportTOF(inst);
                                        inst->newrange = 1;
                                        inst->lastReportSN = rxmsg->seqNum;
                                    }

                                    inst->testAppState = TA_TXE_WAIT;
                                    inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll
                                    inst->relpyAddress = srcAddr; //remember who to send the reply to

                                //non_user_payload_len = TOF_REPORT_MSG_LEN;

                                //dwt_forcetrxoff() ;

                            }
                            break; //RTLS_DEMO_MSG_ANCH_TOFR

                            case RTLS_DEMO_MSG_TAG_FINAL:
                            {
                                uint64 tRxT, tTxT, aRxT, aTxT ;
                                uint64 tagFinalTxTime  = 0;
                                uint64 tagFinalRxTime  = 0;
                                uint64 tagPollTxTime  = 0;
                                uint64 anchorRespRxTime  = 0;
                                uint64 pollRespRTD  = 0;
                                uint64 respFinalRTD  = 0;

                                // time of arrival of Final message
                                tagFinalRxTime = inst->rxu.rxTimeStamp ; //Final's Rx time

#if (FIXED_REPORT_DELAY > 0)
                                inst->delayedReplyTime = tagFinalRxTime + inst->fixedReportDelay ;
                                inst->delayedReplyTime &= MASK_TXDTS ;
#else
                                inst->delayedReplyTime = 0 ;
#endif

                                // times measured at Tag extracted from the message buffer
                                // extract 40bit times
                                memcpy(&tagPollTxTime, &(rxmsg->messageData[PTXT]), 5);
                                memcpy(&anchorRespRxTime, &(rxmsg->messageData[RRXT]), 5);
                                memcpy(&tagFinalTxTime, &(rxmsg->messageData[FTXT]), 5);

                                // poll response round trip delay time is calculated as
                                // (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
                                aRxT = (anchorRespRxTime - tagPollTxTime) & MASK_40BIT;
                                aTxT = (inst->txu.anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT;
                                pollRespRTD = (aRxT - aTxT) & MASK_40BIT;


                                // response final round trip delay time is calculated as
                                // (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
                                tRxT = (tagFinalRxTime - inst->txu.anchorRespTxTime) & MASK_40BIT;
                                tTxT = (tagFinalTxTime - anchorRespRxTime) & MASK_40BIT;
                                respFinalRTD = (tRxT - tTxT) & MASK_40BIT;

                                // add both round trip delay times
                                inst->tof = ((pollRespRTD + respFinalRTD) & MASK_40BIT);

                                { // work out clock offset
                                    double time = (inst->fixedReplyDelay_ms/1000.0); //convert to seconds
                                    double rtd1, rtd2, aveRTD, y;
                                    rtd1 = convertdevicetimetosec8((uint8*) &respFinalRTD) ;
                                    rtd2 = convertdevicetimetosec8((uint8*) &pollRespRTD) ;
                                    aveRTD = (rtd1 + rtd2) / 2.0 ; //average
                                    y = rtd1 - aveRTD ;
                                    inst->clockOffset = y / time ;
                                    inst->clockOffset *= 1e6 ; //in parts per million
                                }
                                reportTOF(inst);
                                inst->newrange = 1;
                                

                                if(inst->sendTOFR2Tag)
                                {
                                    inst->testAppState = TA_TXREPORT_WAIT_SEND ; // send the report with the calculated time of flight
                                    inst->newReportSent = 0; //set the new report flag
                                }
                                else
                                {
#if (DOUBLE_RX_BUFFER == 0)
                                    inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
#endif
                                    inst->shouldDoDelayedRx = FALSE ;               // no delay turning on RX
                                }
                                inst->relpyAddress = srcAddr; //remember who to send the reply to
                                //non_user_payload_len = TAG_FINAL_MSG_LEN;
                            }
                            break; //RTLS_DEMO_MSG_TAG_FINAL


                            default:
                            {
#if (DOUBLE_RX_BUFFER == 0)
                                //we got an unexpected function code...
                                inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
#endif
                                inst->shouldDoDelayedRx = FALSE ;               // no delay turning on RX

                            }
                            break;
                        } //end switch (rxmsg->functionCode)

                        if((inst->instToSleep == 0) && (inst->mode == LISTENER) /*|| (inst->mode == ANCHOR)*/)//update received data, and go back to receiving frames
                        {

                            //instancelogrxdata(inst);
#if (DOUBLE_RX_BUFFER == 0)
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
#endif
                            inst->shouldDoDelayedRx = FALSE ;               // no delay turning on RX
                        }

                    }
                }
                break ;

                case DWT_SIG_RX_TIMEOUT :
                    //printf("PD_DATA_TIMEOUT") ;
                {

                    inst->rxTimeouts ++ ;
                    inst->done = INST_NOT_DONE_YET;

                    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
                    {
                        if((inst->newReportSent) && (inst->newReportSent < MAX_NUMBER_OF_REPORT_RETRYS)) //no ACK send another report
                        {
                            inst->testAppState = TA_TXREPORT_WAIT_SEND ;
                            //printf("Timeout while waiting for ACK -> send next report\n");
                        }
                        else //finished sending reports - wait for next poll message
                        {
                            inst->testAppState = TA_RXE_WAIT ;
                            dwt_setrxtimeout(0);
                            inst->ackexpected = 0 ; //timeout... so no acks expected anymore
                        }
                    }

                    else //if(inst->mode == TAG)
                    {
                        // initiate the re-transmission of the poll that was not responded to
                        inst->testAppState = TA_TXE_WAIT ;

//NOTE: Do not go back to "discovery" mode -  needs a boards reset to go back to "discovery" mode once is starts ranging
#if 0
#if (DR_DISCOVERY == 1)						
						if((inst->mode == TAG) && (inst->rxTimeouts >= MAX_NUMBER_OF_POLL_RETRYS)) //if no response after sending 20 Polls - go back to blink mode
						{
							inst->mode = TAG_TDOA ;
						}
#endif
#endif
						if(inst->mode == TAG)
							inst->nextState = TA_TXPOLL_WAIT_SEND ;
#if (DR_DISCOVERY == 1)
						else //TAG_TDOA
							inst->nextState = TA_TXBLINK_WAIT_SEND ;
#endif
                    }

                    message = 0; //clear the message as we have processed the event

                    //timeout - disable the radio (if using SW timeout the rx will not be off)
                    dwt_forcetrxoff() ;
                }
                break ;

                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
                case 0: //no event - wait in receive...
                {
                    //stay in Rx (fall-through from above state)
                    //if(DWT_SIG_TX_AA_DONE == message) printf("Got SIG_TX_AA_DONE in RX wait - ignore\n");
                    if(inst->done == INST_NOT_DONE_YET) inst->done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

                default :
                {
                    //printf("\nERROR - Unexpected message %d ??\n", message) ;
                    //assert(0) ;                                             // Unexpected Primitive what is going on ?
                }
                break ;

            }
            break ; // end case TA_RX_WAIT_DATA

            default:
                //printf("\nERROR - invalid state %d - what is going on??\n", inst->testAppState) ;
            break;
    } // end switch on testAppState

    return inst->done;
} // end testapprun()

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else

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

int instancenewrange(void)
{
    if(instance_data[0].newrange)
    {
        instance_data[0].newrange = 0;
        return 1;
    }

    return 0;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    instance_data[instance].frame_sn = 0;
    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

    instance_data[instance].idistmax = 0;
    instance_data[instance].idistmin = 1000;

    dwt_configeventcounters(1); //enable and clear

    instance_data[instance].frame_sn = 0;
    instance_data[instance].lastReportSN = 0xff;

    instance_data[instance].tofcount = 0 ;
    instance_data[instance].tofindex = 0 ;

#if (DEEP_SLEEP == 1)
    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
#endif
} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
#if DECA_SUPPORT_SOUNDING==1
int instance_init(accBuff_t *buf)
#else
int instance_init(void)
#endif
{
    int instance = 0 ;
    int result;
    //uint16 temp = 0;

    instance_data[instance].shouldDoDelayedRx = FALSE ;

    instance_data[instance].mode = LISTENER ;                                // assume listener,
    instance_data[instance].testAppState = TA_INIT ;

    instance_data[instance].anchorListIndex = 0 ;
    instance_data[instance].instToSleep = 0;

    instance_data[instance].sentSN = 0;
    instance_data[instance].ackdSN = 0;


    instance_data[instance].tofindex = 0;
    instance_data[instance].tofcount = 0;
    instance_data[instance].last_update = -1 ;           // detect changes to status report

    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();

#if (DEEP_SLEEP_AUTOWAKEUP == 1)
#if 1
    {
        double t;
        instance_data[instance].lp_osc_cal = dwt_calibratesleepcnt(); //calibrate low power oscillator
        //the lp_osc_cal value is number of XTAL/2 cycles in one cycle of LP OSC
        //to convert into seconds (38.4MHz/2 = 19.2MHz (XTAL/2) => 1/19.2MHz ns)
        //so to get a sleep time of 5s we need to program 5 / period and then >> 12 as the register holds upper 16-bits of 28-bit counter
        t = ((double)5/((double)instance_data[instance].lp_osc_cal/19.2e6));
        instance_data[instance].blinktime = (int) t;
        instance_data[instance].blinktime >>= 12;

        dwt_configuresleepcnt(instance_data[instance].blinktime);//configure sleep time

    }
#else
    instance_data[instance].blinktime = 0xf;
#endif
#endif

    //we can enable any configuration loding from NVM/ROM on initialisation
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    //temp = dwt_readtempvbat();
    //printf("Vbat = %d (0x%02x) \tVtemp = %d  (0x%02x)\n",temp&0xFF,temp&0xFF,(temp>>8)&0xFF,(temp>>8)&0xFF);

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);

    //this is platform dependant - only program if DW EVK/EVB
    dwt_setleds(2) ; //configure the GPIOs which control the leds on EVBs

    //this is set though the instance_data[instance].configData.smartPowerEn in the instance_config function
    //dwt_setsmarttxpower(0); //disable smart TX power


    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

    instanceclearcounts() ;

    instance_data[instance].panid = 0xdeca ;

#if (DR_DISCOVERY == 1)
    instance_data[instance].sendTOFR2Tag = 1;
#else
    instance_data[instance].sendTOFR2Tag = 0;
#endif

    instance_data[instance].tag2rxReport = 0; //Tag is not expecting the report

    instance_data[instance].fixedReportDelay = (convertmicrosectodevicetimeu (FIXED_REPORT_DELAY * 1000.0) & MASK_TXDTS);
    instance_data[instance].fixedReplyDelay = (convertmicrosectodevicetimeu (FIXED_REPLY_DELAY * 1000.0) & MASK_TXDTS);
    instance_data[instance].fixedReplyDelay_ms = FIXED_REPLY_DELAY ;
    instance_data[instance].rxOnDelay = convertmicrosectodevicetimeu ((FIXED_REPLY_DELAY - RX_ON_TIME) * 1000.0);
	instance_data[instance].tagBlinkSleepTime_ms = BLINK_SLEEP_DELAY ;

    instance_data[instance].newReportSent = 0; //clear the flag
    instance_data[instance].wait4ack = 0;
    instance_data[instance].ackexpected = 0;
    instance_data[instance].stoptimer = 0;
    instance_data[instance].instancetimer_en = 0;

    instance_data[instance].dwevent[0] = 0;
    instance_data[instance].dwevent[1] = 0;
    instance_data[instance].dweventCnt = 0;

    instance_data[instance].anchReportTimeout_ms = 10 ; //10 ms

    instance_data[instance].rxautoreenable = 0;

    //sample test calibration functions
    //xtalcalibration();
    //powertest();

	dwt_geteui(instance_data[instance].eui64);
    return 0 ;
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
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    int use_nvmdata = DWT_LOADANTDLY | DWT_LOADXTALTRIM;
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
    instance_data[instance].configData.sfdTO = DWT_SFDTOC_DEF; //default value

    instance_data[instance].configData.smartPowerEn = 0;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, use_nvmdata) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

	//firstly check if there are calibrated TX power value in the DW1000 OTP
	power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);
	
	if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    //Configure TX power
	//if smart power is used then the value as read from NVM is used directly
	//if smart power is used the user needs to make sure to transmit only one frame per 1ms or TX spectrum power will be violated
    if(instance_data[instance].configData.smartPowerEn == 1)
    {
        instance_data[instance].configTX.power = power;
    }
	else //if the smart power if not used, then the low byte value (repeated) is used for the whole TX power register
    {
        uint8 pow = power & 0xFF ;
        instance_data[instance].configTX.power = (pow | (pow << 8) | (pow << 16) | (pow << 24));
    }
	dwt_setsmarttxpower(instance_data[instance].configData.smartPowerEn);

	//configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

	//check if to use the antenna delay calibration values as read from the NVM
    if((use_nvmdata & DWT_LOADANTDLY) == 0)
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


}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(int sleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_ms = sleepdelay ;
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed blink reply delay time (in ms)
//
void instancesetblinkreplydelay(double delayms) //delay in ms
{
	int instance = 0 ;
	instance_data[instance].fixedReplyDelay_ms = delayms ;
}
// -------------------------------------------------------------------------------------------------------------------
// function to set the fixed reply delay time (in ms)
//
void instancesetreplydelay(double delayms) //delay in ms
{
    int instance = 0 ;
    instance_data[instance].fixedReplyDelay = convertmicrosectodevicetimeu (delayms * 1e3) ;
    instance_data[instance].fixedReplyDelay_ms = delayms ;
    instance_data[instance].rxOnDelay = convertmicrosectodevicetimeu ((delayms - RX_ON_TIME) * 1e3);
    //printf("Set response delay time to %d ms.\n", (int) delayms);
}

// -------------------------------------------------------------------------------------------------------------------
// function to configure anchor instance whether to send TOF reports to Tag
//
void instancesetreporting(int anchorSendsTofReports)
{
    int instance = 0 ;
    instance_data[instance].sendTOFR2Tag = anchorSendsTofReports ;        // Set whether TOF reports are sent
    //Listener will only listen for reports when this is set (all other frames are filtered out)
    //instance_data[instance].listen4Reports = anchorSendsTofReports ;
}

#if (DR_DISCOVERY == 0)
// -------------------------------------------------------------------------------------------------------------------
//
// Set Payload parameters for the instance
//
// -------------------------------------------------------------------------------------------------------------------
void instancesetaddresses(instanceAddressConfig_t *plconfig)
{
    int instance = 0 ;

    instance_data[instance].payload = *plconfig ;       // copy configurations

    if(instance_data[instance].payload.sendReport == 1)
        instance_data[instance].sendTOFR2Tag = 1;
    else
        instance_data[instance].sendTOFR2Tag = 0;
}
#endif

// -------------------------------------------------------------------------------------------------------------------
//
// Access for low level debug
//
// -------------------------------------------------------------------------------------------------------------------


void instance_close(void)
{
#if ST_MC
#ifdef _MSC_VER
    uint8 buffer[1500];
    // close/tidy up any device functionality - i.e. wake it up if in sleep mode
    if(dwt_spicswakeup(buffer, 1500) == DWT_ERROR)
    {
        //printf("FAILED to WAKE UP\n");
    }
#else
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    port_SPIx_clear_chip_select();  //CS low
    Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    port_SPIx_set_chip_select();  //CS high
    Sleep(5);
#endif
#endif

    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_txcallback(const dwt_callback_data_t *txd)
{
    int instance = 0;
    uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
    uint32 temp = 0;
    uint8 txevent = txd->event;

    if(instance_data[instance].ackreq) //the ACK has been requested in the last RX frame - we got TX event, this means the ACK has been sent
    {
        txevent = DWT_SIG_TX_AA_DONE;
        instance_data[instance].ackreq = 0;
    }

    if(txevent == DWT_SIG_TX_DONE)
    {
        //uint64 txtimestamp = 0;

        if(instance_data[instance].dweventCnt < 2) //if no outstanding event to process
        {
            //NOTE - we can only get TX good (done) while here
            //dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);

            dwt_readtxtimestamp(txTimeStamp) ;
            temp = txTimeStamp[0] + (txTimeStamp[1] << 8) + (txTimeStamp[2] << 16) + (txTimeStamp[3] << 24);
            instance_data[instance].txu.txTimeStamp = txTimeStamp[4];
            instance_data[instance].txu.txTimeStamp <<= 32;
            instance_data[instance].txu.txTimeStamp += temp;

            instance_data[instance].stoptimer = 0;

            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_TX_DONE ;
        }


#if (DEEP_SLEEP == 1)
        instance_data[instance].txmsgcount++;
#endif
        //printf("TX time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
        //printf("TX Timestamp: %4.15e\n", convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp));
#if (DECA_SUPPORT_SOUNDING==1)
    #if DECA_ACCUM_LOG_SUPPORT==1
        if ((instance_data[instance].devicelogdata.accumLogging == LOG_ALL_ACCUM ) || (instance_data[instance].devicelogdata.accumLogging == LOG_ALL_NOACCUM ))
        {
            fprintf(instance_data[instance].devicelogdata.accumLogFile,"\nTX Frame TimeStamp Raw  = %04X %08X\n",(txtimestamp >> 32),txtimestamp & 0xffffffff) ;
            fprintf(instance_data[instance].devicelogdata.accumLogFile,"   Adding Antenna Delay = %04X %08X\n",instance_data[instance].txu.txTimeStamp >> 32,instance_data[instance].txu.txTimeStamp & 0xffffffff) ;
            fprintf(instance_data[instance].devicelogdata.accumLogFile,"%02X Tx time = %4.15e\n", instance_data[instance].msg.seqNum, convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp)) ;
        }
    #endif
#endif
    }
    else if(txevent == DWT_SIG_TX_AA_DONE)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding event to process
    {
            //auto ACK confirmation
            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_TX_AA_DONE ;

    }
        //else
        //{
            //printf("TX SIG_TX_AA_DONE - ERROR event ecount >= 2  (%d) state %d\n", instance_data[instance].eventCnt, instance_data[instance].testAppState);
        //}
        //printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].eventCnt);
    }
}

int instance_getevent(void)
{
    return instance_data[0].dwevent[0] ; //get the 1st event
}

void instance_rxcallback(const dwt_callback_data_t *rxd)
{
    int instance = 0;
    uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
	//int blink = 0;
    uint32 temp = 0;
    uint8 rxd_event = 0;
    int bufferfull = 0;

    if(rxd->event == DWT_SIG_RX_OKAY)
    {
        uint8 buffer[2];

        instance_data[instance].ackreq = rxd->aatset;

        //if(instance_data[instance].ackreq) //this indicates that the ACK request has been set
        //{
            //instance_data[instance].ackreq = rxd->aatset;
            //configuration and sending of ACK frame can be done here.... (when not using auto-ACK feature)
            //  printf("ACK request\n");
        //}

        dwt_readrxdata(buffer, 1, 0);  // Read Data Frame
        //at the moment using length and 1st byte to distinguish between different fame types and "blinks".
        switch(rxd->datalength)
        {
            case 5:
                rxd_event = SIG_RX_ACK;
                break;
            case 12:
                if(buffer[0] == 0xC5)
                {
                    rxd_event = SIG_RX_BLINK;
					//blink = 1;
                }
                break;
            case 18: //blink with Temp and Bat level
                if(buffer[0] == 0xC5)
                {
                    rxd_event = SIG_RX_BLINK;
                    //blinkdw = 1;
                }
                break;
            default:
                rxd_event = DWT_SIG_RX_OKAY;
                break;
        }


    if(rxd_event == DWT_SIG_RX_OKAY)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
        {
            //dwt_readrxtimestamp((uint8*) &instance_data[instance].rxu.rxTimeStamp) ;

            dwt_readrxtimestamp(rxTimeStamp) ;
            temp =  rxTimeStamp[0] + (rxTimeStamp[1] << 8) + (rxTimeStamp[2] << 16) + (rxTimeStamp[3] << 24);
            instance_data[instance].rxu.rxTimeStamp = rxTimeStamp[4];
            instance_data[instance].rxu.rxTimeStamp <<= 32;
            instance_data[instance].rxu.rxTimeStamp += temp;

            instance_data[instance].rxLength = rxd->datalength;

            dwt_readrxdata((uint8 *)&instance_data[instance].rxmsg, rxd->datalength, 0);  // Read Data Frame

            //instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window

            //dwt_readdignostics(&instance_data[instance].devicelogdata.diag);

            instance_data[instance].stoptimer = 1;

            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_RX_OKAY;

            #if DECA_LOG_ENABLE==1
            #if DECA_KEEP_ACCUMULATOR==1
            {
                instance_data[instance].newAccumData = 1 ;
                instance_data[instance].erroredFrame = DWT_SIG_RX_NOERR ;   //no error
                processSoundingData();
            }
            #endif
                logSoundingData(DWT_SIG_RX_NOERR);
            #endif

            //printf("RX OK %d ", instance_data[instance].testAppState);
            //printf("RX time %f\n",convertdevicetimetosecu(instance_data[instance].rxu.rxTimeStamp));
        }
        else
        {
        	bufferfull = 1;
        }


#if (DEEP_SLEEP == 1)
        instance_data[instance].rxmsgcount++;
#endif
    }
    else if (rxd_event == SIG_RX_ACK)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
        {
        	dwt_readrxdata((uint8 *)&instance_data[instance].rxackmsg, rxd->datalength, 0);  // Read Data Frame

        	instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = SIG_RX_ACK;
        }
        else
        {
        	bufferfull = 2;
        }

#if (DEEP_SLEEP == 1)
        instance_data[instance].rxmsgcount++;
#endif
    }
    else if (rxd_event == SIG_RX_BLINK)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
        {
			instance_data[instance].rxLength = rxd->datalength;

			dwt_readrxtimestamp(rxTimeStamp) ;
			temp =  rxTimeStamp[0] + (rxTimeStamp[1] << 8) + (rxTimeStamp[2] << 16) + (rxTimeStamp[3] << 24);
			instance_data[instance].rxu.rxTimeStamp = rxTimeStamp[4];
			instance_data[instance].rxu.rxTimeStamp <<= 32;
			instance_data[instance].rxu.rxTimeStamp += temp;

			if(rxd->datalength == 12)
			{
				dwt_readrxdata((uint8 *)&instance_data[instance].blinku.rxblinkmsg, rxd->datalength, 0);  // Read Data Frame
				instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = SIG_RX_BLINK;
			}
			else
			{
				dwt_readrxdata((uint8 *)&instance_data[instance].blinku.rxblinkmsgdw, rxd->datalength, 0);  // Read Data Frame
				instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = SIG_RX_BLINKDW;
			}

			//instance_readaccumulatordata();     // for diagnostic display in DecaRanging PC window

			//dwt_readdignostics(&instance_data[instance].devicelogdata.diag);
		}
		else
		{
			bufferfull = 4;
		}
#if (DEEP_SLEEP == 1)
        instance_data[instance].rxmsgcount++;
#endif
    }
	}
    else if (rxd->event == DWT_SIG_RX_TIMEOUT)
    {
        if(instance_data[instance].dweventCnt < 2) //if no outstanding events to process
        {
            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_RX_TIMEOUT;
            //printf("RX timeout while in %d count %d\n", instance_data[instance].testAppState, instance_data[instance].eventCnt);
        }
        else
        {
        	bufferfull = 8;
        }
            //printf("RX timeout ignored !!! %d (count %d) \n", instance_data[instance].testAppState, instance_data[instance].eventCnt);
    }
    else //assume other events are errors
    {
        //printf("RX error %d \n", instance_data[instance].testAppState);
        if(instance_data[instance].rxautoreenable == 0)
        {
            //re-enable the receiver
            instancerxon(0, 0); //immediate enable

        }

    }

    if(bufferfull > 0) //buffer full re-enable receiver
    {
    	bufferfull = 10;

#if (DOUBLE_RX_BUFFER == 0)
    	dwt_forcetrxoff() ;
    	dwt_rxenable(0) ;
#endif

    }
}


// -------------------------------------------------------------------------------------------------------------------
double instance_get_ldist(void) //get long term average range
{
    double x = inst_ldist;

    return (x);
}

int instance_get_lcount(void) //get count of ranges used for calculation of lt avg
{
    int x = instance_data[0].longTermRangeCount;

    return (x);
}

double instance_get_idist(void) //get instantaneous range
{
    double x = inst_idist;

    return (x);
}

int instance_get_rxf(void) //get number of Rxed frames
{
    int x = instance_data[0].rxmsgcount;

    return (x);
}

int instance_get_txf(void) //get number of Txed frames
{
    int x = instance_data[0].txmsgcount;

    return (x);
}


double instance_get_adist(void) //get average range
{
    double x = inst_adist;

    return (x);
}


int instance_readaccumulatordata(void)
{
#if DECA_SUPPORT_SOUNDING==1
    int instance = 0;
    uint16 len = 992 ; //default (16M prf)

    if (instance_data[instance].configData.prf == DWT_PRF_64M)  // Figure out length to read
        len = 1016 ;

    instance_data[instance].buff.accumLength = len ;                                       // remember Length, then read the accumulator data

    len = len*4+1 ;   // extra 1 as first byte is dummy due to internal memory access delay

    dwt_readaccdata((uint8*)&(instance_data[instance].buff.accumData->dummy), len, 0);
#endif  // support_sounding
    return 0;
}

// -------------------------------------------------------------------------------------------------------------------


int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    //int update ;
    //int cnt ;
    int message = instance_data[instance].dwevent[0];

    {
        while(done == INST_NOT_DONE_YET)
        {
            //int state = instance_data[instance].testAppState;
            done = testapprun(&instance_data[instance], message) ;                                               // run the communications application

            if(message) // there was an event in the buffer
            {
                instance_data[instance].dwevent[0] = 0; //clear the buffer
                instance_data[instance].dweventCnt--;
                //printf("process event %d in (%d) ecount %d\n", message, state, instance_data[instance].dweventCnt);

                if(instance_data[instance].dwevent[1]) // there is another event in the buffer move it to front
				{
                	instance_data[instance].dwevent[0] = instance_data[instance].dwevent[1];
					instance_data[instance].dwevent[1] = 0; //clear the buffer
					//instance_data[instance].dweventCnt = 1;
				}
            }

            //we've processed message
            message = 0;
        }

    }

    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
        if(instance_data[instance].mode == TAG) //Tag (is either in RX or sleeping)
        {
			instance_data[instance].instancetimer = portGetTickCount() + instance_data[instance].tagSleepTime_ms; //start timer
            instance_data[instance].instancetimer_en = 1;
            //printf("sleep timer on %d\n", instance_data[instance].testAppState);
        }
		if(instance_data[instance].mode == TAG_TDOA)
		{
			instance_data[instance].instancetimer = portGetTickCount() + instance_data[instance].tagBlinkSleepTime_ms; //start timer
			instance_data[instance].instancetimer_en = 1;
		}
        if(instance_data[instance].mode == ANCHOR) //Anchor (to timeout Anchor after sending of the ToF report )
        {
            instance_data[instance].instancetimer = portGetTickCount() + instance_data[instance].anchReportTimeout_ms; //start timer
            instance_data[instance].instancetimer_en = 1;
            //printf("report timer on %d\n", instance_data[instance].testAppState);
        }
        instance_data[instance].stoptimer = 0 ;
        instance_data[instance].done = INST_NOT_DONE_YET;
    }

    if((instance_data[instance].instancetimer_en == 1) && (instance_data[instance].stoptimer == 0))
    {
        if(instance_data[instance].instancetimer < portGetTickCount())
        {
            instance_data[instance].instancetimer_en = 0;
            instance_data[instance].dwevent[instance_data[instance].dweventCnt++] = DWT_SIG_RX_TIMEOUT;
            //printf("PC time out %d ecount %d\n",instance_data[instance].testAppState, instance_data[instance].dweventCnt);
        }
    }

    return 0 ;
}

#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
