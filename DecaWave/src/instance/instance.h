// -------------------------------------------------------------------------------------------------------------------
//
//  File: instance.h - DecaWave header for application level instance
//
//  Copyright (c) 2008 - DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Billy Verso, October 2008
//
// -------------------------------------------------------------------------------------------------------------------

#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "instance_sws.h"
#include "deca_types.h"
#include "deca_device_api.h"

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // MP counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits. 


#define USING_64BIT_ADDR (1) //when set to 0 - the DecaRanging application will use 16-bit addresses

#if (DR_DISCOVERY == 0) && (USING_64BIT_ADDR == 0)
	#pragma message (__FILE__" When DISCOVERY mode is not used the application requires that 64-bit addresses are used.")
#endif

#define SIG_RX_ACK				5		// Frame Received is an ACK (length 5 bytes)
#define SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define SIG_RX_BLINKDW			8		// Received ISO EUI 64 DW blink message
#define SIG_RX_UNKNOWN			99		// Received an unknown frame

//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    3				// FunctionCode(1), Temp (1), Volt (1)
#define ANCH_RESPONSE_MSG_LEN               4               // FunctionCode(1), RespOption (1), OptionParam(2)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define TOF_REPORT_MSG_LEN                  6               // FunctionCode(1), Measured_TOF_Time(5)
#define RANGINGINIT_MSG_LEN					5				// FunctionCode(1), Tag Address (2), Response Time (2)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

#define BLINK_FRAME_CONTROL_BYTES       1
#define BLINK_FRAME_SEQ_NUM_BYTES       1
#define BLINK_FRAME_CRC					2
#define BLINK_FRAME_SOURCE_ADDRESS      8
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes

#define ANCHOR_LIST_SIZE			(4)
#define TAG_LIST_SIZE				(10)

#define SEND_TOF_REPORT				(1)
#define NO_TOF_REPORT				(0)

#define BLINK_SLEEP_DELAY					1000 //ms
#define POLL_SLEEP_DELAY					400 //ms
#define MAX_NUMBER_OF_REPORT_RETRYS (3)         // max number of times to send/re-send the report message
#define MAX_NUMBER_OF_POLL_RETRYS (20)			// max number of times to send poll and don't get a response, after which the tag will go back to sending blinks
#define INST_DONE_WAIT_FOR_NEXT_EVENT		1	//this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO	2	//this signifies that the current event has been processed and that instance is waiting for next one with a timeout
												//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET					0	//this signifies that the instance is still processing the current event

#define ACK_REQUESTED                       (1)				// Request an ACK frame

#define FIXED_REPLY_DELAY						150 //ms  //response delay time (Tag or Anchor when sending Final/Response messages respectively)
#define FIXED_LONG_BLINK_RESPONSE_DELAY			(5*FIXED_REPLY_DELAY) //NOTE: this should be a multiple of FIXED_LONG_REPLY_DELAY see DELAY_MULTIPLE below
#define DELAY_MULTIPLE				(FIXED_LONG_BLINK_RESPONSE_DELAY/FIXED_REPLY_DELAY - 1)


typedef enum instanceModes{LISTENER, TAG, ANCHOR, TAG_TDOA, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
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

    TA_SLEEP_DONE,              //9
    TA_TXBLINK_WAIT_SEND,       //10
    TA_TXRANGINGINIT_WAIT_SEND,  //11
	TA_PAUSED,
	TA_LAST_STATE
} INST_STATES;

typedef struct _Code2String {
	uint8 state;
	uint8 name[30];
}Code2String;

// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit address
    uint8 fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blink_msg ;

//18 octets for IEEE ID blink with Temp and Vbat values
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];           			//  02-09 64 bit addresses
	uint8 enchead[2];								//  10-11 2 bytes (encoded header and header extension)
	uint8 messageID;								//  12 message ID (0xD1) - DecaWave message
	uint8 temp;										//  13 temperature value
	uint8 vbat;										//  14 voltage value
	uint8 gpio;										//  15 gpio status
    uint8 fcs[2] ;                              	//  16-17  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blinkdw_msg ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 fcs[2] ;                              	//  03-04  CRC
} ack_msg ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    //uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

typedef struct
{
	uint64 forwardToFRAddress;
    uint64 anchorAddress;
	uint64 *anchorAddressList;
	//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
    char payloadString[MAX_USER_PAYLOAD_STRING+2] ;                 // user supplied payload string
    int countAppend ;                                               // Flag append a counter
	int payloadLen ;
	int anchorListSize ;
	int anchorPollMask ;
	int sendReport ;
} instancePayloadConfig_t ;

typedef struct
{
	uint32 icid;

	dwt_rxdiag_t diag;

#if DECA_LOG_ENABLE==1
    int         accumLogging ;                                // log data to a file, used to indicate that we are currenty logging (file is open)
	FILE        *accumLogFile ;                               // file
#endif

} devicelogdata_t ;

/******************************************************************************************************************
**************** DATA STRUCTURES USED IN PC APPLICATION LOGGING/DISPLAYING ****************************************************************
*******************************************************************************************************************/

#define MAX_ACCUMULATOR_ELEMENTS	(1016*2)
#define RX_ACCD_MAX_LEN					(9000)            // max accumulator length in bytes
typedef struct
{
	uint8       spi_read_padding ;
    uint8       dummy ;                                 // extra 1 as first byte is dummy due to internal memory access delay
    uint8       accumRawData[RX_ACCD_MAX_LEN] ;         // data bytes
} accBuff_t ;

typedef struct deca_buffer
{
	uint16      accumLength ;                       // length of data actually present (number of complex samples)
#if DECA_SUPPORT_SOUNDING==1
    accBuff_t   *accumData ;						// data bytes
#endif
} deca_buffer_t ;

typedef struct {        int16   real ;
                        int16   imag ;  } complex16 ;

typedef struct
{
#if (DECA_KEEP_ACCUMULATOR==1)
    complex16 accvals[MAX_ACCUMULATOR_ELEMENTS] ;
	int32  magvals [MAX_ACCUMULATOR_ELEMENTS] ;
#else
	complex16 *accvals ;
	int32 *magvals;
#endif
    int accumSize ;

    int16 windInd ;
    uint16 preambleSymSeen ;
	uint16 maxGrowthCIR ;
    double hwrFirstPath ;
    double totalSNR ;
    double RSL ;
    double totalSNRavg ;
    double RSLavg ;
    double fpSNR ;
    double fpRelAtten ;

	char cp[8];
} accumPlotInfo_t ;

typedef struct
{
    #if DECA_SUPPORT_SOUNDING==1
	#if DECA_KEEP_ACCUMULATOR==1
    uint16					accumLength ;
    complex16				accumCmplx[MAX_ACCUMULATOR_ELEMENTS] ;
	int8                    newAccumData ;                                // flag data present
    int8                    erroredFrame ;                                // flag was errored frame
	//uint8					preambData[1024];
	deca_buffer_t			buff ;
	#endif
	#else
	uint8 dummy;
    #endif
} deviceacclogdata_t ;

// number of lines for status report
#define NUM_STAT_LINES              6           // total number of status lines
#define NUM_BASIC_STAT_LINES        4           // first group of four lines
#define STAT_LINE_MSGCNT            0
#define STAT_LINE_INST              1
#define STAT_LINE_MEAN              2
#define STAT_LINE_LTAVE             3
// #define STAT_LINE_RESP              4
#define STAT_LINE_USER_RX1          4
#define STAT_LINE_USER_RX2          5
// character length of lines for status report
#define STAT_LINE_LENGTH    160
#define STAT_LINE_LONG_LENGTH 1024
#define STAT_LINE_BUF_LENGTH    (STAT_LINE_LENGTH+1) // allow space for NULL string
#define STAT_LINE_MSGBUF_LENGTH    ((STAT_LINE_LONG_LENGTH*2)+1+5) // allow space for NULL string + 10 for (bytes) and *2 because of 2chars per byte printing

#define NUM_MSGBUFF_POW2        5
#define NUM_MSGBUFF_LINES       (1<<NUM_MSGBUFF_POW2)       // NB: power of 2 !!!!!!!!
#define NUM_MSGBUFF_MODULO_MASK (NUM_MSGBUFF_LINES-1)

typedef struct
{
    char scr[NUM_STAT_LINES][STAT_LINE_BUF_LENGTH] ;            // screen layout
	char lmsg_scrbuf[NUM_MSGBUFF_LINES][STAT_LINE_MSGBUF_LENGTH] ;      // lmsg_scrbuf listener message log screen buffer
    int lmsg_last_index ; 
    int changed ;
} statusReport_t ;

typedef struct
{
    double idist ;
    double mean4 ; 
    double mean ; 
    int numAveraged ;
	uint32 lastRangeReportTime ;
} currentRangeingInfo_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define MAX_EVENT_NUMBER (5)
//NOTE: Accumulators don't need to be stored as part of the event structure as when reading them only one RX event can happen... 
//the receiver is singly buffered and will stop after a frame is received

typedef struct
{
	uint8  type;			// event type
	uint8  type2;			// holds the event type - does not clear (not used to show if event has been processed)
	uint8  broadcastmsg;	// specifies if the rx message is broadcast message
	uint16 rxLength ;

	uint64 timeStamp ;		// last timestamp (Tx or Rx)
	
	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ; 
			srd_msg_dlss rxmsg_ls ; 
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
			ack_msg rxackmsg ; //holds received ACK frame 
			iso_IEEE_EUI64_blink_msg rxblinkmsg;
			iso_IEEE_EUI64_blinkdw_msg rxblinkmsgdw;
	}msgu;

}event_data_t ;

#define RTD_MED_SZ          8      // buffer size for mean of 8
#define RTD_STDEV_SZ		50	   // buffer size for STDEV of 50
//TX power configuration structure... 
typedef struct {
                uint8 PGdelay;

				//TX POWER
				//31:24		BOOST_0.125ms_PWR
				//23:16		BOOST_0.25ms_PWR-TX_SHR_PWR
				//15:8		BOOST_0.5ms_PWR-TX_PHR_PWR
				//7:0		DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; // 
}tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag, anchor or listener)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
    INST_STATES prevStateDebug;
    int done ;					//done with the current event/wait for next event to arrive
    
	int pauseRequest ;			//user has paused the application 

	
	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration 
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration 
	uint16			txantennaDelay ; //DW1000 TX antenna delay
	// "MAC" features 
	uint8 rxautoreenable;			//auto RX re-enable (the receiver will re-enable on an errored frame)
	uint8 doublebufferon;			//double buffer is enabled
    uint8 frameFilteringEnabled ;	//frame filtering is enabled


	//user payload and address structure for non-discovery mode
	instancePayloadConfig_t payload ;
    uint32 payloadCounter ;

	//timeouts and delays
	int tagSleepTime_ms; //in milliseconds
	int tagBlinkSleepTime_ms;
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64 fixedReplyDelay ;
	int fixedReplyDelay_ms ;

	// xx_sy the units are 1.0256 us
	int fixedReplyDelay_sy ;    // this is the delay used after sending a poll or response and turnign on the reciver to receive response or final
	int rnginitW4Rdelay_sy ;	// this is the delay used after sending a blink and turning on the receiver to receive the ranging init message
	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)
	int fwtoTimeB_sy ;	//this is the ranging init message duration

	uint64 delayedReplyTime;		// delayed reply time of ranging-init/response/final message
    uint32 rxTimeouts ;
	uint32 responseTimeouts ;

	//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)	
	srd_msg_dlsl rng_initmsg ;	// ranging init message (destination long, source long)
    srd_msg_dlsl msg ;			// simple 802.15.4 frame structure (used for tx message) - using long addresses
#else
	srd_msg_dlss rng_initmsg ;  // ranging init message (destination long, source short)
    srd_msg_dsss msg ;			// simple 802.15.4 frame structure (used for tx message) - using short addresses
#endif
	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

	//Tag function address/message configuration 
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  tagShortAdd ;		    // Tag's short address (16-bit) used when USING_64BIT_ADDR == 0
	uint16  psduLength ;			// used for storing the frame length
    uint8   frame_sn;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion 
	uint16  panid ;					// panid used in the frames

	//union of TX timestamps 
	union {
		uint64 txTimeStamp ;		   // last tx timestamp
		uint64 tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;

	uint64 anchorRespRxTime ;	    // receive time of response message
	uint64 tagPollRxTime ;          // receive time of poll message


	//application control parameters 
	uint8	ackreq;					// set if the last RX message had ACK request bit set - it is cleared when the ACK tx confirmation is processed 
									// the received frame with the ACK request bit set will only be processed once the ACK has been sent
    uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
    uint8	ackexpected ;			// application has sent ACK request and is waiting for an ACK
	uint8	ackTO ;					// using ACK timeout (this means that after an ACK request the RX will timeout in ~5ms) 
	uint8	newReportSent;			// count of number of sent reports
    uint8	sentSN;					// sent sequence number 
    uint8	sendTOFR2Tag ;			// sends report to Tag else forwards to Forwarding Address
    uint8	tag2rxReport ;			// tag should get ready to rx report after final message is sent
	
	uint8   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message
	uint8	stoptimer;				// stop/disable an active timer
    uint8	instancetimer_en;		// enable/start a timer
    uint32	instancetimer;			// e.g. this timer is used to timeout Tag when in deep sleep so it can send the next poll message
	uint8	deviceissleeping;		// this disabled reading/writing to DW1000 while it is in sleep mode 
									// (DW1000 will wake on chip select so need to disable and chip select line activity)
	uint8	gotTO;					// got timeout event

	uint8   tagDeepSleep;

    //diagnostic counters/data, results and logging 
    uint64 tof ;
	double clockOffset ;
	
	int blinkRXcount ;
	int txmsgcount;
	int	rxmsgcount;
	int lateTX;
	int lateRX;

	double  astdevdist[RTD_STDEV_SZ] ;
    double	adist[RTD_MED_SZ] ;
    double	adist4[4] ;
    double	longTermRangeSum ;
    int		longTermRangeCount ;
    int		tofindex ;
	int     tofstdevindex ;
    int		tofcount ;
    uint8	lastReportSN ;
    uint8   dispFeetInchesEnable ;                         // Display Feet and Inches
	uint8   dispClkOffset ;								// Display Clock Offset
    double	idistmax;
    double	idistmin;
    double	idistance ; // instantaneous distance
    double	shortTermMean ;
    double	shortTermMean4 ;
    int		shortTermMeanNumber ;
    int		newrange;
	uint32	lastReportTime;
	double  RSL;
	double  FSL;

    //logging data/structures used for data/graphical displaying for GUI
    statusReport_t statusrep;
    int last_update ;           // detect changes to status report

	devicelogdata_t dwlogdata;

	deviceacclogdata_t dwacclogdata ;

	uint8 tagToRangeWith;	//it is the index of the tagList array which contains the address of the Tag we are ranging with
    uint8 tagListLen ;
    uint8 anchorListIndex ;
	uint8 tagList[TAG_LIST_SIZE][8];


	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
	event_data_t saved_dwevent; //holds an RX event while the ACK is being sent
    uint8 dweventCntOut;
    uint8 dweventCntIn;
	uint8 dweventPeek;

} instance_data_t ;


//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to clear the RX message buffer used for frame display in listener mode
void clearlogmsgbuffer(void) ;
// function to calculate and report the Time of Flight to the GUI/display
void reportTOF(instance_data_t *inst);
// function to clear the ToF data structures
void clearreportTOF(statusReport_t *st);
// clear the status/ranging data 
void instanceclearcounts(void) ;
// copy new data to display
void copylonglinetoscrline(char *scrline);
// copy new data to display
void copylongline2scrline(char *scrla,char *scrlb)  ;                // allow copy to wrap into a 2nd status line
// display new status data
void instancedisplaynewstatusdata(instance_data_t *inst, dwt_deviceentcnts_t *itemp);
// display the rx message data
void instancelogrxdata(instance_data_t *inst, uint8 *buffer, int length);
void instancelogrxblinkdata(instance_data_t *inst, event_data_t *dw_event);
// display the user payload from the rx message data
void instancelogrxpayload(statusReport_t *statusrep, uint8* srcadd, int srclen, int rxlen, int uplen, uint8* messageData);
// display the current range info (instantaneous, long term average, mean)
void instancegetcurrentrangeinginfo(currentRangeingInfo_t *info) ;

uint8* instgettaglist(int i);
uint8 instgettaglistlength(void) ;
uint32 instgetblinkrxcount(void) ;
void instcleartaglist(void);
int instgettaglistlen(void);
void instsettagtorangewith(int tagID);
int instaddtagtolist(instance_data_t *inst, uint8 *tagAddr);
instance_data_t* getInstanceData(int index);


// enable reading of the accumulator - used for displaying channel impulse response
void instancesetdisplayfeetinches(int feetInchesEnable) ;           // status report distance in feet and inches
void instancesetdisplayclockoffset(int clockOff) ;					// enable/disable clock offset display
int instancenewplotdata(void) ;                                     // Is there new plot data to display, (returns 1 if yes, otherwise 0)
void instance_calculatepower(void);
// sets size and pointer to its accumulator data, and first path values, return string indicates Error Type or OKAY.
const char * instancegetaccumulatordata(accumPlotInfo_t *ap) ;

#if (DECA_ACCUM_LOG_SUPPORT==1) || (DECA_LOG_ENABLE==1)
#include <stdio.h>
#define LOG_SINGLE_ACC      4
#define LOG_ALL_NOACCUM     3
#define LOG_ERR_ACCUM       2
#define LOG_ALL_ACCUM       1
#define LOG_ACCUM_OFF       0
#endif

statusReport_t * getstatusreport(void) ;

#define INST_LOG_OFF    0
#define INST_LOG_ALL    1
#define INST_LOG_ERR    2
#define INST_LOG_ALL_NOACC    3
#define INST_LOG_SINGLE 4

FILE * instanceaccumloghandle(void) ; 
int instanceaccumlogenable(int logEnable) ;     // Returns 0 for success, 1 for failure
void instanceaccumlogflush(void) ;              // flush accumulator log file
int instance_readaccumulatordata(void);
void instancestatusaccumlog(int instance, const char *tag, const char *line);
void processSoundingData(void);
void logSoundingData(int8 errorFlag, int fc, int sn, event_data_t *dw_event);


// read the event counters from the DW1000 - (TX events, RX events, RX Errors, RX timeouts...)
int instancereadevents(dwt_deviceentcnts_t *temp);

// read/write the DW1000 registers
int instancelowlevelreadreg(int regID,int regOffset,unsigned int *retRegValue) ;  // Read low level 32 bit register
int instancelowlevelwritereg(int regID,int regOffset,unsigned int regValue) ;     // Write low level 32 bit register value


//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

// close the SPI Cheetah interface  
void instance_close(void);
// called to reset the DW1000 device
void instancedevicesoftreset(void) ;

// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(accBuff_t *buf) ;

// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;  
// configure the payload and MAC address
void instancesetpayloadandaddresses(instancePayloadConfig_t *plconfig) ;
// configure the antenna delays
void instancesetantennadelays(double fdelay) ;                      // delay in nanoseconds
double instancegetantennadelay(int prf);        // returns delay in nanoseconds
// configure whether the Anchor sends the ToF reports to Tag
void instancesetreporting(int anchorSendsTofReports) ; // configure anchor to send TOF reports to Tag

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
void inst_processackmsg(instance_data_t *inst, uint8 seqNum);
void inst_processrxtimeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(uint32 time) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, int message, uint32 time_ms);
// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxcallback(const dwt_callback_data_t *rxd);
void instance_txcallback(const dwt_callback_data_t *txd);

//resets the instance back to initial state
void instance_reset(void);

// Go to pause mode when "run" input parameter is 0, Exit pause mode when "run" input parameter is 1.
void instance_pause(int run) ;
// Return true(1) if instance is in the pause state or false(0) if not
int isinstancepaused(void) ;

// returns the deepsleep status (is DW1000 in deep sleep or not)
int instance_sleeping(void) ;
// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesettagsleepdelay(int, int);

void instancesetblinkreplydelay(int delayms);
// sets the Tag/Anchor reply delay time (the time Tag/Anchor wait before sending the reply message (Final/Response))
void instancesetreplydelay(int delayms, int datalength);
// set the SPI rate
int instancesetspibitrate(int newRateKHz) ;                         // Set SPI rate
// set/get the instance roles e.g. Tag/Anchor/Listener
void instancesetrole(int mode) ;                // 
int instancegetrole(void) ;

int instancesetdeepsleep(int inst_mode);
int instancegetdeepsleep(void);
// get the DW1000 device ID (e.g. 0xDECA0130 for MP)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

int instancegetRxCount(void);

int instancegetRngCount(void);

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)
extern const uint16 rfDelays[2];
extern const tx_struct txSpectrumConfig[8];

int instance_starttxtest(int framePeriod);

int instance_startcwmode(int chan);

int instance_peekevent(void);

void instance_saveevent(event_data_t newevent);

event_data_t instance_getsavedevent(void);

void instance_putevent(event_data_t newevent);

event_data_t instance_getevent(void);

void instance_clearevents(void);

#ifdef __cplusplus
}
#endif

#endif
