// -------------------------------------------------------------------------------------------------------------------
//
//  File: instance_sws.h - Decawave switches - Controls if errors/accumulators are logged
//
//  Copyright (c) 2008 - DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Billy Verso, June 2009
//
// -------------------------------------------------------------------------------------------------------------------


#ifndef _INSTANCE_SWS_H_
#define _INSTANCE_SWS_H_

#ifdef __cplusplus
extern "C" {
#endif


// Select whether we deliver sounding data (accumulator data)
#define DECA_SUPPORT_SOUNDING   (1)                     // NB: if enabled this needs extra buffer memory and processing power.

// Select whether we generate a log file.
#define DECA_LOG_ENABLE   (0)                           // set to 0 to disable loging

// Select whether software needs to copy accumulator data, e.g. for diagnostic graphing
#define DECA_KEEP_ACCUMULATOR   (1)                     // set to zero if not required

// Select whether we support logging accumulator data to a file.
#define DECA_ACCUM_LOG_SUPPORT  (0)                     // set to 0 to remove support

// Select whether software reads accumulator for bad frames
#define DECA_BADF_ACCUMULATOR   (0)                     // set to zero if not required

// Select whether software logs errored/bad frames
#define DECA_ERROR_LOGGING   (1)                     // set to zero if not required

#if DECA_SUPPORT_SOUNDING==0
    #if DECA_KEEP_ACCUMULATOR==1
    #error 'Code expects DECA_SUPPORT_SOUNDING set to keep accumulator data'
    #endif
    #if DECA_ACCUM_LOG_SUPPORT==1
    #error 'Code expects DECA_SUPPORT_SOUNDING set to keep accumulator data for logging'
    #endif
#endif

/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/

#define DEEP_SLEEP (1) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode, there are two cases:
// 1. when the Anchor is sending the range report back to the Tag, the Tag will enter sleep after a ranging exchange is finished
// once it receives a report or times out, before the next poll message is sent (before next ranging exchange is started).
// 2. when the Anchor is not sending the report the Tag will go automatically to sleep after the final message is sent
#define DEEP_SLEEP_XTAL_ON (0)
//NOTE: on the EVK1000 the DEEPSLEEP is not actually putting the DW1000 into full DEEPSLEEP mode as XTAL is kept on 

#define ENABLE_AUTO_ACK		(0)		//To enable auto-ack feature set this to 1, frame filtering also needs to be set (to allow ACK frames)
#define ACK_RESPONSE_TIME	(5)     //ACK response time is 5 us (5 symb.), the instance receiving an ACK request will send the ACK after this delay.
#define WAIT_FOR_RESPONSE_DLY	(0) //Tx to Rx delay time, the instance waiting for response will delay turning the receiver on by this amount
// The receiver will be enabled automatically after frame transmission if DWT_RESPONSE_EXPECTED is set in dwt_starttx.
// The instance requesting an ACK, can also program a delay in turning on its receiver, if it knows that the ACK will be sent after particular delay.
// Here it is set to 0 us, which will enable the receiver a.s.a.p so it is ready for the incoming ACK message. 
// The minimum ACK response time about 5us, and the IEEE standard, specifies that the ACK has to be able to be sent within 12 us of reception of an ACK request frame.

//Note:	Double buffer mode can only be used with auto rx re-enable, auto rx re-enable is on by default in Listener and Anchor instances
#define DOUBLE_RX_BUFFER (0) //To enable double RX buffer set this to 1 - this only works for the Listener instance
//NOTE: this feature is really meant for a RX only instance, as TX will not be possible while double-buffer and auto rx-enable is on.

#if DOUBLE_RX_BUFFER==1
    #if DECA_KEEP_ACCUMULATOR==1
#pragma message (__FILE__" Accumulators will not be displayed when DOUBLE_RX_BUFFER set as well as DECA_KEEP_ACCUMULATOR")
    #endif
#endif

#define DR_DISCOVERY	(1) //to use discovery ranging mode (tag will blink until it receives ranging request from an anchor)
							//after which it will pair with that anchor and start ranging exchange


#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

#define DEEP_SLEEP_AUTOWAKEUP (0) //to test SLEEP mode

#define PUT_TEMP_VOLTAGE_INTO_POLL (0)     // to insert wakeup sampled TEMP/VOLTAGE into POLL message

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // _INSTANCE_SWS_H_
