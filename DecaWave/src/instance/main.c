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
#include "instance_sws.h"
#include "udpClient.h"
#include "instance.h"
#include "gpioDriver.h"


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
typedef struct
{
    uint64 address ;                                    // contains this own address
    char userString[MAX_USER_PAYLOAD_STRING+2] ;        // user supplied payload string
    int counterAppend ;
	int anchorListSize ;
	int anchorPollMask ;
	int tagSleepDuration ;
	int tagBlinkSleepDuration ;
    int anchorSendReports ;
	int responseDelay ;
} plConfig_t ;

plConfig_t payloadConfig = { 0xDECA000011223300,         // packetAddress 64 bit extended address
                            "",							 // payload string - probably will initialise to empty string
                            0,							 // flag - include ASCII a counter in the payload
							ANCHOR_LIST_SIZE,			 // anchor list size
							1,							 // anchor poll mask (bit mask: 1 = poll Anchor ID 1, 3 = poll both Anchors ID 1 and ID 2)
							400,						 // tag sleep time
							1000,						 // tag blink sleep time
                            SEND_TOF_REPORT,			 // anchor sends reports by default (to tag that it ranged to)
							FIXED_REPLY_DELAY	} ;		 // default response delay time


plConfig_t tempPayloadConfig ;              // a copy for temp update in dialog
static SpiConfig s_spi;
int instance_anchaddr = 0;
int instance_tagaddr = 0;
int dr_mode = 0;
int poll_delay = 0;
unsigned char* ipAddress;
int port;
int instance_mode = ANCHOR;
int viewClockOffset = 0 ;
double antennaDelay  ;                          // This is system effect on RTD subtracted from local calculation.
double antennaDelay16 ;                         // holds antenna delay at 16 MHz PRF
double antennaDelay64 ;                         // holds antenna delay at 64 MHz PRF
int initComplete = 0 ;                          // Wait for initialisation before polling status register

#if (DECA_SUPPORT_SOUNDING==1 && DECA_KEEP_ACCUMULATOR==1)
	static accBuff_t buffers[1];
#else
accBuff_t *buffers; //don't need this when DECA_SUPPORT_SOUNDING is off
#endif

uint32 inittestapplication( int mode );



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

    instance_close() ;                          //shut down instance, PHY, SPI close, etc.
    openspi( &s_spi );

    inittestapplication( 1 ) ;                     //re-initialise instance/device
} // end restartinstance()


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

uint32 getmstime()
{
  struct timeval tv;
  gettimeofday( &tv, NULL );

  return (uint32)((tv.tv_sec * 1000) + ( tv.tv_usec / 1000));
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-drcipt] [data,..]\n", prog);
	puts("  -d --device   	device to use (default /dev/spidev1.1)\n"
	     "  -r --role     	0 = LISTENER 1 = TAG 2 = ANCHOR\n"
	     "  -c --channel  	channel selection 0-7 like on evkDW1000\n"
		 "  -i --ipAddress  udp server address\n"
		 "  -p --port 		server listening port"
		 "  -t --tagID		tag serial number"
	);
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device"	, 1, 0, 'd' },
			{ "role"	, 1, 0, 'r' },
			{ "channel"	, 1, 0, 'c' },
			{ "ipAddress"	, 1, 0, 'i' },
			{ "port"	, 1, 0, 'p' },
			{ "tagID"	, 1, 0, 't' },
			{ NULL		, 0, 0, 0 	},
		};
		int c;

		c = getopt_long(argc, argv, "d:r:c:i:p:t:", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'd':
			s_spi.device = optarg;
			break;
		case 'i':
					ipAddress = optarg;
					break;
		case 'r':
			instance_mode = (int)strtol(optarg, NULL, 0);
			break;
		case 'c':
			dr_mode = (int)strtol(optarg, NULL, 0);
			break;
		case 'p':
			port = (int)strtol(optarg, NULL, 0);
			break;
		case 't':
			instance_tagaddr = (int)strtol(optarg, NULL, 0);
			break;

		default:
			print_usage(argv[0]);
			break;
		}
	}
}
int main(int argc, char *argv[])
{
	uint32 status = 0;
	uint32 states = 0;
	uint32 time_ms;

	GPIOExport(DW_RESET_PIN);
	GPIODirection(DW_RESET_PIN,OUT);
	GPIOWrite( DW_RESET_PIN, LOW );

	GPIOExport(DW_EXT_ON);
	GPIODirection(DW_EXT_ON,OUT);
	GPIOWrite( DW_EXT_ON, HIGH );

	//GPIOExport(DW_WAKEUP_PIN);
	//GPIODirection(DW_WAKEUP_PIN,OUT);
	//GPIOWrite( DW_WAKEUP_PIN, LOW );

	usleep( 3000 ); //wait 3ms according to datasheet

	GPIOWrite( DW_RESET_PIN, HIGH ); //take out of reset drive to low for reset

	GPIOExport(DW_WAKEUP_PIN);
	GPIODirection(DW_WAKEUP_PIN,OUT);
	GPIOWrite( DW_WAKEUP_PIN, LOW );

	s_spi.bits = 8;
	s_spi.speed = 4500000;
	s_spi.delay = 0;
	s_spi.mode = 0;
	s_spi.toggle_cs = 0;
	s_spi.device = "/dev/spidev0.0";

	parse_opts(argc, argv);

	openspi( &s_spi );

	PINFO( "spi initialized");

    if(inittestapplication( 1 ) == (uint32)-1)
	{
		PERROR("init failed");
		return 0; //error
	}

    while( 1 )
    {
    	 if (initComplete)   // if application is not paused (and initialsiation completed)
    	 {
    		 time_ms = getmstime();
			//system_services();
			instance_run( time_ms );
			usleep( poll_delay * 1000);
    	 }
    }
}

// ======================================================
//
//  Configure payload in instance
//
void payloadconfigure(void)
{
    instancePayloadConfig_t ipc ;
	int max_user_payload_string = MAX_USER_PAYLOAD_STRING;

	ipc.forwardToFRAddress = forwardingAddress[0];
	ipc.anchorAddress = anchorAddressList[instance_anchaddr];
	ipc.anchorAddressList = anchorAddressList;
	ipc.anchorListSize = payloadConfig.anchorListSize ;
	ipc.anchorPollMask = payloadConfig.anchorPollMask ;

	ipc.sendReport = payloadConfig.anchorSendReports ;  //1 => SEND_TOF_REPORT anchor sends TOF report to tag

    ipc.countAppend = payloadConfig.counterAppend ;

	if(payloadConfig.counterAppend)
		ipc.payloadLen = 8;
	else
		ipc.payloadLen = 0;

    //strncpy_s(ipc.payloadString,sizeof(ipc.payloadString),payloadConfig.userString,(max_user_payload_string-ipc.payloadLen)) ;
	ipc.payloadLen += strlen(ipc.payloadString);

    instancesetpayloadandaddresses(&ipc) ;
}

uint32 inittestapplication( int reset )
{
	instanceConfig_t instConfig ;
	uint32 devID ;
	uint8 buffer[1500];
	int result ;

	//this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
	if(DWT_DEVICE_ID != devID)
	{
		dwt_spicswakeup(buffer, 1500);
	}
	devID = instancereaddeviceid() ;

    if ((devID & 0xFFFF0000) != 0xDECA0000)
    {
        PERROR("dev id problem");
    		return(-1) ;
    }

    PINFO("DEVID = 0x%x", devID);

    result = instance_init(buffers) ;       // reset, done as part of init

	if (result != DWT_SUCCESS)
	{
	   PINFO("ERROR in initializing device", "DecaWave Application");
	   exit(-1) ;
	}

	PINFO( "%s",instance_mode == 0 ? "LISTENER": instance_mode == 1 ? "TAG": "ANCHOR");
	instancesetrole(instance_mode) ;                                                // Set this instance role

	if(instance_mode == LISTENER) instcleartaglist();


	if(instance_mode == ANCHOR)
	{
	    if( UdpclinetConnect((const char *)ipAddress, port))
		{
			PINFO("udp client failed to init socket");
		}

	}
	else
	{

	}

	if(instance_mode != TAG)
	{

	}
	else
	{
	}

    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    PINFO("ch selection: %d ch=%d preamble=%d prf=%d pacSize=%d nsSFD=%d datarate=%d preambleLength=%d",
    		dr_mode, instConfig.channelNumber, instConfig.preambleCode, instConfig.pulseRepFreq,
			instConfig.pacSize, instConfig.nsSFD, instConfig.dataRate, instConfig.preambleLen);

    instance_config(&instConfig) ;                  // Set operating channel etc

    if (initComplete == 0)
  	{
  	    antennaDelay16 = instancegetantennadelay(DWT_PRF_16M);          // MP Antenna Delay at 16MHz PRF (read from OTP calibration data)
          antennaDelay64 = instancegetantennadelay(DWT_PRF_64M);          // MP Antenna Delay at 64MHz PRF (read from OTP calibration data)
          if (antennaDelay16 == 0) antennaDelay16 = DWT_PRF_16M_RFDLY;    // Set a value locally if not programmed in OTP cal data
          if (antennaDelay64 == 0) antennaDelay64 = DWT_PRF_64M_RFDLY;    // Set a value locally if not programmed in OTP cal data
  	}

	if (instConfig.pulseRepFreq == DWT_PRF_64M)
	{
	  antennaDelay = antennaDelay64 ;
	}
	else
	{
	  antennaDelay = antennaDelay16 ;
	}

	//PINFO("antena delay=%0.3f",antennaDelay);
	instancesetantennadelays(antennaDelay) ;

	payloadconfigure() ;                            // set up initial payload configuration

	instancesettagsleepdelay(payloadConfig.tagSleepDuration, payloadConfig.tagBlinkSleepDuration) ;

	instancesetblinkreplydelay(payloadConfig.responseDelay);
	instancesetreplydelay(payloadConfig.responseDelay, 0) ;

	instancesetreporting(payloadConfig.anchorSendReports) ;     // Set whether anchor instance sends reports

	initComplete = 1;

	return devID;
}

