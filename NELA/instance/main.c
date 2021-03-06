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
#include <sys/time.h>
#include <pthread.h>

#include "deca_device_api.h"
#include "compiler.h"
#include "spiDriver.h"
//#include "udpClient.h"
#include "AfUnix.h"
#include "instance.h"

#include "deca_types.h"
#include "gpioDriver.h"


#if (DR_DISCOVERY == 0)
#define SOFTWARE_VER_STRING    "Version 2.37 Mx " //
#else
#define SOFTWARE_VER_STRING    "Version 2.37    " //
#endif
#define ROBOMOW_SW_VER_STRING  "1.2.2"

//#define INTEGRATION_TESTING


static SpiConfig s_spi;
int instance_anchaddr = 0;
int instance_tagaddr = 0;
int dr_mode = 0;
int responseDelay = 150;
int noPrint = 0;
uint32 resetTimeout = 0;
uint64 lastCommunication = 0;

uint32 txPower = 0x1f1f1f1f;
uint64 burnAddress = 0;
unsigned char* ipAddress;
int port = 0;
int instance_mode = ANCHOR;
int viewClockOffset = 0 ;
double antennaDelay  ;                          // This is system effect on RTD subtracted from local calculation.
double antennaDelay16 ;                         // holds antenna delay at 16 MHz PRF
double antennaDelay64 ;                         // holds antenna delay at 64 MHz PRF
int paused = 0;
int lastReportTime;
double antennaDelay  ;                          // This is system effect on RTD subtracted from local calculation.
extern uint32 startTime;
int aliveTest = 0;

char reset_request;

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
    closespi();
    openspi( &s_spi );

    inittestapplication() ;                     //re-initialise instance/device
} // end restartinstance()


uint32 inittestapplication()
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
#if DECA_SUPPORT_SOUNDING==1
    result = instance_init(buffers) ;       // reset, done as part of init
#else
	result = instance_init() ;       		// reset, done as part of init
#endif

	if (result != DWT_SUCCESS)
	{
	   PINFO("ERROR in initializing device", "DecaWave Application");
	   exit(-1) ;
	}

	PINFO( "%s",instance_mode == 0 ? "LISTENER": instance_mode == 1 ? "TAG": "ANCHOR");
	instancesetrole(instance_mode) ;                                                // Set this instance role

	if(instance_mode == LISTENER) instcleartaglist();



	if( 0 /*responseDelay < FIXED_REPLY_DELAY*/ ) //if fast ranging then initialise instance for fast ranging application
	{
		instance_init_f(instance_mode); //initialise Fast 2WR specific data
		//when using fast ranging the channel config is either mode 2 or mode 6
		//default is mode 2
		//dr_mode = decarangingmode();
		if( dr_mode != 1 && dr_mode != 5 )
		{
			PINFO("channel mode has to be 1 or 5");
			exit(1);
		}
	}
	else
	{
		instance_init_s(instance_mode);
		//dr_mode = decarangingmode();
	}

    instConfig.channelNumber = chConfig[dr_mode].channel;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;

    PINFO("ch selection: %d ch=%d preamble=%d prf=%d pacSize=%d nsSFD=%d datarate=%d preambleLength=%d",
    		dr_mode, instConfig.channelNumber, instConfig.preambleCode, instConfig.pulseRepFreq,
			instConfig.pacSize, instConfig.nsSFD, instConfig.dataRate, instConfig.preambleLen);

    instance_config(&instConfig) ;                  // Set operating channel etc

#if (DR_DISCOVERY == 0)
    addressconfigure() ;                            // set up initial payload configuration
    instancesettagsleepdelay(responseDelay*4, BLINK_SLEEP_DELAY); //set the Tag sleep time
#else
    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time
#endif


      if( 0 /*responseDelay < FIXED_REPLY_DELAY*/ )
      {
      	//Fast 2WR specific config
      	//configure the delays/timeouts
      	instance_config_f();
      }
      else //use default ranging modes
      {
      	// NOTE: this is the delay between receiving the blink and sending the ranging init message
      	// The anchor ranging init response delay has to match the delay the tag expects
      	// the tag will then use the ranging response delay as specified in the ranging init message
      	// use this to set the long blink response delay (e.g. when ranging with a PC anchor that wants to use the long response times != 150ms)
		if( 0 /*responseDelay < FIXED_REPLY_DELAY*/ )
		{
			instancesetblinkreplydelay(FIXED_LONG_BLINK_RESPONSE_DELAY);
		}
		else //this is for ARM to ARM tag/anchor (using normal response times 150ms)
		{
			instancesetblinkreplydelay(responseDelay);
		}

#if (DR_DISCOVERY == 1)
        //set the default response delays
        instancesetreplydelay(responseDelay, 0);
#else
        //set the default response delays
        instancesetreplydelay(responseDelay, 0);
#endif
    }

    return devID;
}

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
        printf("Usage: %s [-drciptsab] [data,..]\n", prog);
        puts("  -d --device				device to use (default /dev/spidev1.1)\n"
             "  -r --role				0 = LISTENER 1 = TAG 2 = ANCHOR\n"
             "  -c --channel			channel selection 0-7 like on evkDW1000\n"
			 "  -i --ipAddress			udp server address\n"
			 "  -p --port				server listening port\n"
        	 "  -a --anchorID			anchor id\n"
			 "  -s --response_delay		in milisec\n"
        	 "  -b --burnOTPAddress		8byte hex\n"
        	 "  -x --txPower			tx power\n"
        	 "  -n --noPrint			no print of results to console\n"
        	 "  -t --resetTimeout		in case no communication restart state machine in ms\n"
        	 "  -q --aliveTest			if decawave module returns id through spi exits with 0 otherwise 1\n"
        	 "  -v --version			version num\n"
        );
        exit(1);
}

static void parse_opts(int argc, char *argv[])
{
        while (1) {
                static const struct option lopts[] = {
                        { "device"              , 1, 0, 'd' },
                        { "role"                , 1, 0, 'r' },
                        { "channel"             , 1, 0, 'c' },
                        { "ipAddress"           , 1, 0, 'i' },
                        { "port"                , 1, 0, 'p' },
                        { "anchorID"            , 1, 0, 'a' },
                        { "response_delay"      , 1, 0, 's' },
						{ "burnOTPAddress"      , 1, 0, 'b' },
						{ "txPower"		        , 1, 0, 'x' },
						{ "noPrint"      		, 1, 0, 'n' },
						{ "resetTimout"    		, 1, 0, 't' },
						{ "aliveTest"    		, 0, 0, 'q' },
						{ "version"    			, 0, 0, 'v' },
                        { NULL          		, 0, 0, 0   },
                };
                int c;

                c = getopt_long(argc, argv, "d:r:c:i:p:a:s:b:x:n:t:qv", lopts, NULL);

                if (c == -1)
                       break;

                switch (c)
                {
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
					case 'a':
							instance_anchaddr = (int)strtol(optarg, NULL, 0);
							break;
					case 's':
							responseDelay = (int)strtol(optarg, NULL, 0);
							break;
					case 'b':
							burnAddress = strtoll(optarg, NULL, 16);
							instance_mode = -1;
							PINFO("%llx",burnAddress);
							break;
					case 'x':
							txPower = (uint32)strtol(optarg, NULL, 0);
							break;
					case 'n':
							noPrint = (int)strtol(optarg, NULL, 0);
							break;
					case 't':
							resetTimeout = (int)strtol(optarg, NULL, 0);
							break;
					case 'q':
							aliveTest = 1;
							break;
					case 'v':
							printf("%s\n",ROBOMOW_SW_VER_STRING);
							exit(0);
							break;
					default:
							print_usage(argv[0]);
							break;
                }
        }
}

void swap4Bytes( uint8* buf )
{
	uint8 tmp = buf[0];
	buf[0] = buf[3];
	buf[3]= tmp;
	tmp = buf[1];
	buf[1] = buf[2];
	buf[2] = tmp;
}

#ifdef DW_ISR_SUPPORT
void *DecawaveIrq( void *ptr )
{
	while(1)
	{
		GPIOPoll(DW_IRQ_PIN, 0);
#ifdef DEBUG_MULTI
		PINFO("dwt_isr received time %u",getmstime()-startTime);
#endif
	}
}
#endif

int main(int argc, char *argv[])
{
	int toggle = 1;
	int ranging = 0;
	double range_result = 0;
	double avg_result = 0;
	unsigned char buffer[100];
	int gpioVal = 1;
	pthread_t irqThread;
	int ret;


	lastCommunication = getmstime();

	GPIOExport(DW_RESET_PIN);
	GPIODirection(DW_RESET_PIN,OUT);
	GPIOWrite( DW_RESET_PIN, LOW );

#ifdef DW_ISR_SUPPORT
	GPIOExport(DW_IRQ_PIN);
	GPIODirection(DW_IRQ_PIN,IN);
	GPIOPoll(DW_IRQ_PIN, 1);  //to open the value file
#endif

	usleep( 3000 ); //wait 3ms according to datasheet

	GPIOWrite( DW_RESET_PIN, HIGH ); //take out of reset drive to low for reset

	s_spi.bits = 8;
	s_spi.speed = 1500000;
	s_spi.delay = 0;
	s_spi.mode = 0;
	s_spi.toggle_cs = 0;
	s_spi.device = "/dev/spidev0.0";

	if( argc == 1)
	{
		print_usage(argv[0]);
		exit(1);
	}
	parse_opts(argc, argv);

	openspi( &s_spi );
	PINFO("Deca SW Vesion: %s Friendly SW Version: %s\r\n",SOFTWARE_VER_STRING,ROBOMOW_SW_VER_STRING);
	PINFO( "spi initialized");

    if(inittestapplication() == (uint32)-1)
	{
		PERROR("init failed");
		exit(1);
	}

    if(aliveTest)
    {
    	PINFO("init device success");
    	exit (0);
    }

    if( burnAddress > 0 )
    {
    	int ret = 1;
    	ret = dwt_otpwriteandverify((uint32)(burnAddress>>32), 0);
    	if( ret > 0 )
    	{
    		PINFO("OTP ADDR 0 burned successfully %x",(uint32)(burnAddress>>32));
    	}
    	ret = dwt_otpwriteandverify((uint32)(burnAddress & 0xffffffff), 1);
		if( ret > 0 )
		{
			PINFO("OTP ADDR 1 burned successfully %x", (uint32)(burnAddress & 0xffffffff));
		}
    	exit(1);
    }
#ifdef INTEGRATION _TESTING
	range_result = 1.00;
#endif

#if (DR_DISCOVERY == 0)
	if(instance_mode == TAG)
	{
		if( AfUnixClinetConnect())
		{
			PINFO("AfUnixClientConnect failed to init socket");
		}
	}
#endif

#ifdef DW_ISR_SUPPORT
	ret = pthread_create( &irqThread, NULL, DecawaveIrq, NULL);

	if(ret)
	{
		PERROR("Error - pthread_create() return code: %d\n",ret);
		exit(1);
	}
#endif

    while( 1 )
    {

#ifdef INTEGRATION_TESTING
    	struct timeval time;
    	gettimeofday(&time,NULL);
        (*(uint32*) buffer)= MAGIC_NUMBER;
        (*(uint32*)(buffer + 4))  = time.tv_sec;
        (*(uint32*)(buffer + 8))  = time.tv_usec;
        (*(uint32*)(buffer + 12)) = (uint32)1;
        (*(float* )(buffer + 16)) = (float)range_result;
        if( instance_mode == TAG )
        {
        	if(AfUnixClinetSendReportTOF(buffer, 20))
        	{
        		sleep(1);
        		//socket probably disconnected
        		AfUnixClinetConnect();
        	}
        }
    	usleep(100000);
    	range_result += 0.01;
    	continue;
#endif

		instance_run();
#if 0
		if( resetTimeout > 0 )
		{
			if( getmstime() >  lastCommunication + resetTimeout )
			{
				PINFO("no Communication resetting reset: %d",resetTimeout );
				restartinstance();
				lastCommunication = getmstime();
				ranging = 0;
			}
		}
#endif
		if(instancenewrange())
		{
			ranging = 1;
			//send the new range information to LCD and/or USB
			range_result = instance_get_idist();
#if (DR_DISCOVERY == 0)
			if(instance_mode == ANCHOR)
#endif
	            avg_result = instance_get_adist();
	            //set_rangeresult(range_result);
	            //PCLS;

			if( !noPrint )
			{
				PINFO("**************************************************LAST: %4.2f m t:%u **************************************************", range_result,getmstime()-lastReportTime);
			}
//	            gpioVal ^= 1;
//	            GPIOWrite( DW_WAKEUP_PIN, gpioVal );
			lastReportTime=getmstime();
			uint64 id = instance_get_anchaddr();
			struct timeval time;
			gettimeofday(&time,NULL);
			(*(uint32*) buffer)= MAGIC_NUMBER;
			(*(uint32*)(buffer + 4))  = time.tv_sec;
			(*(uint32*)(buffer + 8))  = time.tv_usec;
			(*(uint32*)(buffer + 12)) = (uint32)1;
			(*(float* )(buffer + 16)) = (float)range_result;
#if (DR_DISCOVERY == 0)
			if( instance_mode == TAG )
			{
				if( port != 0 )
				{
					//UdpClinetSendReportTOF(buffer, 8);
					if(AfUnixClinetSendReportTOF(buffer, 20))
					{
						AfUnixClinetCloseSocket();
						sleep(1);
						//socket probably disconnected
						AfUnixClinetConnect();
					}
				}
			}
#else
			if( instance_mode == ANCHOR )
			{
				UdpClinetSendReportTOF(buffer, 8);
			}
#endif

#if (DR_DISCOVERY == 0)
			if(instance_mode == ANCHOR)
				PINFO("AVG8: %4.2f m", avg_result);
			//else
				//PINFO("%llx", instance_get_anchaddr());
#else
			//PINFO("AVG8: %4.2f m", avg_result);
#endif
		}

		if(ranging == 0)
		{
			if(instance_mode != ANCHOR)
			{
				if(instancesleeping())
				{
					if(toggle)
					{
						toggle = 0;
						PINFO("AWAITING RESPONSE");
					}
					else
					{
						toggle = 1;
						PINFO("TAG BLINK %llX", instance_get_addr());
					}
				}

				if(instanceanchorwaiting() == 2)
				{
					ranging = 1;
					PINFO("RANGING WITH %016llX", instance_get_anchaddr());
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
							PINFO("AWAITING POLL");
						}
						else
						{
							toggle = 1;
#if (DR_DISCOVERY == 1)
							PINFO(" DISCOVERY MODE %llX", instance_get_addr());
#else
							PINFO(" NON DISCOVERY MODE %llX", instance_get_addr());
#endif
						}
					}
				}
				else if(instanceanchorwaiting() == 2)
				{
					PINFO("    RANGING WITH%llX", instance_get_tagaddr());
				}
			}
		}
    }
}



