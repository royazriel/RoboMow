#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "../dwDriver/deca_device_api.h"
#include "spiDriver.h"

static int s_fd = NULL;
uint32 startTime;
#define RELEASE


int getSpiHandle()
{
	return s_fd;
}

void _waitForBitUp()
{
#if 0
    int res;
    struct pollfd fdset[1];
    //fdset[0].fd = m_gpioSpiSlaveDataReady.getHandle();
    fdset[0].events = POLLPRI | POLLERR;
    while ( true ) {
        if ( m_gpioSpiSlaveDataReady.isSet() ) 
        {
            return;
        }
        fdset[0].revents = 0;
        res = poll( fdset , 1,  POLL_TIME_OUT_MILLI_SECONDS );
        if ( res == RESULT_TIMEOUT ) {
            continue;
        }
        if ( fdset[0].revents & POLLPRI ) {
            continue;
        }
    }
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi( SpiConfig * spiConfig )
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi( SpiConfig * spiConfig )
{
    int ret = 0;
	
	s_fd = open( spiConfig->device, O_RDWR );
	if ( s_fd < 0 )
    {
		pabort( "can't open device" );
    }
	/*
	 * spi mode
	 */
	ret = ioctl( s_fd, SPI_IOC_WR_MODE, &spiConfig->mode );
	if ( ret == -1 )
    {
		pabort( "can't set spi mode" );
    }
	ret = ioctl( s_fd, SPI_IOC_RD_MODE, &spiConfig->mode );
	if ( ret == -1 )
    {
		pabort( "can't get spi mode" );
    }
	/*
	 * bits per word
	 */
	ret = ioctl( s_fd, SPI_IOC_WR_BITS_PER_WORD, &spiConfig->bits );
	if (ret == -1)
    {
		pabort("can't set bits per word");
    }

	ret = ioctl( s_fd, SPI_IOC_RD_BITS_PER_WORD, &spiConfig->bits );
	if ( ret == -1 )
    {
		pabort( "can't get bits per word" );
    }
	/*
	 * max speed hz
	 */
	ret = ioctl( s_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiConfig->speed );
	if ( ret == -1 )
    {
		pabort("can't set max speed hz");
    }
	ret = ioctl( s_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spiConfig->speed );
	if ( ret == -1 )
    {
		pabort("can't get max speed hz");
    }
	startTime = getmstime();
	return 0;

} // end openspi()


/*! ------------------------------------------------------------------------------------------------------------------
* Function: closespi()
*
* Low level abstract function to close the the SPI device.
* returns 0 for success, or -1 for error
*/
int closespi(void)
{
	//while (port_SPIx_busy_sending()); //wait for tx buffer to empty
	close(s_fd);

	//port_SPIx_disable();

	return 0;
} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodyLength,
    const uint8 *bodyBuffer
)
{
    int ret = 0;
    struct spi_ioc_transfer  transfer[2] = {};

    memset( transfer, 0, sizeof( transfer[2] ) );
    transfer[0].tx_buf = (unsigned long) headerBuffer;
    transfer[1].tx_buf = (unsigned long) bodyBuffer;
    transfer[0].len = headerLength;
    transfer[1].len = bodyLength;
    ret = ioctl( s_fd, SPI_IOC_MESSAGE( 2 ), transfer );
    if (  ret < 0 ) 
    {
        PERROR( "SPI ERROR [%d]", errno );
        return RESULT_ERR;
    }
#ifndef RELEASE 
    printf( "%u Tx write Header:\n",getmstime()-startTime );
    hexDump( headerBuffer, headerLength );
    printf( "%u Tx write Body:\n",getmstime()-startTime  );
    hexDump( bodyBuffer, bodyLength );
#endif
    return ret;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readLength,
    uint8       *readBuffer
)
{
    int ret = 0;
    struct spi_ioc_transfer transfer[2] = {};

#ifndef RELEASE
    printf( "%u Tx Read Header:\n",getmstime()-startTime  );
    hexDump( headerBuffer, headerLength );
#endif
    memset( &transfer, 0, sizeof( transfer[2] ) );
    transfer[0].tx_buf = ( unsigned long ) headerBuffer;
    transfer[0].len = headerLength;
	transfer[1].rx_buf = ( unsigned long ) readBuffer;
	transfer[1].len = readLength;
	ret = ioctl( s_fd, SPI_IOC_MESSAGE( 2 ), &transfer );
	if( ret < 0 )
	{
		PERROR( "SPI ERROR [%d]", errno );
	}

#ifndef RELEASE
    printf( "%u Rx read Body:\n",getmstime()-startTime  );
    hexDump( readBuffer, readLength );
#endif
    return 0;
} // end readfromspi()

int setspibitrate(int desiredRatekHz)
{
	int result = DWT_ERROR;
#if (SPI_CHEETAH_SUPPORT == 1)
	if(spihandle != DWT_ERROR)
		result = setspibitrate_ch(desiredRatekHz);
	else
#endif
#if (SPI_VCP_SUPPORT == 1)
	if(vcphandle != DWT_ERROR)
		result = setspibitrate_vcp(desiredRatekHz);
#endif
	return result;
}
