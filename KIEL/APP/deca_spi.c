/*! ------------------------------------------------------------------------------------------------------------------
 * FILE: deca_spi.c - SPI Interface Access Functions
 *
 * Copyright 2009, 2012 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * Author(s): Billy Verso / Zoran Skrba
 */

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"

char buf[100];
extern uint32 startTime;

#define RELEASE

#if 0
int writetospi_serial( uint16 headerLength,
			   	    const uint8 *headerBuffer,
					uint32 bodylength,
					const uint8 *bodyBuffer
				  );

int readfromspi_serial( uint16	headerLength,
			    	 const uint8 *headerBuffer,
					 uint32 readlength,
					 uint8 *readBuffer );
#endif
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

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
	while (port_SPIx_busy_sending()); //wait for tx buffer to empty

	port_SPIx_disable();

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */

void hexDump( const uint8 * buf, uint32 length )
{
#if 0
   char format[255];
   unsigned int i = 0;
   int size = length;
   int indexSize = 0;
   int rawLength = 80;
   while ( size > 0 ) {
       indexSize++;
       size = size / 10;
   }
   sprintf( format, " [%%0%dd]:0x%%02x", indexSize );
   rawLength = rawLength/strlen( format );
   for ( ; i < length; i++ ) {
       sprintf( format, i, buf[i] );
       printUSARTnoNew(format);
       if ( ( ( i + 1 ) % rawLength ) == 0 ) {
    	   printUSARTnoNew( "\r\n");
       }
   }
   printUSARTnoNew( "\r\n");
#endif
}
#pragma GCC optimize ("O3")
int writetospi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodylength,
    const uint8 *bodyBuffer
)
{
	int i=0;
	uint8 temp;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	//SPIx->DR = headerBuffer[i];
    	*(uint8_t *)&(SPIx->DR) = headerBuffer[i];
    	while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	temp = (uint8)SPIx->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
     	//SPIx->DR = bodyBuffer[i];
     	*(uint8_t *)&(SPIx->DR) = bodyBuffer[i];
    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		temp = (uint8)SPIx->DR;
	}

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
#ifndef RELEASE
    sprintf(buf, "%u Tx write Header:\n",portGetTickCount()-startTime);
    printUSART( buf );
    hexDump( headerBuffer, headerLength );
    sprintf(buf, "%u Tx write Body:\n",portGetTickCount()-startTime  );
    printUSART( buf );
    hexDump( bodyBuffer, bodylength );
#endif

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer
)
{

	int i=0;
	uint8_t temp;
	uint8 dataseq[40];
    decaIrqStatus_t  stat ;

#ifndef RELEASE
    sprintf( buf, "%u Tx Read Header:\n",portGetTickCount()-startTime  );
    printUSART( buf );
    hexDump( headerBuffer, headerLength );
#endif
    stat = decamutexon() ;

    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	//SPIx->DR = headerBuffer[i];
    	*(uint8_t *)&(SPIx->DR) = headerBuffer[i];
    	//while(port_SPIx_busy_sending());
     	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

     	temp = (uint8_t)SPIx->DR ;
    }

    for(i=0; i<readlength; i++)
    {
    	*(uint8_t *)&(SPIx->DR) = 0;

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
    	//memset(dataseq, 40, 0);
		//sprintf(dataseq,"0x%x",SPIx->DR);
		//writetoLCD( 6, 1, dataseq); //send some data

    	readBuffer[i] = (uint8_t)SPIx->DR ;//port_SPIx_receive_data(); //this clears RXNE bit
    }

    SPIx_CS_GPIO->BSRR = SPIx_CS;
#ifndef RELEASE
    sprintf( buf, "%u Rx read Body:\n",portGetTickCount()-startTime  );
    printUSART( buf );
    hexDump( readBuffer, readlength );
#endif
    decamutexoff(stat) ;

    return 0;
} // end readfromspi()

#if (EVB1000_LCD_SUPPORT == 1)

void writetoLCD
(
    uint32       bodylength,
    uint8        rs_enable,
    const uint8 *bodyBuffer
)
{

	int i = 0;
	int sleep = 0;
	//int j = 10000;

	if(rs_enable)
	{
    	port_LCD_RS_set();
    }
	else
	{
		if(bodylength == 1)
		{
			if(bodyBuffer[0] & 0x3) //if this is command = 1 or 2 - exsecution time is > 1ms
				sleep = 1 ;
		}
    	port_LCD_RS_clear();
    }

    port_SPIy_clear_chip_select();  //CS low


    //while(j--); //delay

    for(i=0; i<bodylength; i++)
    {
		port_SPIy_send_data(bodyBuffer[i]); //send data on the SPI

		while (port_SPIy_no_data()); //wait for rx buffer to fill

		port_SPIy_receive_data(); //this clears RXNE bit
	}

    //j = 10000;

    port_LCD_RS_clear();

    //while(j--); //delay

    port_SPIy_set_chip_select();  //CS high

    if(sleep)
    	Sleep(2);
} // end writetoLCD()
#else
void writetoLCD(uint32 bodylength,
    uint8        rs_enable,
    uint8 *bodyBuffer)
{
	int i;
	bodylength+=2;
	bodyBuffer[bodylength-2] = '\n';
	bodyBuffer[bodylength-1] = '\r';
	for( i=0; i<bodylength; i++ )
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1,(uint16)bodyBuffer[i]);
	}
}

void printUSART(const char* buf )
{
	int i;
	int len = strlen(buf);
	for( i=0; i<len; i++ )
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1,(uint16)buf[i]);
	}

	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,'\r');
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1,'\n');
}
void printUSARTnoNew(const char* buf )
{
	int i;
	int len = strlen(buf);
	for( i=0; i<len; i++ )
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1,(uint16)buf[i]);
	}
}
#endif
