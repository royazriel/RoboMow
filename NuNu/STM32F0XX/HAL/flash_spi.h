/*
 * flash_spi.h
 *
 *  Created on: Jan 4, 2015
 *      Author: roy
 */

#ifndef FLASH_SPI_H_
#define FLASH_SPI_H_

#include "hardware_config.h"

#define 	SPI_FLASH_CMD_BE   		0xC7
#define 	SPI_FLASH_CMD_RDID   		0x9F
#define 	SPI_FLASH_CMD_RDSR   		0x05
#define 	SPI_FLASH_CMD_READ   		0x03
#define 	SPI_FLASH_CMD_SE   		0xD8
#define 	SPI_FLASH_CMD_WREN   		0x06
#define 	SPI_FLASH_CMD_WRITE   		0x02
//M25P SPI Flash supported commands.
#define 	SPI_FLASH_CMD_WRSR   		0x01
#define 	SPI_FLASH_CS_HIGH()   		GPIO_SetBits(SPI_FLASH_PORT, SPI_FLASH_CS )  //Deselect sFLASH: Chip Select pin high.
#define 	SPI_FLASH_CS_LOW()   		GPIO_ResetBits(SPI_FLASH_PORT, SPI_FLASH_CS) //Select sFLASH: Chip Select pin low.
#define 	SPI_FLASH_DUMMY_BYTE   	0xA5
#define 	SPI_FLASH_M25P128_ID   	0x202018
#define 	SPI_FLASH_M25P64_ID   		0x202017
#define 	SPI_FLASH_SPI_PAGESIZE   	0x100
#define 	SPI_FLASH_WIP_FLAG   		0x01


void 		sFLASH_DeInit (void);
void 		FlashSpiEraseBulk (void);
void 		FlashSpiEraseSector (uint32_t SectorAddr);
void 		sFLASH_Init (void);
void 		FlashSpiReadBuffer (uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint8_t 	FlashSpiReadByte (void);
uint32_t 	FlashSpiReadID (void);
uint8_t 	FlashSpiSendByte (uint8_t byte);
uint16_t 	FlashSpiSendHalfWord (uint16_t HalfWord);
void 		FlashSpiStartReadSequence (uint32_t ReadAddr);
void 		FlashSpiWaitForWriteEnd (void);
void 		FlashSpiWriteBuffer (uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void 		FlashSpiWriteEnable (void);
void 		FlashSpiWritePage (uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

#endif /* FLASH_SPI_H_ */
