/*
 * flash_spi.h
 *
 *  Created on: Jan 4, 2015
 *      Author: roy
 */

#ifndef FLASH_SPI_H_
#define FLASH_SPI_H_

#include "hardware_config.h"

#define 	SPI_FLASH_CMD_BE   			0xC7
#define 	SPI_FLASH_CMD_RDID   		0x9F
#define 	SPI_FLASH_CMD_RDSR   		0x05
#define 	SPI_FLASH_CMD_READ   		0x03
#define 	SPI_FLASH_CMD_BE   			0xD8   //64K
#define 	SPI_FLASH_CMD_SE   			0x20   //4K
#define 	SPI_FLASH_CMD_WREN   		0x06
#define 	SPI_FLASH_CMD_WRITE   		0x02
//M25P SPI Flash supported commands.
#define 	SPI_FLASH_CMD_WRSR   		0x01
#define 	SPI_FLASH_CS_HIGH()   		GPIO_SetBits(SPI_FLASH_PORT, SPI_FLASH_CS )  //Deselect sFLASH: Chip Select pin high.
#define 	SPI_FLASH_CS_LOW()   		GPIO_ResetBits(SPI_FLASH_PORT, SPI_FLASH_CS) //Select sFLASH: Chip Select pin low.
#define 	SPI_FLASH_DUMMY_BYTE   		0xA5
#define 	SPI_FLASH_S25FL204K_ID   	0x14013
#define 	SPI_FLASH_SPI_PAGESIZE   	0x100
#define 	SPI_FLASH_WIP_FLAG   		0x01
/*
– Page program time: 1.5 ms typical
– Sector erase time (4 kB): 50 ms typical
– Block erase time (64 kB): 500 ms typical
– Chip erase time: 3.5 seconds typical
*/

void 		FlashSpiEraseBulk (void);  //earasing all chip
void 		FlashSpiEraseBlock(uint32_t BlockAddr);
void 		FlashSpiEraseSector (uint32_t SectorAddr);
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
