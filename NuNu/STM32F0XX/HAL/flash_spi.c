/**
  ******************************************************************************
  *
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment            S25FL204K   |
  *          +-----------------------------+---------------+-------------+
  *          |  STM32 SPI Pins             |     sFLASH    |    Pin      |
  *          +-----------------------------+---------------+-------------+
  *          | SPI_CS		               | ChipSelect(/S)|    1        |
  *          | SPI_MISO  				   |   DataOut(Q)  |    2        |
  *          |                             |   VCC         |    3 (3.3 V)|
  *          |                             |   GND         |    4 (0 V)  |
  *          | SPI_MOSI  				   |   DataIn(D)   |    5        |
  *          | SPI_CLK   				   |   Clock(C)    |    6        |
  *          |                             |    VCC        |    7 (3.3 V)|
  *          |                             |    VCC        |    8 (3.3 V)|
  *          +-----------------------------+---------------+-------------+
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_spi.h"

/**
  * @brief  Erases the specified FLASH block. 64k
  * @param  SectorAddr: address of the block to erase.
  * @retval None
  */
void FlashSpiEraseBlock(uint32_t BlockAddr)
{
  /*!< Send write enable instruction */
  FlashSpiWriteEnable();

  /*!< Sector Erase */
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /*!< Send Sector Erase instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_BE);
  /*!< Send SectorAddr high nibble address byte */
  FlashSpiSendByte((BlockAddr & 0xFF0000) >> 16);
  /*!< Send SectorAddr medium nibble address byte */
  FlashSpiSendByte((BlockAddr & 0xFF00) >> 8);
  /*!< Send SectorAddr low nibble address byte */
  FlashSpiSendByte(BlockAddr & 0xFF);
  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  FlashSpiWaitForWriteEnd();
}

/**
  * @brief  Erases the specified FLASH sector. 4k
  * @param  SectorAddr: address of the sector to erase.
  * @retval None
  */
void FlashSpiEraseSector(uint32_t SectorAddr)
{
  /*!< Send write enable instruction */
  FlashSpiWriteEnable();

  /*!< Sector Erase */
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /*!< Send Sector Erase instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_SE);
  /*!< Send SectorAddr high nibble address byte */
  FlashSpiSendByte((SectorAddr & 0xFF0000) >> 16);
  /*!< Send SectorAddr medium nibble address byte */
  FlashSpiSendByte((SectorAddr & 0xFF00) >> 8);
  /*!< Send SectorAddr low nibble address byte */
  FlashSpiSendByte(SectorAddr & 0xFF);
  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  FlashSpiWaitForWriteEnd();
}

/**
  * @brief  Erases the entire FLASH.
  * @param  None
  * @retval None
  */
void FlashSpiEraseBulk(void)
{
  /*!< Send write enable instruction */
  FlashSpiWriteEnable();

  /*!< Bulk Erase */
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /*!< Send Bulk Erase instruction  */
  FlashSpiSendByte(SPI_FLASH_CMD_BE);
  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  FlashSpiWaitForWriteEnd();
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
  *         (Page WRITE sequence).
  * @note   The number of byte can't exceed the FLASH page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
  *         or less than "SPI_FLASH_PAGESIZE" value.
  * @retval None
  */
void FlashSpiWritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /*!< Enable the write access to the FLASH */
  FlashSpiWriteEnable();

  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /*!< Send "Write to Memory " instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_WRITE);
  /*!< Send WriteAddr high nibble address byte to write to */
  FlashSpiSendByte((WriteAddr & 0xFF0000) >> 16);
  /*!< Send WriteAddr medium nibble address byte to write to */
  FlashSpiSendByte((WriteAddr & 0xFF00) >> 8);
  /*!< Send WriteAddr low nibble address byte to write to */
  FlashSpiSendByte(WriteAddr & 0xFF);

  /*!< while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /*!< Send the current byte */
    FlashSpiSendByte(*pBuffer);
    /*!< Point on the next byte to be written */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /*!< Wait the end of Flash writing */
  FlashSpiWaitForWriteEnd();
}

/**
  * @brief  Writes block of data to the FLASH. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the FLASH.
  * @param  WriteAddr: FLASH's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the FLASH.
  * @retval None
  */
void FlashSpiWriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_SPI_PAGESIZE;
  count = SPI_FLASH_SPI_PAGESIZE - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_SPI_PAGESIZE;
  NumOfSingle = NumByteToWrite % SPI_FLASH_SPI_PAGESIZE;

  if (Addr == 0) /*!< WriteAddr is SPI_FLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < SPI_FLASH_PAGESIZE */
    {
      FlashSpiWritePage(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /*!< NumByteToWrite > SPI_FLASH_PAGESIZE */
    {
      while (NumOfPage--)
      {
        FlashSpiWritePage(pBuffer, WriteAddr, SPI_FLASH_SPI_PAGESIZE);
        WriteAddr +=  SPI_FLASH_SPI_PAGESIZE;
        pBuffer += SPI_FLASH_SPI_PAGESIZE;
      }

      FlashSpiWritePage(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /*!< WriteAddr is not SPI_FLASH_PAGESIZE aligned  */
  {
    if (NumOfPage == 0) /*!< NumByteToWrite < SPI_FLASH_PAGESIZE */
    {
      if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > SPI_FLASH_PAGESIZE */
      {
        temp = NumOfSingle - count;

        FlashSpiWritePage(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        FlashSpiWritePage(pBuffer, WriteAddr, temp);
      }
      else
      {
        FlashSpiWritePage(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /*!< NumByteToWrite > SPI_FLASH_PAGESIZE */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_SPI_PAGESIZE;
      NumOfSingle = NumByteToWrite % SPI_FLASH_SPI_PAGESIZE;

      FlashSpiWritePage(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        FlashSpiWritePage(pBuffer, WriteAddr, SPI_FLASH_SPI_PAGESIZE);
        WriteAddr +=  SPI_FLASH_SPI_PAGESIZE;
        pBuffer += SPI_FLASH_SPI_PAGESIZE;
      }

      if (NumOfSingle != 0)
      {
        FlashSpiWritePage(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the FLASH.
  * @retval None
  */
void FlashSpiReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_READ);

  /*!< Send ReadAddr high nibble address byte to read from */
  FlashSpiSendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte to read from */
  FlashSpiSendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte to read from */
  FlashSpiSendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /*!< while there is data to be read */
  {
    /*!< Read a byte from the FLASH */
    *pBuffer = FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE);
    /*!< Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/**
  * @brief  Reads FLASH identification.
  * @param  None
  * @retval FLASH identification
  */
uint32_t FlashSpiReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /*!< Send "RDID " instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_RDID);

  /*!< Read a byte from the FLASH */
  Temp0 = FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE);

  /*!< Read a byte from the FLASH */
  Temp1 = FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE);

  /*!< Read a byte from the FLASH */
  Temp2 = FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE);

  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}

/**
  * @brief  Initiates a read data byte (READ) sequence from the Flash.
  *   This is done by driving the /CS line low to select the device, then the READ
  *   instruction is transmitted followed by 3 bytes address. This function exit
  *   and keep the /CS line low, so the Flash still being selected. With this
  *   technique the whole content of the Flash is read with a single READ instruction.
  * @param  ReadAddr: FLASH's internal address to read from.
  * @retval None
  */
void FlashSpiStartReadSequence(uint32_t ReadAddr)
{
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /*!< Send "Read from Memory " instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_READ);

  /*!< Send the 24-bit address of the address to read from -------------------*/
  /*!< Send ReadAddr high nibble address byte */
  FlashSpiSendByte((ReadAddr & 0xFF0000) >> 16);
  /*!< Send ReadAddr medium nibble address byte */
  FlashSpiSendByte((ReadAddr& 0xFF00) >> 8);
  /*!< Send ReadAddr low nibble address byte */
  FlashSpiSendByte(ReadAddr & 0xFF);
}

/**
  * @brief  Reads a byte from the SPI Flash.
  * @note   This function must be used only if the Start_Read_Sequence function
  *         has been previously called.
  * @param  None
  * @retval Byte Read from the SPI Flash.
  */
uint8_t FlashSpiReadByte(void)
{
  return (FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE));
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t FlashSpiSendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_SendData8(SPI_FLASH, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_ReceiveData8(SPI_FLASH);
}

/**
  * @brief  Enables the write access to the FLASH.
  * @param  None
  * @retval None
  */
void FlashSpiWriteEnable(void)
{
  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /*!< Send "Write Enable" instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_WREN);

  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
  *         status register and loop until write opertaion has completed.
  * @param  None
  * @retval None
  */
void FlashSpiWaitForWriteEnd(void)
{
  uint8_t flashstatus = 0;

  /*!< Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /*!< Send "Read Status Register" instruction */
  FlashSpiSendByte(SPI_FLASH_CMD_RDSR);

  /*!< Loop as long as the memory is busy with a write cycle */
  do
  {
    /*!< Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    flashstatus = FlashSpiSendByte(SPI_FLASH_DUMMY_BYTE);

  }
  while ((flashstatus & SPI_FLASH_WIP_FLAG) == SET); /* Write in progress */

  /*!< Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}
