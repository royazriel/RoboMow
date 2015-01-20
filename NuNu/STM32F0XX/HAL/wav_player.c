/*
 * wav_player.c
 *
 *  Created on: Dec 31, 2014
 *      Author: roy
 */

#include "wav_player.h"
#include "flash_spi.h"

static WAVE_FormatTypeDef sWaveFormat;
static ErrorCode sWaveFileStatus = Unvalid_RIFF_ID;
static uint16_t sTIM6ARRValue = 6000;
static int32_t sWaveDataLength = 0;
static uint32_t sSpeechDataOffset = 0x00;
static uint32_t sBufferSwapCounter = 0;
static uint8_t sBuffer1[DMA_BUFFER_SIZE];
static uint8_t sBuffer2[DMA_BUFFER_SIZE];
static uint8_t sPlayingFileId;
static uint8_t sPlayingBuffer = 0;


FileInfoRecord sFileListTable[MAX_FILES_IN_LIST_TABLE] = {
		{  0, "hasta-la-vista-baby.wav"		  ,	0		,17826	 },
		{  1, "ill-be-back.wav"				  ,	17920	,8565    },
		{  2, "its-not-that-easy.wav"		  ,	26624	,9768    },
		{  3, "its-your-sweetheart-again.wav" ,	36864	,15640   },
		{  4, "let-me-out-of-here-1.wav"	  ,	52736	,9214    },
		{  5, "nice-work.wav"				  ,	61952	,5744    },
		{  6, "no-problemo.wav"				  ,	68096	,7191    },
		{  7, "oh-yeah-everything-is-fine.wav", 75776	,14286   },
		{  8, "okay-bye.wav"				  , 90112	,8146    },
		{  9, "R2D2a.wav"					  , 98304	,9662    },
		{ 10, "that-feels-really-powerful.wav", 108032	,15086   },
		{ 11, "tubbyuhoh.wav"				  , 123392	,15818   },
		{ 12, "werent-you-listening.wav"	  , 139264	,14019   },
		{ 13, "wow-1.wav"					  , 153600	,11883   },
		{ 14, "yes-3.wav"					  , 165888	,7612    },
		{ 15, "you-can-do-it.wav"			  , 173568	,6011    },
		{ 16, "you-got-it-1.wav"			  , 179712	,5744    },
		{ 17, "youll-see-very-soon.wav"		  , 185856	,20425   },
		{ 18, "youre-almost-there.wav"		  , 206336	,12417   },
		{ 19, "youre-on-the-right-track.wav"  , 219136	,11883   }
};


static ErrorCode WavPlayerWaveParsing( int fileid );

void WavPlayerPlaySound( int fileId )
{
	UsartPrintf("playing file: %-40s file size: %d\r\n", sFileListTable[fileId].name,sFileListTable[sPlayingFileId].size );
	if( WavPlayerLoad(fileId) != Valid_WAVE_File)
	{
		UsartPrintf("Not a valid wav file\r\n");
		return;
	}

	/* Start TIM6 */
	TIM_Cmd(TIM6, ENABLE);
	sPlayingFileId = fileId;
	sPlayingBuffer = 1;
	FlashSpiReadBuffer( sBuffer2, WAV_DATA_START + sFileListTable[fileId].offset + (sBufferSwapCounter * DMA_BUFFER_SIZE),DMA_BUFFER_SIZE);
	sBufferSwapCounter++;
}

void WavPlayerSwapBuffers()
{
	uint8_t tmp;

	if( sPlayingBuffer == 1 )
	{
		DMA1_Channel3->CCR = 0x0;

		DMA1_Channel3->CNDTR = 0x200;
		DMA1_Channel3->CPAR = DAC_DHR8R1_Address;
		DMA1_Channel3->CMAR = (uint32_t) &sBuffer2;
		DMA1_Channel3->CCR = 0x2091;
		sPlayingBuffer = 2;
		DMA1->IFCR = DMA1_FLAG_TC3;
		FlashSpiReadBuffer( sBuffer1, WAV_DATA_START + sFileListTable[sPlayingFileId].offset + (sBufferSwapCounter * DMA_BUFFER_SIZE),DMA_BUFFER_SIZE);
		sBufferSwapCounter++;
		sWaveDataLength-=DMA_BUFFER_SIZE;
	}
	else
	{
		DMA1_Channel3->CCR = 0x0;
		DMA1_Channel3->CNDTR = 0x200;
		DMA1_Channel3->CPAR = DAC_DHR8R1_Address;
		DMA1_Channel3->CMAR = (uint32_t) &sBuffer1;
		DMA1_Channel3->CCR = 0x2091;
		sPlayingBuffer = 1;
		DMA1->IFCR = DMA1_FLAG_TC3;
		FlashSpiReadBuffer( sBuffer2, WAV_DATA_START + sFileListTable[sPlayingFileId].offset + (sBufferSwapCounter * DMA_BUFFER_SIZE),DMA_BUFFER_SIZE);
		sBufferSwapCounter++;
		sWaveDataLength-=DMA_BUFFER_SIZE;
	}

	if( sWaveDataLength <=0 )
	{
		DMA1_Channel3->CCR = 0x0;
		/* Disable TIM6 */
		TIM_Cmd(TIM6, DISABLE);
		sWaveDataLength = 0;
	}
}

ErrorCode WavPlayerLoad( int fileId )
{
	DAC_InitTypeDef DAC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	sWaveFileStatus = Unvalid_RIFF_ID;

	/* TIM6 Configuration */
	TIM_DeInit(TIM6);

	/* DMA1 channel2 configuration */
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR8R1_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &sBuffer1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, ENABLE);

	/* DAC deinitialize */
	DAC_DeInit();
	DAC_StructInit(&DAC_InitStructure);

	/* Fill DAC InitStructure */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* DAC Channel1: 8bit right---------------------------------------------------*/
	/* DAC Channel1 Init */
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	/* Read the Speech wave file status */
	sWaveFileStatus = WavPlayerWaveParsing( fileId );

	/* TIM6 TRGO selection */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	TIM_SetAutoreload(TIM6, sTIM6ARRValue);

	if (sWaveFileStatus == Valid_WAVE_File)  /* the .WAV file is valid */
	{
		/* Set WaveDataLenght to the Speech wave length */
		sWaveDataLength = sWaveFormat.DataSize;
	}

	return sWaveFileStatus;
}

static ErrorCode WavPlayerWaveParsing( int fileId )
{
	uint32_t temp = 0x00;
	uint32_t extraformatbytes = 0;

	FlashSpiReadBuffer( sBuffer1, sFileListTable[fileId].offset,WAV_DATA_START);
	//memcpy( sBuffer1,veryshort_wav, 44);

	/* Read chunkID, must be 'RIFF'  ----------------------------------------------*/
	temp = ReadUnit(sBuffer1, 0, 4, BigEndian);
	if (temp != CHUNK_ID)
	{
		return(Unvalid_RIFF_ID);
	}

	/* Read the file length ----------------------------------------------------*/
	sWaveFormat.RIFFchunksize = ReadUnit(sBuffer1, 4, 4, LittleEndian);

	/* Read the file format, must be 'WAVE' ------------------------------------*/
	temp = ReadUnit(sBuffer1, 8, 4, BigEndian);
	if (temp != FILE_FORMAT)
	{
		return(Unvalid_WAVE_Format);
	}

	/* Read the format chunk, must be'fmt ' --------------------------------------*/
	temp = ReadUnit(sBuffer1, 12, 4, BigEndian);
	if (temp != FORMAT_ID)
	{
		return(Unvalid_FormatChunk_ID);
	}
	/* Read the length of the 'fmt' data, must be 0x10 -------------------------*/
	temp = ReadUnit(sBuffer1, 16, 4, LittleEndian);
	if (temp != 0x10)
	{
		extraformatbytes = 1;
	}
	/* Read the audio format, must be 0x01 (PCM) -------------------------------*/
	sWaveFormat.FormatTag = ReadUnit(sBuffer1, 20, 2, LittleEndian);
	if (sWaveFormat.FormatTag != WAVE_FORMAT_PCM)
	{
		return(Unsupporetd_FormatTag);
	}

	/* Read the number of channels, must be 0x01 (Mono) ------------------------*/
	sWaveFormat.NumChannels = ReadUnit(sBuffer1, 22, 2, LittleEndian);
	if (sWaveFormat.NumChannels != CHANNEL_MONO)
	{
		return(Unsupporetd_Number_Of_Channel);
	}

	/* Read the Sample Rate ----------------------------------------------------*/
	sWaveFormat.SampleRate = ReadUnit(sBuffer1, 24, 4, LittleEndian);
	/* Update the OCA value according to the .WAV file Sample Rate */
	switch (sWaveFormat.SampleRate)
	{
	case SAMPLE_RATE_8000 :
	  sTIM6ARRValue = 937;
	  break; /* 8KHz = 48MHz / 6000 */
	case SAMPLE_RATE_11025:
	  sTIM6ARRValue = 4353;
	  break; /* 11.025KHz = 48MHz / 4353 */
	case SAMPLE_RATE_22050:
	  sTIM6ARRValue = 2176;
	  break; /* 22.05KHz = 48MHz / 2176 */
	case SAMPLE_RATE_44100:
	  sTIM6ARRValue = 1088;
	  break; /* 44.1KHz = 48MHz / 1088 */
	default:
	  return(Unsupporetd_Sample_Rate);
	}

	/* Read the Byte Rate ------------------------------------------------------*/
	sWaveFormat.ByteRate = ReadUnit(sBuffer1, 28, 4, LittleEndian);

	/* Read the block alignment ------------------------------------------------*/
	sWaveFormat.BlockAlign = ReadUnit(sBuffer1, 32, 2, LittleEndian);

	/* Read the number of bits per sample --------------------------------------*/
	sWaveFormat.BitsPerSample = ReadUnit(sBuffer1, 34, 2, LittleEndian);
	if (sWaveFormat.BitsPerSample != BITS_PER_SAMPLE_8)
	{
		return(Unsupporetd_Bits_Per_Sample);
	}
	sSpeechDataOffset = 36;
	/* If there is Extra format bytes, these bytes will be defined in "Fact Chunk" */
	if (extraformatbytes == 1)
	{
		/* Read th Extra format bytes, must be 0x00 ------------------------------*/
		temp = ReadUnit(sBuffer1, 36, 2, LittleEndian);
		if (temp != 0x00)
		{
			return(Unsupporetd_ExtraFormatBytes);
		}
		/* Read the Fact chunk, must be 'fact' -----------------------------------*/
		temp = ReadUnit(sBuffer1, 38, 4, BigEndian);
		if (temp != FACT_ID)
		{
			return(Unvalid_FactChunk_ID);
		}
		/* Read Fact chunk data Size ---------------------------------------------*/
		temp = ReadUnit(sBuffer1, 42, 4, LittleEndian);

		sSpeechDataOffset += 10 + temp;
	}
	/* Read the Data chunk, must be 'data' ---------------------------------------*/
	temp = ReadUnit(sBuffer1, sSpeechDataOffset, 4, BigEndian);
	sSpeechDataOffset += 4;
	if (temp != DATA_ID)
	{
		return(Unvalid_DataChunk_ID);
	}

	/* Read the number of sample data ------------------------------------------*/
	sWaveFormat.DataSize = ReadUnit(sBuffer1, sSpeechDataOffset, 4, LittleEndian);
	sSpeechDataOffset += 4;
	sBufferSwapCounter = 0;
	FlashSpiReadBuffer( sBuffer1, WAV_DATA_START + sFileListTable[fileId].offset,DMA_BUFFER_SIZE);
	//memcpy( sBuffer1,veryshort_wav + WAV_DATA_START, DMA_BUFFER_SIZE);
	sBufferSwapCounter++;
	return(Valid_WAVE_File);
}

uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat)
{
  uint32_t index = 0;
  uint32_t temp = 0;

  for (index = 0; index < NbrOfBytes; index++)
  {
    temp |= buffer[idx + index] << (index * 8);
  }

  if (BytesFormat == BigEndian)
  {
    temp = __REV(temp);
  }
  return temp;
}
