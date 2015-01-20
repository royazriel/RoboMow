/*
 * wav_player.h
 *
 *  Created on: Dec 31, 2014
 *      Author: roy
 */

#ifndef WAV_PLAYER_H_
#define WAV_PLAYER_H_

#include "hardware_config.h"

/**
@code
  .WAV file format :
 Endian      Offset      Length      Contents
  big         0           4 bytes     'RIFF'             // 0x52494646
  little      4           4 bytes     <file length - 8>
  big         8           4 bytes     'WAVE'             // 0x57415645
Next, the fmt chunk describes the sample format:
  big         12          4 bytes     'fmt '          // 0x666D7420
  little      16          4 bytes     0x00000010      // Length of the fmt data (16 bytes)
  little      20          2 bytes     0x0001          // Format tag: 1 = PCM
  little      22          2 bytes     <channels>      // Channels: 1 = mono, 2 = stereo
  little      24          4 bytes     <sample rate>   // Samples per second: e.g., 22050
  little      28          4 bytes     <bytes/second>  // sample rate * block align
  little      32          2 bytes     <block align>   // channels * bits/sample / 8
  little      34          2 bytes     <bits/sample>   // 8 or 16
Finally, the data chunk contains the sample data:
  big         36          4 bytes     'data'        // 0x64617461
  little      40          4 bytes     <length of the data block>
  little      44          *           <sample data>
@endcode
*/

typedef struct __FileInfoRecord
{
	uint8_t 	id;
	char		name[40];
	uint32_t	offset;
	uint32_t	size;
}FileInfoRecord;


typedef enum
{
  LittleEndian,
  BigEndian
}Endianness;

typedef struct
{
  uint32_t  RIFFchunksize;
  uint16_t  FormatTag;
  uint16_t  NumChannels;
  uint32_t  SampleRate;
  uint32_t  ByteRate;
  uint16_t  BlockAlign;
  uint16_t  BitsPerSample;
  uint32_t  DataSize;
}
WAVE_FormatTypeDef;
typedef enum
{
  Valid_WAVE_File = 0,
  Unvalid_RIFF_ID,
  Unvalid_WAVE_Format,
  Unvalid_FormatChunk_ID,
  Unsupporetd_FormatTag,
  Unsupporetd_Number_Of_Channel,
  Unsupporetd_Sample_Rate,
  Unsupporetd_Bits_Per_Sample,
  Unvalid_DataChunk_ID,
  Unsupporetd_ExtraFormatBytes,
  Unvalid_FactChunk_ID
} ErrorCode;

#define SpeechReadAddr         0x0  /* Speech wave start read address */

#define MAX_FILES_IN_LIST_TABLE			  	20
#define DMA_BUFFER_SIZE						512
#define WAV_DATA_START						44
/* Audio Play STATUS */
#define AudioPlayStatus_STOPPED       0
#define AudioPlayStatus_PLAYING	      1
#define AudioPlayStatus_PAUSED        2

#define  CHUNK_ID                            0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT                         0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID                           0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID                             0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID                             0x66616374  /* correspond to the letters 'fact' */
#define  WAVE_FORMAT_PCM                     0x01
#define  FORMAT_CHNUK_SIZE                   0x10
#define  CHANNEL_MONO                        0x01
#define  SAMPLE_RATE_8000                    8000
#define  SAMPLE_RATE_11025                   11025
#define  SAMPLE_RATE_22050                   22050
#define  SAMPLE_RATE_44100                   44100
#define  BITS_PER_SAMPLE_8                   8
#define  WAVE_DUMMY_BYTE                     0xA5
#define  DAC_DHR8R1_Address                  0x40007410
#define  DAC_DHR12R1_Address                 0x40007408
#define  DAC_DHR12L1_Address                 0x4000740C

void WavPlayerPlaySound( int fileId);
void WavPlayerSwapBuffers();
void WavPlayerLoadNextBuffer();
ErrorCode WavPlayerLoad( int fileId );
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat);

#endif /* WAV_PLAYER_H_ */
