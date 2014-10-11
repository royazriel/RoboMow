// -------------------------------------------------------------------------------------------------------------------
//
//  File: deca_spinit.h - SPI Interface Initialisation definitions
//
//  Copyright (c) 2008 - DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Billy Verso, March 2010
//
// -------------------------------------------------------------------------------------------------------------------

#ifndef _DECA_SPINIT_H_
#define _DECA_SPINIT_H_

#ifdef __cplusplus
extern "C" {
#endif

// rate in kHz (1000 == 1Mbit/s)
#define DW_DEFAULT_SPI		(3000) //3MHz
#define DW_FAST_SPI			(20000) //20MHz

int spilogenable(int enable);                       // run time enable/disable logging of SPI activity to a file

#ifdef __cplusplus
}
#endif

#endif



