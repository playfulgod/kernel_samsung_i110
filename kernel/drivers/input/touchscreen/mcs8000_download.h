//--------------------------------------------------------
//
//
//	Melfas MCS7000 Series Download base v1.0 2010.04.05
//
//
//--------------------------------------------------------


#ifndef __MELFAS_FIRMWARE_DOWNLOAD_H__
#define __MELFAS_FIRMWARE_DOWNLOAD_H__



//=====================================================================
//
//   MELFAS Firmware download pharameters
//
//=====================================================================

#define MELFAS_TRANSFER_LENGTH					(32/8)		// Fixed value
#define MELFAS_FIRMWARE_MAX_SIZE				(32*1024)

#define MELFAS_2CHIP_DOWNLOAD_ENABLE            0       // 0 : 1Chip Download, 1: 2Chip Download

//----------------------------------------------------
//   ISP Mode
//----------------------------------------------------
#define ISP_MODE_ERASE_FLASH					0x01
#define ISP_MODE_SERIAL_WRITE					0x02
#define ISP_MODE_SERIAL_READ					0x03
#define ISP_MODE_NEXT_CHIP_BYPASS				0x04


//----------------------------------------------------
//   Return values of download function
//----------------------------------------------------
#define MCSDL_RET_SUCCESS						0x00
#define MCSDL_RET_ERASE_FLASH_VERIFY_FAILED		0x01
#define MCSDL_RET_PROGRAM_VERIFY_FAILED			0x02

#define MCSDL_RET_PROGRAM_SIZE_IS_WRONG			0x10
#define MCSDL_RET_VERIFY_SIZE_IS_WRONG			0x11
#define MCSDL_RET_WRONG_BINARY					0x12

#define MCSDL_RET_READING_HEXFILE_FAILED		0x21
#define MCSDL_RET_FILE_ACCESS_FAILED			0x22
#define MCSDL_RET_MELLOC_FAILED					0x23

#define MCSDL_RET_WRONG_MODULE_REVISION			0x30


//----------------------------------------------------
//	When you can't control VDD nor CE.
//	Set this value 1
//	Then Melfas Chip can prepare chip reset.
//----------------------------------------------------

#define MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD 	0		// If 'enable download command' is needed ( Pinmap dependent option ).

//============================================================
//
//	Port setting. ( Melfas preset this value. )
//
//============================================================

// If want to set Enable : Set to 1

#define MCSDL_USE_CE_CONTROL						0
#define MCSDL_USE_VDD_CONTROL						1
#define MCSDL_USE_RESETB_CONTROL                    1

void mcsdl_vdd_on(void);
void mcsdl_vdd_off(void);

//----------------
// VDD
//----------------
#define mcsdl_vdd_on()                  gpio_set_value(GPIO_TSP_LDO_ON, 1)
#define mcsdl_vdd_off()                 gpio_set_value(GPIO_TSP_LDO_ON, 0)

//============================================================
//
//	Porting factors for Baseband
//
//============================================================
#include "mcs8000_download_porting.h"


//----------------------------------------------------
//	Functions
//----------------------------------------------------
int mcsdl_download_binary_data(INT32 hw_ver);			// with binary type .c   file.
int mcsdl_download_binary_file(void);			// with binary type .bin file.
#if MELFAS_ENABLE_DELAY_TEST					// For initial porting test.
void mcsdl_delay_test(INT32 nCount);
#endif


#endif		//#ifndef __MELFAS_FIRMWARE_DOWNLOAD_H__

