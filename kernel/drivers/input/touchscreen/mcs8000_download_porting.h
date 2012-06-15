
//--------------------------------------------------------
//
//
//	Melfas MCS7000 Series Download base v1.0 2010.04.05
//
//
//--------------------------------------------------------

#ifndef __MELFAS_DOWNLOAD_PORTING_H_INCLUDED__
#define __MELFAS_DOWNLOAD_PORTING_H_INCLUDED__

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/jiffies.h>

//============================================================
//
//	Porting order
//
//============================================================
/*

1. melfas_download_porting.h
   - Check typedef	[melfas_download_porting.h]

   - Check download options	[melfas_download_porting.h]

   - Add Port control code  ( CE, RESETB, I2C,... )	[melfas_download_porting.h]

   - Apply your delay function ( inside mcsdl_delaly() )	[melfas_download.c]
      Modify delay parameter constant ( ex. MCSDL_DELAY_5MS ) to make it fit to your delay function.

   - Rename 'uart_printf()' to your console print function for debugging. [melfas_download_porting.h]
   	  or, define uart_printf() as different function properly.

   - Check Watchdog timer, Interrupt factor

   - Including Melfas binary .c file

   - Basenad dealy function
      fill up mcsdl_delay()

   - Implement processing external Melfas binary .bin file.

*/

//============================================================
//
//	Type define
//
//============================================================

typedef char				INT8;
typedef unsigned char		UINT8;
//typedef unsigned char		uint8_t;
typedef short				INT16;
typedef unsigned short		UINT16;
//typedef unsigned short	uint16_t;
typedef int					INT32;
typedef unsigned int		UINT32;
typedef unsigned char		BOOLEAN;


#ifndef TRUE
#define TRUE 				(1==1)
#endif

#ifndef FALSE
#define FALSE 				(1==0)
#endif

#ifndef NULL
#define NULL 				0
#endif


//============================================================
//
//	Porting Download Options
//
//============================================================

// For printing debug information. ( Please check 'printing function' )
#define MELFAS_ENABLE_DBG_PRINT											1
#define MELFAS_ENABLE_DBG_PROGRESS_PRINT								1

// For delay function test. ( Disable after Porting is finished )
#define MELFAS_ENABLE_DELAY_TEST										0



//============================================================
//
//	IO Control poting.
//
//	Fill 'Using signal' up only.
// 	See MCSDL_USE_VDD_CONTROL,
//		MCSDL_USE_CE_CONTROL,
//
//============================================================

//----------------
// VDD
//----------------
#if MCSDL_USE_VDD_CONTROL
#define mcsdl_vdd_on()                  gpio_set_value(GPIO_TSP_LDO_ON, 1)
#define mcsdl_vdd_off()                 gpio_set_value(GPIO_TSP_LDO_ON, 0)

#define MCSDL_VDD_SET_HIGH()            mcsdl_vdd_on()
#define MCSDL_VDD_SET_LOW()             mcsdl_vdd_off()
#else
#define MCSDL_VDD_SET_HIGH()            // Nothing
#define MCSDL_VDD_SET_LOW()             // Nothing
#endif

//----------------
// CE
//----------------
#if MCSDL_USE_CE_CONTROL
#define MCSDL_CE_SET_HIGH()   	       gpio_set_value(TOUCH_EN, 1)
#define MCSDL_CE_SET_LOW()      	   gpio_set_value(TOUCH_EN, 0)
#define MCSDL_CE_SET_OUTPUT()   	   gpio_tlmm_config(GPIO_CFG(TOUCH_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE)
#else
#define MCSDL_CE_SET_HIGH()				// Nothing
#define MCSDL_CE_SET_LOW()				// Nothing
#define MCSDL_CE_SET_OUTPUT()			// Nothing
#endif


//----------------
// RESETB
//----------------
#if MCSDL_USE_RESETB_CONTROL
#define MCSDL_RESETB_SET_HIGH()         gpio_set_value(GPIO_TOUCH_INT, 1)
#define MCSDL_RESETB_SET_LOW()          gpio_set_value(GPIO_TOUCH_INT, 0)
#define MCSDL_RESETB_SET_OUTPUT()       s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(1));
#define MCSDL_RESETB_SET_INPUT()        s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(0));
#define MCSDL_RESETB_SET_ALT()  	    s3c_gpio_cfgpin(GPIO_TOUCH_INT, S3C_GPIO_SFN(0xF));
#else
#define MCSDL_RESETB_SET_HIGH()
#define MCSDL_RESETB_SET_LOW()
#define MCSDL_RESETB_SET_OUTPUT()
#define MCSDL_RESETB_SET_INPUT()
#endif


//------------------
// I2C SCL & SDA
//------------------

#define MCSDL_GPIO_SCL_SET_HIGH()		gpio_set_value(GPIO_TSP_SCL_28V, 1)
#define MCSDL_GPIO_SCL_SET_LOW()		gpio_set_value(GPIO_TSP_SCL_28V, 0)

#define MCSDL_GPIO_SDA_SET_HIGH()		gpio_set_value(GPIO_TSP_SDA_28V, 1)
#define MCSDL_GPIO_SDA_SET_LOW()		gpio_set_value(GPIO_TSP_SDA_28V, 0)

#define MCSDL_GPIO_SCL_SET_OUTPUT()	    s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_SFN(1));
#define MCSDL_GPIO_SCL_SET_INPUT()	    s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_SFN(0));
#define MCSDL_GPIO_SCL_SET_ALT()		s3c_gpio_cfgpin(GPIO_TSP_SCL_28V, S3C_GPIO_SFN(2));

#define MCSDL_GPIO_SDA_SET_OUTPUT()	    s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_SFN(1));
#define MCSDL_GPIO_SDA_SET_INPUT()	    s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_SFN(0));
#define MCSDL_GPIO_SDA_SET_ALT()		s3c_gpio_cfgpin(GPIO_TSP_SDA_28V, S3C_GPIO_SFN(2));

#define MCSDL_GPIO_SDA_IS_HIGH()		((gpio_get_value(GPIO_TSP_SDA_28V) > 0) ? 1 : 0)


//#if MCSDL_USING_HW_I2C
//#define MCSDL_SET_GPIO_I2C()						____HERE!_____
//#define MCSDL_SET_HW_I2C()							____HERE!_____
//#else
//#define MCSDL_SET_GPIO_I2C()						// Nothing
//#define MCSDL_SET_HW_I2C()							// Nothing
//#endif




//============================================================
//
//	Delay parameter setting
//
//	These are used on 'mcsdl_delay()'
//
//============================================================

#define MCSDL_DELAY_1US								    1
#define MCSDL_DELAY_2US								    2
#define MCSDL_DELAY_3US								    3
#define MCSDL_DELAY_5US								    5
#define MCSDL_DELAY_7US 								7
#define MCSDL_DELAY_10US 							   10
#define MCSDL_DELAY_15US							   15
#define MCSDL_DELAY_20US							   20
#define MCSDL_DELAY_40US                               40

#define MCSDL_DELAY_100US							  100
#define MCSDL_DELAY_150US							  150
#define MCSDL_DELAY_300US                             300
#define MCSDL_DELAY_500US             				  500
#define MCSDL_DELAY_800US							  800


#define MCSDL_DELAY_1MS								 1000
#define MCSDL_DELAY_5MS								 5000
#define MCSDL_DELAY_10MS							10000
#define MCSDL_DELAY_25MS							25000
#define MCSDL_DELAY_30MS							30000
#define MCSDL_DELAY_40MS							40000
#define MCSDL_DELAY_45MS							45000
#define MCSDL_DELAY_60MS                            60000


//============================================================
//
//	Defence External Effect
//
//============================================================
#if 0

#define MELFAS_DISABLE_BASEBAND_ISR()				____HERE!_____
#define MELFAS_DISABLE_WATCHDOG_TIMER_RESET()		____HERE!_____

#define MELFAS_ROLLBACK_BASEBAND_ISR()				____HERE!_____
#define MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET()		____HERE!_____

#else

#define MELFAS_DISABLE_BASEBAND_ISR()				// Nothing
#define MELFAS_DISABLE_WATCHDOG_TIMER_RESET()		// Nothing

#define MELFAS_ROLLBACK_BASEBAND_ISR()				// Nothing
#define MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET()		// Nothing

#endif


typedef enum
{
    /* chipset independent */
    TOUCH_SCREEN_TESTMODE_ENTER,
    TOUCH_SCREEN_TESTMODE_EXIT,
    TOUCH_SCREEN_TESTMODE_GET_OP_MODE,
    TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST,
    TOUCH_SCREEN_TESTMODE_GET_THRESHOLD,
    TOUCH_SCREEN_TESTMODE_GET_DELTA,
    TOUCH_SCREEN_TESTMODE_GET_REFERENCE,
    TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_LOW,
    TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_HIGH,

    /* chipset dependent */
    /* melfas */
    TOUCH_SCREEN_TESTMODE_GET_INSPECTION,
    TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_LOW,
    TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_HIGH,

    TOUCH_SCREEN_TESTMODE_MAX
}touch_screen_testmode_ioctrl_t;

typedef enum
{
    TOUCH_SCREEN_NORMAL,
    TOUCH_SCREEN_TESTMODE
}touch_screen_operation_t;


typedef struct
{
    /* chipset independent */
    const char *ven_name;
    const char *chip_name;
    uint16_t ven_id;
    uint16_t mod_rev;
    uint16_t fw_ver;

    uint16_t delta_point_num;
    uint16_t x_channel_num;
    uint16_t y_channel_num;
    uint16_t reference_size; // byte size of reference value
    uint16_t reference_spec_low;
    uint16_t reference_spec_high;
    uint16_t inspection_spec_low;
    uint16_t inspection_spec_high;

    /* chipset dependent */
    /* melfas */
    uint16_t* inspection_spec_table;
}touch_screen_driver_info_t;

typedef union
{
    /* chipset independent */
    uint16_t threshold;
    uint16_t delta;
    uint16_t reference;
    uint16_t bad_point;

    /* chipset dependent */
    /* melfas */
    uint16_t inspection;
}touch_screen_testmode_info_t;

typedef struct
{
    uint16_t bad_point;
    touch_screen_testmode_info_t *table;
}touch_screen_inspection_t;


typedef struct
{
    /* chipset independent */
    touch_screen_driver_info_t *driver;
    touch_screen_inspection_t reference;

    /* chipset dependent */
    /* melfas */
    touch_screen_inspection_t inspection;
}touch_screen_info_t;

typedef struct
{
    touch_screen_driver_info_t ts_info;
    touch_screen_operation_t ts_mode;
    int (*test_mode)(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num);
} touch_screen_driver_t;

typedef struct
{
    touch_screen_info_t tsp_info;
    bool device_state;
    touch_screen_driver_t *driver;
} touch_screen_t;


/* Touch Screen Interface Specification Multi Touch (V0.5) */

/* REGISTERS */
#define MCSTS_STATUS_REG        0x00 //Status
#define MCSTS_MODE_CONTROL_REG  0x01 //Mode Control
#define MCSTS_RESOL_HIGH_REG    0x02 //Resolution(High Byte)
#define MCSTS_RESOL_X_LOW_REG   0x08 //Resolution(X Low Byte)
#define MCSTS_RESOL_Y_LOW_REG	0x0A //Resolution(Y Low Byte)
#define MCSTS_INPUT_INFO_REG    0x10 //Input Information
#define MCSTS_POINT_HIGH_REG    0x11 //Point(High Byte)
#define MCSTS_POINT_X_LOW_REG   0x12 //Point(X Low Byte)
#define MCSTS_POINT_Y_LOW_REG   0x13 //Point(Y Low Byte)
#define MCSTS_STRENGTH_REG      0x14 //Strength
#define MCSTS_MODULE_VER_REG    0x63 //H/W Module Revision
#define MCSTS_FIRMWARE_VER_REG  0x64 //F/W Version


/***************************************************************************
* Define local constants
***************************************************************************/

#define TS_MELFAS_VENDOR_NAME							"MELFAS"
#define TS_MELFAS_VENDOR_CHIP_NAME						"MMS100"
#define TS_MELFAS_VENDOR_ID								0x50
#define TS_MELFAS_MODULE_REV							0x01
#define TS_MELFAS_FIRMWARE_VER							0x05

#define TS_MELFAS_TESTMODE_DATA_CTRL_REG		        0xA0
#define TS_MELFAS_TESTMODE_DATA_READ_REG		        0xAF

#define TS_MELFAS_TESTMODE_CTRL_REG						0x09
#define TS_MELFAS_TESTMODE_TSP_THRESHOLD_REG			0xAC
#define TS_MELFAS_TESTMODE_GET_DATA_REG		            0xB2

#define TS_MELFAS_SENSING_CHANNEL_NUM					10
#define TS_MELFAS_EXCITING_CHANNEL_NUM					14


#endif

