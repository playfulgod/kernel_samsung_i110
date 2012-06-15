/****************************************************************************
**
** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2011
**
** AUTHOR       : Vikas Gupta  			@LDK@
** DESCRIPTION: wtlfota_idpram.h: copied from Viswanath, Puttagunta's dpram.h.  removed unused stuff  @LDK@
** REFERENCES: Stealth dpram driver (dpram.c/.h)
*   This program is free software; you can redistribute it and/or modify it
*   under the terms of the GNU General Public License version 2 as published
*   by the Free Software Foundation.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program; if not, write to the Free Software Foundation, Inc., 59
*   Temple Place, Suite 330, Boston, MA 02111-1307, USA.
*
****************************************************************************/

#ifndef __WTLFOTA_IDPRAM_H__
#define __WTLFOTA_IDPRAM_H__

/* 16KB Size */
#define WTLFOTA_DPRAM_SIZE										0x4000
/* Memory Address */
#define WTLFOTA_DPRAM_START_ADDRESS 							0x0000


/*CHNGED FOR LTE - Swapped the address*/
//#define WTLFOTA_DPRAM_PDA2PHONE_INTERRUPT_ADDRESS			(WTLFOTA_DPRAM_START_ADDRESS + 0x3FFE)
//#define WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS			(WTLFOTA_DPRAM_START_ADDRESS + 0x3FFC)
#define WTLFOTA_DPRAM_PDA2PHONE_INTERRUPT_ADDRESS			(WTLFOTA_DPRAM_START_ADDRESS + 0x3FFC)
#define WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS			(WTLFOTA_DPRAM_START_ADDRESS + 0x3FFE)

#define WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE						2

/*CHNGED FOR LTE -*/
//#define WTLFOTA_DPRAM_START_ADDRESS_PHYS                 0x30000000		//Need to verify this.
//#define WTLFOTA_DPRAM_START_ADDRESS_PHYS 		    0x98000000          // Aegis/STV
#define WTLFOTA_DPRAM_START_ADDRESS_PHYS 		    0xED000000          // Viper
#define WTLFOTA_IDPRAM_SFR_MIFCON_PHYSICAL_ADDR  0xED008000          // Viper
#define WTLFOTA_DPRAM_SHARED_BANK                              0x00000000		// In InstincQ, this has an offset from WTLFOTA_DPRAM_START_ADDRESS_PHYS. In Via6410 no offset			
#define IDPRAM_SFR_SIZE                                                     0x1C

// Viper //DPRAM*3
//#define WTLFOTA_DPRAM_SHARED_BANK_SIZE				0x4000*3		// 16 * 1024 bytes
#define WTLFOTA_DPRAM_SHARED_BANK_SIZE				0x4000		// 16 * 1024 bytes

#define MAX_INDEX			2

#define IOC_SEC_MAGIC		(0xf0)
#define DPRAM_PHONE_ON		_IO(IOC_SEC_MAGIC, 0xc0)
#define DPRAM_PHONE_OFF		_IO(IOC_SEC_MAGIC, 0xc1)
#define DPRAM_PHONE_GETSTATUS	_IOR(IOC_SEC_MAGIC, 0xc2, unsigned int)
#define DPRAM_PHONE_RESET	_IO(IOC_SEC_MAGIC, 0xc5)
#define DPRAM_PHONE_RAMDUMP_ON	_IO(IOC_SEC_MAGIC, 0xc6)
#define DPRAM_PHONE_RAMDUMP_OFF	_IO(IOC_SEC_MAGIC, 0xc7)
#define DPRAM_EXTRA_MEM_RW	_IOWR(IOC_SEC_MAGIC, 0xc8, unsigned long)
#define DPRAM_MEM_RW		_IOWR(IOC_SEC_MAGIC, 0xc9, unsigned long)
#define DPRAM_TEST		_IO(IOC_SEC_MAGIC, 0xcf) /* TODO: reimplement. Dummy for now */


#define TRUE	1
#define FALSE	0

/* interrupt masks.*/
#define INT_MASK_VALID			0x0080
#define INT_MASK_COMMAND		0x0040
#define INT_MASK_REQ_ACK_F		0x0020
#define INT_MASK_REQ_ACK_R		0x0010
#define INT_MASK_RES_ACK_F		0x0008
#define INT_MASK_RES_ACK_R		0x0004
#define INT_MASK_SEND_F			0x0002
#define INT_MASK_SEND_R			0x0001

#define INT_MASK_CMD_INIT_START		0x0001
#define INT_MASK_CMD_INIT_END		0x0002
#define INT_MASK_CMD_REQ_ACTIVE		0x0003
#define INT_MASK_CMD_RES_ACTIVE		0x0004
#define INT_MASK_CMD_REQ_TIME_SYNC	0x0005
/* #define INT_MASK_CMD_FLIP_ASSERT 	0x0007  cp sleep nack*/
#define INT_MASK_CMD_DPRAM_DOWN_NACK 	0x0007 /* cp sleep nack*/
#define INT_MASK_CMD_PHONE_START 	0x0008
#define INT_MASK_CMD_ERR_DISPLAY	0x0009
//#define INT_MASK_CMD_PHONE_DEEP_SLEEP	0x000A
#define INT_MASK_CMD_NV_REBUILDING	0x000B
//#define INT_MASK_CMD_EMER_DOWN		0x000C
#define INT_MASK_CMD_PIF_INIT_DONE     	0x000D
#define INT_MASK_CMD_SILENT_NV_REBUILDING	0x000E
#define INT_MASK_CMD_NORMAL_POWER_OFF		0x000F

#define INT_MASK_CMD_PDA_SLEEP           0x000C  // PDA Sleep Interrupt to Modem 0xCD
#define INT_MASK_CMD_DPRAM_DOWN          0x000C  // DPRAM_DOWN Interrupt from Modem 0xCB
#define INT_MASK_CMD_PDA_WAKEUP          0x000A  // PDA Wake up Interrupt to Modem 0xCC
#define INT_MASK_CMD_CP_WAKEUP_START     0x000A  // CP Send ack CE to PDA

#define INT_COMMAND(x)			(INT_MASK_VALID | INT_MASK_COMMAND | x)
#define INT_NON_COMMAND(x)		(INT_MASK_VALID | x)

/* special interrupt cmd indicating modem boot failure. */
#define INT_POWERSAFE_FAIL              0xDEAD

/* 16KB Size */
#define DPRAM_SIZE					0x4000
/* Memory Address */
#define DPRAM_START_ADDRESS 				0x0000
#define DPRAM_MAGIC_CODE_ADDRESS			(DPRAM_START_ADDRESS)
#define DPRAM_ACCESS_ENABLE_ADDRESS			(DPRAM_START_ADDRESS + 0x0002)

#define DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS		(DPRAM_START_ADDRESS + 0x0004)
#define DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS		(DPRAM_PDA2PHONE_FORMATTED_START_ADDRESS)
#define DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS		(DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS + 0x0002)
#define DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS	(DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS + 0x0004)
#define DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE		(1020+1024)	/* 0x03FC */


struct _mem_param {
	unsigned short addr;
	unsigned long data;
	int dir;
};

typedef struct wtlfota_dpram_device {
	/* WTLFOTA_DPRAM memory addresses */
	u_int16_t mask_req_ack;
	u_int16_t mask_res_ack;
	u_int16_t mask_send;
} wtlfota_dpram_device_t;


/* currently supported gpios: */
/* #define GPIO_TRANSLATION_LIST_LEN 4 */
/* static gpio_name_value_pair_t GPIO_TRANSLATION_LIST[GPIO_TRANSLATION_LIST_LEN]={ */
/*   {"GPIO_PHONE_ON", GPIO_PHONE_ON}, */
/*   {"GPIO_PHONE_RST_N", GPIO_PHONE_RST_N}, */
/*   {"GPIO_DPRAM_INT", GPIO_DPRAM_INT_N}, */
/*   {"GPIO_PHONE_ACTIVE", GPIO_PHONE_ACTIVE},  */
/* }; */
struct _gpio_param {
#define _WTLFOTA_GPIO_PARAM_NAME_LENGTH 20
  char name[_WTLFOTA_GPIO_PARAM_NAME_LENGTH];
  int data;
#define _WTLFOTA_GPIO_OP_WRITE 1
#define _WTLFOTA_GPIO_OP_READ 0
  int op;//1: write. 0: read. others could be added later
};

/*ioctl cmd numbers. we use the same magic number dpram is using for now since the two are not up at the same time.*/
#define IOC_SEC_MAGIC		(0xf0)
/* #define DPRAM_PHONE_ON		_IO(IOC_SEC_MAGIC, 0xc0) */
#define DPRAM_GPIO_OP		_IO(IOC_SEC_MAGIC, 0xc0)
/* #define DPRAM_PHONE_OFF		_IO(IOC_SEC_MAGIC, 0xc1) */
/* #define DPRAM_PHONE_GETSTATUS	_IOR(IOC_SEC_MAGIC, 0xc2, unsigned int) */
/* #define DPRAM_PHONE_RAMDUMP_ON	_IO(IOC_SEC_MAGIC, 0xc6) */
/* #define DPRAM_PHONE_RAMDUMP_OFF	_IO(IOC_SEC_MAGIC, 0xc7) */
/* #define DPRAM_EXTRA_MEM_RW	_IOWR(IOC_SEC_MAGIC, 0xc8, unsigned long) */
/* #define DPRAM_TEST		_IO(IOC_SEC_MAGIC, 0xcf) /\* TODO: reimplement. Dummy for now *\/ */

#define IOCTL_DEVICE "/dev/dpramctl"
#define IOCTL_DEVICE_NAME "dpramctl"
#define MMAP_DEVICE "/dev/uio0"

#define WRITE_TO_DPRAM(dest, src, size) \
	_memcpy((void *)(DPRAM_VBASE + dest), src, size)

#define READ_FROM_DPRAM(dest, src, size) \
	_memcpy(dest, (void *)(DPRAM_VBASE + src), size)

void dpram_clear(void);
void ClearPendingInterruptFromModem(void);

#endif	/* __WTLFOTA_IDPRAM_H__ */
