/****************************************************************************
 **
 ** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2011
 **
 ** AUTHOR       : Song Wei  			@LDK@
 **                WTLFOTA_DPRAM Device Driver for Via6410
 **			Reference: Via6419 DPRAM driver (dpram.c/.h)
 *
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
#ifndef _HSDPA_WTLFOTA_IDPRAM
#define _HSDPA_WTLFOTA_IDPRAM
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/clk.h>
//#include <asm/io.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-mem.h>
#include <mach/gpio.h>
#include <mach/param.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/kernel_sec_common.h>

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/ip.h>
#include <linux/icmp.h>
#include <linux/time.h>
#include <linux/if_arp.h>
#include <linux/suspend.h>

#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/time.h>

#include "dpram_uio_driver.h"
#include "wtlfota_idpram.h"




/***************************************************************************/
/*                              GPIO SETTING                               */
/***************************************************************************/
#include <mach/gpio.h>

extern unsigned int HWREV;

#define GPIO_LEVEL_LOW				0
#define GPIO_LEVEL_HIGH				1

// Viper
//#define IRQ_WTLFOTA_DPRAM_INT_N         IRQ_EINT8
#define IRQ_WTLFOTA_DPRAM_INT_N        IRQ_MSM
#define IRQ_HOST_WAKEUP                        IRQ_EINT(29)//IRQ_EINT16_31
#define IRQ_PHONE_ACTIVE                        IRQ_EINT15

static u8 is_net_stopped = 0;
static int phone_sync;
static void __iomem *dpram_base;
#define DPRAM_VBASE dpram_base

/*	S5PV210 Interanl Dpram Special Function Register 	*/
#define IDPRAM_MIFCON_INT2APEN      (1<<2)
#define IDPRAM_MIFCON_INT2MSMEN     (1<<3)
#define IDPRAM_MIFCON_DMATXREQEN_0  (1<<16)
#define IDPRAM_MIFCON_DMATXREQEN_1  (1<<17)
#define IDPRAM_MIFCON_DMARXREQEN_0  (1<<18)
#define IDPRAM_MIFCON_DMARXREQEN_1  (1<<19)
#define IDPRAM_MIFCON_FIXBIT        (1<<20)
#define IDPRAM_MIFPCON_ADM_MODE     (1<<6) // mux / demux mode

struct idpram_sfr_reg {
	unsigned int2ap;
	unsigned int2msm;
	unsigned mifcon;
	unsigned mifpcon;
	unsigned msmintclr;
	unsigned dma_tx_adr;
	unsigned dma_rx_adr;
};

/*	S5PV210 Internal Dpram GPIO table 	*/
struct idpram_gpio_data {
	unsigned num;
	unsigned cfg;
	unsigned pud;
	unsigned val;
};

struct idpramctl {
	volatile void __iomem *idpram_base;
	volatile struct idpram_sfr_reg __iomem *idpram_sfr;
	atomic_t read_lock;
	atomic_t write_lock;
	struct wake_lock rd_wlock;
	struct wake_lock hold_wlock; /**/
	struct wake_lock wakeup_wlock; /**/
	struct completion complete_dpramdown;
	struct delayed_work resume_work;
	unsigned last_pm_mailbox; /* 0xCC or 0x CA*/
	unsigned resume_retry;
	unsigned pm_states;
};
struct idpramctl g_idpram;
//EXPORT_SYMBOL_GPL(g_idpram);

#define IDPRAM_ADDRESS_DEMUX 1
static struct idpram_gpio_data idpram_gpio_address[] = {
#ifdef IDPRAM_ADDRESS_DEMUX
	{
		.num = S5PV210_GPJ0(0),	/* MSM_ADDR 0 -12 */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(1),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(2),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(3),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(4),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(5),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(6),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ0(7),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(0),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(1),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(2),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(3),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(4),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ1(5),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	},
#endif
};

static struct idpram_gpio_data idpram_gpio_data[] = {
	{
		.num = S5PV210_GPJ2(0), /* MSM_DATA 0 - 15 */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(1),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(2),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(3),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(4),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(5),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(6),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ2(7),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(0),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(1),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(2),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(3),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(4),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(5),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(6),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ3(7),
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	},
};

static struct idpram_gpio_data idpram_gpio_init_control[] = {
	{
		.num = S5PV210_GPJ4(0), /* MSM_CSn */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ4(1), /* MSM_WEn */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ4(2), /* MSM_Rn */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	}, {
		.num = S5PV210_GPJ4(3), /* MSM_IRQn */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	},
#ifndef IDPRAM_ADDRESS_DEMUX
	{
		.num = S5PV210_GPJ4(4), /* MSM_ADVN */
		.cfg = S3C_GPIO_SFN(0x2),
		.pud = S3C_GPIO_PULL_NONE,
	},
#endif
};


static void idpram_gpio_cfg(struct idpram_gpio_data *gpio)
{
	printk("[iDPRAM] %s idpram set gpio num=%d, cfg=%d, pud=%d, val=%d\n",
		__func__,gpio->num, gpio->cfg, gpio->pud, gpio->val);

	s3c_gpio_cfgpin(gpio->num, gpio->cfg);
	s3c_gpio_setpull(gpio->num, gpio->pud);
	if (gpio->val)
		gpio_set_value(gpio->num, gpio->val);
}

static void idpram_gpio_init(void)
{
	int i;
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);

	for (i = 0; i < ARRAY_SIZE(idpram_gpio_address); i++)
		idpram_gpio_cfg(&idpram_gpio_address[i]);

	for (i = 0; i < ARRAY_SIZE(idpram_gpio_data); i++)
		idpram_gpio_cfg(&idpram_gpio_data[i]);

	for (i = 0; i < ARRAY_SIZE(idpram_gpio_init_control); i++)
		idpram_gpio_cfg(&idpram_gpio_init_control[i]);
}

static void idpram_sfr_init(struct idpramctl *idpram)
{
	volatile struct idpram_sfr_reg __iomem *sfr = idpram->idpram_sfr;
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);

//	sfr->mifcon = (IDPRAM_MIFCON_FIXBIT | IDPRAM_MIFCON_INT2APEN |
//		IDPRAM_MIFCON_INT2MSMEN);

	sfr->mifcon = (IDPRAM_MIFCON_FIXBIT | IDPRAM_MIFCON_INT2APEN);

#ifndef IDPRAM_ADDRESS_DEMUX
	sfr->mifpcon = (IDPRAM_MIFPCON_ADM_MODE);
#endif
}

//#define IDPRAM_INT_CLEAR()	idpram_sfr_int_clear(&g_sfr)
void _idpram_sfr_int_clear(struct idpramctl *idpram)
{
	volatile struct idpram_sfr_reg __iomem *sfr;
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	sfr = idpram->idpram_sfr;
	sfr->msmintclr = 0xFF;
}

void idpram_int_clear(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	return _idpram_sfr_int_clear(&g_idpram);
}

/* HOLD Wake Lock*/
void idpram_wakelock_timeout(unsigned msec)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	printk(KERN_ERR "%s : %d\n", __func__, msec);
	wake_lock_timeout(&g_idpram.wakeup_wlock, msecs_to_jiffies(msec));
}

static void idpram_resume_retry(struct work_struct *work)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	struct idpramctl *idpram =
		container_of(work, struct idpramctl, resume_work.work);

	printk(KERN_INFO "%s\n", __func__);

//	if (!_idpram_resume_check(idpram)) {
//		printk(KERN_INFO "idpram resume ok\n");
//		_idpram_write_lock(idpram, 0);
//		wake_lock_timeout(&idpram->hold_wlock, msecs_to_jiffies(20));
//		return;
//	}
	if (idpram->resume_retry--) {
		schedule_delayed_work(&idpram->resume_work, msecs_to_jiffies(200));
		wake_lock_timeout(&idpram->hold_wlock, msecs_to_jiffies(260));
	}else {
		printk(KERN_INFO "idpram resume T-I-M-E-O-UT\n");
//		idpram_timeout_handler();
		/* hold wakelock until uevnet sent to rild */
		wake_lock_timeout(&idpram->hold_wlock, HZ*7);
//		_idpram_write_lock(idpram, 0);
	}
}


#define IDPRAM_SFR_PHYSICAL_ADDR 0xED008000
#define IDPRAM_SFR_SIZE 0x1C

static int _dpram_ex_init(struct idpramctl *idpram)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	volatile struct idpram_sfr_reg __iomem *sfr;
	struct clk *clk;

	wake_lock_init(&idpram->rd_wlock, WAKE_LOCK_SUSPEND, "dpram_pwrdn");
	wake_lock_init(&idpram->hold_wlock, WAKE_LOCK_SUSPEND, "dpram_hold");
	wake_lock_init(&idpram->wakeup_wlock, WAKE_LOCK_SUSPEND, "dpram_wakeup");
	atomic_set(&idpram->read_lock, 0);
	atomic_set(&idpram->write_lock, 0);
	INIT_DELAYED_WORK(&idpram->resume_work, idpram_resume_retry);

	/* enable internal dpram clock */
	clk = clk_get(NULL, "modem");
	if (!clk) {
		printk(KERN_ERR  "idpram failed to get clock %s\n", __func__);
		return -EFAULT;
	}
	clk_enable(clk);

	/* get sfr io-remap */
	sfr = (volatile struct idpram_sfr_reg __iomem *)
		ioremap_nocache(IDPRAM_SFR_PHYSICAL_ADDR, IDPRAM_SFR_SIZE);
	if (!sfr) {
	        printk(KERN_ERR "idpram_sfr_base io-remap fail\n");
		/*iounmap(idpram_base);*/
		return -EFAULT;
	}
	idpram->idpram_sfr = sfr;

	idpram_sfr_init(idpram);
	/**/dpram_clear();
	printk(KERN_ERR "dpram clear add\n");
	idpram_gpio_init();

	printk("[iDPRAM] %s init done\n", __func__);
	//wake_lock(&idpram->hold_wlock);

	return 0;
}

static int _dpram_ex_deinit(struct idpramctl *idpram)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	wake_lock_destroy(&idpram->rd_wlock);
	wake_lock_destroy(&idpram->hold_wlock);
	wake_lock_destroy(&idpram->wakeup_wlock);
	return 0;
}

int dpram_ex_init(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	return _dpram_ex_init(&g_idpram);
}

int dpram_ex_deinit(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	return _dpram_ex_deinit(&g_idpram);
}


/* tty related functions. */
static inline void byte_align(unsigned long dest, unsigned long src)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	u16 *p_src;
	volatile u16 *p_dest;

	if (!(dest % 2) && !(src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0xFF00) | (*p_src & 0x00FF);
	} else if ((dest % 2) && (src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0x00FF) | (*p_src & 0xFF00);
	} else if (!(dest % 2) && (src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0xFF00) | ((*p_src >> 8) & 0x00FF);
	} else if ((dest % 2) && !(src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0x00FF) | ((*p_src << 8) & 0xFF00);
	} else {
              printk("[iDPRAM] %s %d oops.~\n",__FUNCTION__,__LINE__);
	}
}

static inline void _memcpy(void *p_dest, const void *p_src, int size)
{
//	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	unsigned long dest = (unsigned long)p_dest;
	unsigned long src = (unsigned long)p_src;

	if (size <= 0)
		return;

	if (dest & 1) {
		byte_align(dest, src);
		dest++, src++;
		size--;
	}

	if (size & 1) {
		byte_align(dest + size - 1, src + size - 1);
		size--;
	}

	if (src & 1) {
		unsigned char *s = (unsigned char *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) {
			*d++ = s[0] | (s[1] << 8);
			s += 2;
		}
	} else {
		u16 *s = (u16 *)src;
		volatile u16 *d = (unsigned short *)dest;
		size >>= 1;
		while (size--)
			*d++ = *s++;
	}
}


/* Note the use of non-standard return values (0=match, 1=no-match) */
static inline int _memcmp(u8 *dest, u8 *src, int size)
{
//	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
#if 1
	while (size--)
		if (*dest++ != *src++)
			return 1;
	return 0;
#else
	int i = 0;
	for (i = 0 ; i < size ; i++)
		if (*(dest + i) != *(src + i))
			return 1;
	return 0;
#endif
}




void dpram_platform_init(void)
{

  /* LTE-STEALTH Related config. CS related settings are done in Machine Init*/
  unsigned int regVal;

  /* SINGALS
     1) C110_WTLFOTA_DPRAM_nCS --> XM0CSN_3  ( ie Xm0CSn[3] MP0_1[3])
     2) C110_OE_N -->XM0OEN
     3) C110_LB -> XM0BEN_0
     4) C110_UB --> XM0BEN_1
     5) C110_WTLFOTA_DPRAM_INT_N --> XEINT_8 : how to config this one?
     6) C110_WE_N --> XM0WEN
     7) DATA LINES --> XM0DATA_0 to XM0DATA_15
     8) Address Lines -->XM0ADDR_0 to XM0ADDR_12 */
    
  //ADDR LINES //0xE0200340  and 0xE0200360
  regVal = 0x22222222;
  writel(regVal, S5PV210_GPA0_BASE + 0x0340);
		
  regVal = __raw_readl (S5PV210_GPA0_BASE + 0x0360);
  regVal |= 0x00022222;
  writel(regVal, S5PV210_GPA0_BASE + 0x0360);
	
  //DATA LINES MP06 and MP07 //0xE0200380 and 0xE02003A0
  regVal = 0x22222222;
  writel(regVal, S5PV210_GPA0_BASE + 0x0380);
      
  regVal = 0x22222222;
  writel(regVal,S5PV210_GPA0_BASE + 0x03A0);
}


static inline int READ_FROM_DPRAM_VERIFY(void *dest, u32 src, int size)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	int cnt = 3;
	while (cnt--) {
		_memcpy((void *)dest, (void *)(DPRAM_VBASE + src), size);
		if (!_memcmp((u8 *)dest, (u8 *)(DPRAM_VBASE + src), size))
			return 0;
	}
	return -1;
}

void send_interrupt_to_phone(u16 irq_mask)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	u16 temp;
	READ_FROM_DPRAM(&temp, WTLFOTA_DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE);

	if ((temp & INT_MASK_REQ_ACK_R) && is_net_stopped)
	{
		/* don't let the mailbox overwrite happen if we req for ACK_R */
		printk ("<=== Setting the Interrupt Mask: %d\n", irq_mask);
		irq_mask |= INT_MASK_REQ_ACK_R;
	}

	WRITE_TO_DPRAM(WTLFOTA_DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&irq_mask, WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE);

	printk("[iDPRAM] =====> send IRQ: %x \n", irq_mask);
#ifdef PRINT_SEND_IRQ
	DPRAM_LOG_SEND_IRQ("[iDPRAM] =====> send IRQ: %x \n", irq_mask);
#endif
}


static int dpram_shared_bank_remap(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	dpram_base = ioremap_nocache(WTLFOTA_DPRAM_START_ADDRESS_PHYS +
		WTLFOTA_DPRAM_SHARED_BANK, WTLFOTA_DPRAM_SHARED_BANK_SIZE);
	if (dpram_base == NULL) {
		printk("failed ioremap\n");
		return -ENOENT;
	}

	printk("[DPRAM] ioremap returned %p\n", dpram_base);
	return 0;
}

void dpram_clear(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	long i = 0, err = 0;
	unsigned long flags;

	printk("[iDPRAM] *** entering dpram_clear()\n");
	/* clear DPRAM except interrupt area */
	local_irq_save(flags);

	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < WTLFOTA_DPRAM_SIZE - (WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		*((u16 *)(DPRAM_VBASE + i)) = i;
	}
	local_irq_restore(flags);

	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < WTLFOTA_DPRAM_SIZE - (WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		if (*((u16 *)(DPRAM_VBASE + i)) != i) {
			printk("[iDPRAM] *** dpram_clear() verification failed at %8.8X\n", (((unsigned int)DPRAM_VBASE) + i));
			if (err++ > 128)
				break;
		}
	}

	local_irq_save(flags);
	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < WTLFOTA_DPRAM_SIZE - (WTLFOTA_DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		*((u16 *)(DPRAM_VBASE + i)) = 0;
	}
	local_irq_restore(flags);

	printk("[DPRAM] *** leaving dpram_clear()\n");
}

void dpram_magickey_init(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	const u16 magic_code = 0x00aa;
	u16 acc_code = 0x0001;
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &acc_code, sizeof(acc_code));
}


static void dpram_phone_reset(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	/* hardware guide */
#if 1
	gpio_set_value(GPIO_CP_RST, GPIO_LEVEL_LOW);
	mdelay(600);
	gpio_set_value(GPIO_CP_RST, GPIO_LEVEL_HIGH);
#endif
}

static int dpram_phone_power_on(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	int RetVal = 0;
	int dpram_init_RetVal = 0;

	printk("[iDPRAM] dpram_phone_power_on using GPIO_PHONE_ON()\n");

	gpio_set_value(GPIO_PHONE_ON, GPIO_LEVEL_HIGH);
#if defined(CONFIG_MACH_VIPER)
	if (HWREV < 2)
		gpio_set_value(GPIO_PHONE_ON_REV00, GPIO_LEVEL_HIGH);
#endif
	mdelay(10);
	gpio_set_value(GPIO_CP_RST, GPIO_LEVEL_LOW);
	mdelay(600);
	gpio_set_value(GPIO_CP_RST, GPIO_LEVEL_HIGH);

	/* We have a reset. Use that */

	//dpram_platform_init();

	/* Wait here until the PHONE is up. Waiting as the this called from
	 * IOCTL->UM thread */
	printk("[iDPRAM] power control waiting for INT_MASK_CMD_PIF_INIT_DONE\n");
	/* 1HZ = 1 clock tick, 100 default */
	ClearPendingInterruptFromModem();

	return TRUE;
}

static void dpram_phone_power_off(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	printk("[DPRAM] Phone power Off. - do nothing\n");
}

static int dpram_phone_getstatus(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	return gpio_get_value(GPIO_PHONE_ACTIVE);
}

//Used only in debug mode
static void dpram_mem_rw(struct _mem_param *param)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	if (param->dir)
		WRITE_TO_DPRAM(param->addr, (void *)&param->data, sizeof(param->data));
	else
		READ_FROM_DPRAM((void *)&param->data, param->addr, sizeof(param->data));
}

u16 read_phone_pda_maibox(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	u16 read;
	READ_FROM_DPRAM((void *)&read, WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS,
		sizeof(read));
	return read;
}



static int claim_output_gpio(unsigned gpio, const char *label, int pull){
  if (gpio_is_valid(gpio)) {
    if (gpio_request(gpio, label)){
      printk("[iDPRAM] %s : Failed gpio_request GPIO%u : %s \n", __FUNCTION__, gpio, label);
      return -1;
    }
    if(gpio_direction_output(gpio, pull)){
      printk("[iDPRAM] %s Failed gpio_direction_output GPIO%u : %s \n", __FUNCTION__, gpio, label);
      gpio_free(gpio);
      return -1;
    }else{
      return 0;
    }
  }else{
    printk("[iDPRAM] %s : Error Invalid GPIO%d : %s \n", __FUNCTION__, gpio, label);
    return -1;
  }
}

static int claim_input_gpio(unsigned gpio, const char *label){
  if (gpio_is_valid(gpio)) {
    if (gpio_request(gpio, label)){
      printk("[iDPRAM] %s : Failed gpio_request GPIO%d : %s \n", __FUNCTION__, gpio, label);
      return -1;
    }
    if(gpio_direction_input(gpio)){
      printk("[iDPRAM] %s : Failed gpio_direction_input GPIO%d : %s \n", __FUNCTION__, gpio, label);
      gpio_free(gpio);
      return -1;
    }else{
      return 0;
    }
  }else{
    printk("[iDPRAM] %s : Error Invalid GPIO%d : %s \n", __FUNCTION__, gpio, label);
    return -1;
  }
}

static void release_gpio(unsigned gpio){
  if (gpio_is_valid(gpio)) {
    gpio_free(gpio);
  }else{
    printk("[iDPRAM] %s : Error Invalid GPIO%d\n", __FUNCTION__, gpio);
  }
}


static void init_hw_setting(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	/*
	 * int irq
	 * irq = gpio_to_irq()
	 */

	//	u32 mask;
	/* initial pin settings - dpram driver control */
	printk("[iDPRAM] %s GPIO%d : GPIO_PHONE_ACTIVE\n",__FUNCTION__,GPIO_PHONE_ACTIVE);
	printk("[iDPRAM] %s GPIO%d : IRQ_PHONE_ACTIVE\n",__FUNCTION__,IRQ_PHONE_ACTIVE);
	printk("[iDPRAM] %s GPIO%d : IRQ_DPRAM_INT_N\n",__FUNCTION__,IRQ_WTLFOTA_DPRAM_INT_N);
	printk("[iDPRAM] %s GPIO%d : GPIO_DPRAM_INT_N\n",__FUNCTION__,GPIO_DPRAM_INT_N);
	printk("[iDPRAM] %s GPIO%d : IRQ_HOST_WAKEUP\n",__FUNCTION__,IRQ_HOST_WAKEUP);
	printk("[iDPRAM] %s GPIO%d : GPIO_PHONE_ON\n",__FUNCTION__,GPIO_PHONE_ON);
	printk("[iDPRAM] %s GPIO%d : GPIO_PHONE_ON_REV00\n",__FUNCTION__,GPIO_PHONE_ON_REV00);
	printk("[iDPRAM] %s GPIO%d : GPIO_CP_RST\n",__FUNCTION__,GPIO_CP_RST);
	printk("[iDPRAM] %s GPIO%d : GPIO_PDA_ACTIVE\n",__FUNCTION__,GPIO_PDA_ACTIVE);

	s3c_gpio_cfgpin(GPIO_PHONE_ACTIVE, S3C_GPIO_SFN(GPIO_PHONE_ACTIVE_AF));
	s3c_gpio_setpull(GPIO_PHONE_ACTIVE, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_PHONE_ACTIVE, IRQ_TYPE_EDGE_BOTH);

	set_irq_type(IRQ_WTLFOTA_DPRAM_INT_N, IRQ_TYPE_EDGE_FALLING);

	/* host wakeup gpio */
	s3c_gpio_cfgpin(GPIO_DPRAM_INT_N, S3C_GPIO_SFN(GPIO_DPRAM_INT_N_AF));
	s3c_gpio_setpull(GPIO_DPRAM_INT_N, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_HOST_WAKEUP, IRQ_TYPE_EDGE_RISING);

	if (gpio_is_valid(GPIO_PHONE_ON)) {
		if (gpio_request(GPIO_PHONE_ON, "dpram/GPIO_PHONE_ON"))
			printk(KERN_ERR "[iDPRAM] request fail GPIO_PHONE_ON\n");
		gpio_direction_output(GPIO_PHONE_ON, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_PHONE_ON, S3C_GPIO_PULL_NONE);

#if defined(CONFIG_MACH_VIPER)
	if (HWREV < 2) {
		if (gpio_is_valid(GPIO_PHONE_ON_REV00)) {
			if (gpio_request(GPIO_PHONE_ON_REV00, "dpram/GPIO_PHONE_ON"))
				printk(KERN_ERR "[iDPRAM] request fail GPIO_PHONE_ON\n");
			gpio_direction_output(GPIO_PHONE_ON_REV00, GPIO_LEVEL_HIGH);
		}
		s3c_gpio_setpull(GPIO_PHONE_ON_REV00, S3C_GPIO_PULL_NONE);
	}
#endif
	if (gpio_is_valid(GPIO_CP_RST)) {
		if (gpio_request(GPIO_CP_RST, "dpram/GPIO_CP_RST"))
			printk(KERN_ERR "[iDPRAM] request fail GPIO_CP_RST\n");
		gpio_direction_output(GPIO_CP_RST, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_CP_RST, S3C_GPIO_PULL_NONE);

	gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_HIGH);
}



static void fini_hw_setting(void)
{
  release_gpio(GPIO_PHONE_ACTIVE);
  release_gpio(GPIO_PHONE_ON);
  release_gpio(GPIO_CP_RST);
  release_gpio(GPIO_DPRAM_INT_N);
}



static int  wtlfota_dpram_probe(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	int retval = 0;

	printk("[iDPRAM] *** Entering dpram_probe5()\n");

	/* @LDK@ H/W setting */
      init_hw_setting();

      retval = dpram_shared_bank_remap();
      if (retval != 0) {
        printk("[iDPRAM] %s dpram_shared_bank_remap() failed ErrorCode= %d \n",__FUNCTION__,retval);
        return retval;
      }
	
	retval = dpram_ex_init();
      if (retval != 0) {
        printk("[iDPRAM] %s dpram_ex_init() failed ErrorCode= %d \n",__FUNCTION__,retval);
        return retval;
      }

	//dpram_platform_init();

	printk(KERN_ERR  "[iDPRAM] *** Leaving dpram_probe()\n");
	return 0;


  /* @LDK@ check out missing interrupt from the phone */
  //check_miss_interrupt();

  printk("[iDPRAM] %s initialized \n",__FUNCTION__);

  return retval;
}



/***************************************************************************/
/*                              IOCTL                                      */
/***************************************************************************/

typedef struct gpio_name_value_pair{
  char name[_WTLFOTA_GPIO_PARAM_NAME_LENGTH];
  int value;
}gpio_name_value_pair_t;

#define GPIO_TRANSLATION_LIST_LEN 4
static gpio_name_value_pair_t GPIO_TRANSLATION_LIST[GPIO_TRANSLATION_LIST_LEN]={
// Viper
//  {"GPIO_PHONE_ON", GPIO_PHONE_ON_REV00},
  {"GPIO_PHONE_ON", GPIO_PHONE_ON},
  {"GPIO_PHONE_RST_N", GPIO_CP_RST},
  {"GPIO_DPRAM_INT", GPIO_DPRAM_INT_N},
  {"GPIO_PHONE_ACTIVE", GPIO_PHONE_ACTIVE}, 
};

unsigned int gpio_name_to_value(const char *name){
  unsigned val = -1;
  int i;
  for(i=0; i<GPIO_TRANSLATION_LIST_LEN; i++){
    if(!strncmp(name, GPIO_TRANSLATION_LIST[i].name, _WTLFOTA_GPIO_PARAM_NAME_LENGTH)){
      val = GPIO_TRANSLATION_LIST[i].value;
//      printk("%s : GPIO%d = %s, \n",__FUNCTION__,GPIO_TRANSLATION_LIST[i].value, GPIO_TRANSLATION_LIST[i].name);  // Viper
      break;
    }
  }
  return val;
}

static void dpram_gpio_op(struct _gpio_param *param)
{
  unsigned int gpio;
  gpio = gpio_name_to_value(param->name);
  if(gpio < 0){
    printk("[iDPRAM] %s : gpio name not recognized\n",__FUNCTION__);
    return;
  }
  switch(param->op){
  case _WTLFOTA_GPIO_OP_READ:
    param->data = gpio_get_value(gpio);
    printk("[iDPRAM] %s : _WTLFOTA_GPIO_OP_READ  : %s : GPIO%d=%d \n",__FUNCTION__, param->name, gpio, param->data);   // Viper
    break;
  case _WTLFOTA_GPIO_OP_WRITE:
    gpio_set_value(gpio, param->data);
    printk("[iDPRAM] %s : _WTLFOTA_GPIO_OP_WRITE : %s : GPIO%d=%d \n",__FUNCTION__, param->name, gpio, param->data);    // Viper
    break;
  default:
    break;
  }
}


static int dpramctl_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long l)
{
  int ret;
  unsigned char *arg = (unsigned char *)l;
  switch (cmd) {
  case DPRAM_GPIO_OP:
    {
      struct _gpio_param param;
      ret = copy_from_user((void *)&param, (void *)arg, sizeof(param));
      if(ret != 0){
	printk("[iDPRAM] %s : copy_from_user failed!\n", __FUNCTION__);
	return -EINVAL;
      }
      dpram_gpio_op(&param);
      if (param.op == _WTLFOTA_GPIO_OP_READ){
	return copy_to_user((unsigned long *)arg, &param, sizeof(param));
      }
      return 0;
    }
  default:
    break;
  }
  return -EINVAL;
}

static struct file_operations dpramctl_fops = {
  .owner =	THIS_MODULE,
  .ioctl =	dpramctl_ioctl,
  .llseek =	no_llseek,
};

//use the minor number of dpramctl_dev in dev
static struct miscdevice dpramctl_dev = {
  .minor =	132, //MISC_DYNAMIC_MINOR,
  .name =		IOCTL_DEVICE_NAME,
  .fops =		&dpramctl_fops,
};

/**ioctl ends**/

static struct uio_info uinfo={
  .name = "wtlfota_idpram",
  .version = "0.0.1",
};




/* @LDK@ interrupt handlers. */
static irqreturn_t dpram_irq_handler(int irq, void *dev_id)
{

#if 0
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	u16 irq_mask = 0;
	unsigned long flags;

	READ_FROM_DPRAM_VERIFY(&irq_mask, WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(irq_mask));
	local_irq_save(flags);
	local_irq_disable();

	printk("[iDPRAM] %s received mailboxAB [%X] = 0x%X \n", __func__,WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, irq_mask);

	// valid bit verification. @LDK@
	if (!(irq_mask & INT_MASK_VALID)) {
		printk("[iDPRAM] %s Invalid interrupt mask: 0x%X \n",__func__, irq_mask);
		goto exit;
	}

	// Say something about the phone being dead...
	if (irq_mask == INT_POWERSAFE_FAIL) {
		printk("[iDPRAM] %s *** MODEM image corrupt.  Rerun download. urq_mask = 0x%X ***\n",__func__, irq_mask);
		goto exit;
	}


exit:
#endif
	printk("[iDPRAM] Leaving dpram_irq_handler()\n");
	idpram_int_clear();
#if 0
	local_irq_restore(flags);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t phone_active_irq_handler(int irq, void *dev_id)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	volatile int gpio_state;
	int ret;
	gpio_state = gpio_get_value(GPIO_PHONE_ACTIVE);
	printk("[iDPRAM] %s PHONE_ACTIVE level: %s, phone_sync: %d\n",
			__func__, ((gpio_state) ? "HIGH" : "LOW "), phone_sync);
	return IRQ_HANDLED;
}


static irqreturn_t host_wakeup_irq_handler(int irq, void *dev_id)
{
	printk("[iDPRAM] %s - AP_CP_INT(%d), PDA_ACTIVE(%d)\n", __func__,
		gpio_get_value(GPIO_DPRAM_INT_N),
		gpio_get_value(GPIO_PDA_ACTIVE));
	idpram_wakelock_timeout(5000);
	return IRQ_HANDLED;
}


static int uio_register(struct uio_info * info ){
  int retval = 0;
  printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);

  info->mem[0].addr = WTLFOTA_DPRAM_START_ADDRESS_PHYS + WTLFOTA_DPRAM_SHARED_BANK;
  info->mem[0].size = WTLFOTA_DPRAM_SHARED_BANK_SIZE;
  info->mem[0].memtype = UIO_MEM_PHYS;

  info->irq = IRQ_WTLFOTA_DPRAM_INT_N;
  info->irq_flags=IRQF_DISABLED;
  info->handler = dpram_irq_handler;
  //todo: i should use global encapsulation here. just for debugging
  retval = dpram_uio_register_device(dpramctl_dev.this_device, info);
  if(retval !=0)
  {
      printk("[iDPRAM] %s: dpram_uio_register_device FAILED for info->GPIOirq:%ld \n",__FUNCTION__, info->irq);
      return retval;
  }
  else
      printk("[iDPRAM] %s: dpram_uio_register_device PASSED for info->GPIOirq:%ld \n",__FUNCTION__, info->irq);


   retval = request_irq(IRQ_PHONE_ACTIVE, phone_active_irq_handler, IRQF_DISABLED, "VIAActive", NULL);
   if(retval !=0)
  {
      printk("[iDPRAM] %s: dpram_uio_register_device FAILED for info->GPIOirq \n",__FUNCTION__);
      free_irq(IRQ_WTLFOTA_DPRAM_INT_N, NULL);
      return retval;
  }
  else
  {
      printk("[iDPRAM] %s: dpram_uio_register_device PASSED for info->GPIOirq \n",__FUNCTION__);
      enable_irq_wake(IRQ_PHONE_ACTIVE);
  }


   retval = request_irq(IRQ_HOST_WAKEUP, host_wakeup_irq_handler, IRQF_DISABLED, "Host Wakeup", NULL);
   if(retval !=0)
   {
      printk("[iDPRAM] %s: dpram_uio_register_device FAILED for info->GPIOirq \n",__FUNCTION__);
      free_irq(IRQ_WTLFOTA_DPRAM_INT_N, NULL);
      free_irq(IRQ_PHONE_ACTIVE, NULL);
      return retval;
    }
  else
  {
      printk("[iDPRAM] %s: dpram_uio_register_device PASSED for info->GPIOirq \n",__FUNCTION__);
      enable_irq_wake(IRQ_HOST_WAKEUP);
  }

   return retval;
   
}


static void uio_unregister(struct uio_info * info ){
  dpram_uio_unregister_device(info);
}


static void check_miss_interrupt(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	unsigned long flags;
	/*TODO: intr reg instead of gpio -- idpram */
	if (gpio_get_value(GPIO_PHONE_ACTIVE) &&
			(!gpio_get_value(GPIO_DPRAM_INT_N))) {
		printk("[iDPRAM] %s there is a missed interrupt. try to read it!\n",__func__);

		local_irq_save(flags);
		dpram_irq_handler(IRQ_WTLFOTA_DPRAM_INT_N, NULL);
		local_irq_restore(flags);
	}
}


void ClearPendingInterruptFromModem(void)
{
	printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
	u16 in_interrupt = 0;
	READ_FROM_DPRAM((void *)&in_interrupt, WTLFOTA_DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
}


/* init & cleanup. */
static int __init wtlfota_dpram_init(void)
{

  printk("[iDPRAM] %s Started \n",__FUNCTION__,__LINE__);
  int ret;
  ret =  wtlfota_dpram_probe();
  if(ret != 0){
    printk("[iDPRAM] %s : wtlfota_dpram_probe fail!\n",__FUNCTION__);
    return -1;
  }
  ret = misc_register(&dpramctl_dev);
  if (ret < 0) {
    printk("[iDPRAM] %s : misc_register() failed\n",__FUNCTION__);
    return -1;
  }
  
  ret = uio_register(&uinfo);
  if (ret != 0) {
    printk("[iDPRAM] %s : uio_register() failed\n",__FUNCTION__);
    return -1;
  }

	
  printk("[iDPRAM]  %s Completed : %d  \n", __FUNCTION__, ret);
  return ret;
}


static void __exit wtlfota_dpram_exit(void)
{
  printk("[iDPRAM] %s %d \n",__FUNCTION__,__LINE__);
  uio_unregister(&uinfo);
  misc_deregister(&dpramctl_dev);
  fini_hw_setting();
  printk("[iDPRAM] %s : SUCCESS \n",__FUNCTION__);
}


module_init(wtlfota_dpram_init);
module_exit(wtlfota_dpram_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");

MODULE_DESCRIPTION("WTLFOTA_IDPRAM Device Driver.");

MODULE_LICENSE("GPL");
