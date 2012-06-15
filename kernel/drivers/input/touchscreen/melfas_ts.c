/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

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
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/melfas_ts.h>
#include <linux/miscdevice.h>
#include "mcs8000_download.h"

#define MELFAS_MAX_TOUCH       5
#define FW_VERSION             0x01

#define TS_MAX_X_COORD         320
#define TS_MAX_Y_COORD         480
#define TS_MAX_Z_TOUCH         255
#define TS_MAX_W_TOUCH         30

#define TS_READ_START_ADDR 	   0x10
#define TS_READ_VERSION_ADDR   0x63
#define TS_READ_REGS_LEN       30

#define I2C_RETRY_CNT			10

#define	SET_DOWNLOAD_BY_GPIO	0

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define DEBUG_PRINT 			0

#define IRQ_TOUCH_INT  (IRQ_EINT_GROUP3_BASE+1)

#define DEVICE_NAME "touchkey"

#ifdef CONFIG_MACH_VIPER
#define TSP_TEST_MODE
#endif

#if SET_DOWNLOAD_BY_GPIO
#include <melfas_download.h>
#endif // SET_DOWNLOAD_BY_GPIO

struct muti_touch_info
{
    int state;
    int strength;
    int width;
    int posX;
    int posY;
};

struct melfas_ts_data
{
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct  work;
    uint32_t flags;
    int (*power)(int on);
    struct early_suspend early_suspend;
    int hw_rev;
    int fw_ver;
};

static struct miscdevice touchkey_update_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
};

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int melfas_ts_resume(struct i2c_client *client);

#ifdef TSP_TEST_MODE
static uint16_t tsp_test_temp[TS_MELFAS_SENSING_CHANNEL_NUM];
static uint16_t tsp_test_reference[TS_MELFAS_EXCITING_CHANNEL_NUM][TS_MELFAS_SENSING_CHANNEL_NUM];
static uint16_t tsp_test_inspection[TS_MELFAS_EXCITING_CHANNEL_NUM][TS_MELFAS_SENSING_CHANNEL_NUM];
static bool sleep_state = false;
uint8_t refer_y_channel_num = 1;
uint8_t inspec_y_channel_num = 1;
uint8_t refer_test_cnt = 0;
uint8_t inspec_test_cnt = 0;

int touch_screen_ctrl_testmode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num);
static int ts_melfas_test_mode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num);

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

static uint16_t melfas_ts_inspection_spec_table[TS_MELFAS_SENSING_CHANNEL_NUM*TS_MELFAS_EXCITING_CHANNEL_NUM*2] =
{
	453, 	841, 	508, 	943, 	510, 	946, 	506, 	940, 	508, 	943, 	507, 	941, 	506, 	940, 	508, 	944, 	505, 	939, 	442, 	822, 
	363, 	673, 	383, 	711, 	389, 	723, 	383, 	711, 	389, 	722, 	389, 	722, 	382, 	710, 	386, 	718, 	378, 	702, 	357, 	663, 
	335, 	623, 	348, 	646, 	361, 	670, 	347, 	645, 	359, 	667, 	359, 	667, 	347, 	644, 	356, 	660, 	343, 	637, 	328, 	610, 
	312, 	580, 	316, 	588, 	335, 	621, 	314, 	584, 	334, 	620, 	334, 	620, 	314, 	582, 	331, 	615, 	311, 	577, 	306, 	568, 
	295, 	549, 	293, 	543, 	316, 	588, 	290, 	538, 	316, 	586, 	315, 	585, 	289, 	537, 	313, 	581, 	286, 	532, 	288, 	536, 
	284, 	527, 	272, 	506, 	304, 	564, 	270, 	501, 	303, 	563, 	302, 	562, 	270, 	501, 	301, 	559, 	267, 	497, 	278, 	516, 
	274, 	510, 	257, 	477, 	293, 	545, 	253, 	469, 	292, 	542, 	291, 	541, 	253, 	469, 	291, 	540, 	252, 	468, 	269, 	499, 
	266, 	494, 	243, 	451, 	286, 	532, 	242, 	449, 	286, 	532, 	286, 	532, 	243, 	451, 	288, 	534, 	246, 	456, 	273, 	507, 
	272, 	506, 	235, 	437, 	281, 	523, 	234, 	434, 	283, 	525, 	282, 	524, 	235, 	436, 	284, 	527, 	237, 	441, 	266, 	494, 
	258, 	480, 	230, 	426, 	278, 	516, 	228, 	423, 	279, 	519, 	279, 	519, 	229, 	425, 	280, 	520, 	232, 	432, 	273, 	507, 
	257, 	477, 	226, 	420, 	276, 	512, 	223, 	415, 	278, 	516, 	278, 	516, 	224, 	416, 	279, 	517, 	228, 	424, 	261, 	485, 
	256, 	475, 	223, 	415, 	275, 	511, 	221, 	411, 	277, 	515, 	277, 	515, 	223, 	413, 	279, 	517, 	226, 	420, 	260, 	484, 
	255, 	473, 	222, 	412, 	274, 	510, 	219, 	407, 	277, 	514, 	277, 	514, 	221, 	410, 	277, 	515, 	225, 	417, 	260, 	484, 
	246, 	458, 	238, 	442, 	264, 	490, 	239, 	443, 	265, 	491, 	266, 	494, 	237, 	439, 	267, 	497, 	241, 	447, 	257, 	477 
};

touch_screen_t touch_screen =
{
    {0},
    1,
    {0}
};

touch_screen_driver_t melfas_test_mode =
{
    {
        TS_MELFAS_VENDOR_NAME,
        TS_MELFAS_VENDOR_CHIP_NAME,
        TS_MELFAS_VENDOR_ID,
        0,
        0,
        5,
        TS_MELFAS_SENSING_CHANNEL_NUM, TS_MELFAS_EXCITING_CHANNEL_NUM,
        2,
        2900, 3500,0,0,
        melfas_ts_inspection_spec_table
    },
    0,
    ts_melfas_test_mode
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

static struct melfas_ts_data *ts;

static int melfas_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}
	
	return 0;
}

static int melfas_i2c_write(struct i2c_client* p_client, u8* data, int len)
{
	struct i2c_msg msg;

	msg.addr = p_client->addr;
	msg.flags = 0; /* I2C_M_WR */
	msg.len = len;
	msg.buf = data ;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int melfas_int_low_level_checking(void)
{
    int retry_n;

    /* wait IRQ low */
    for (retry_n = 500 ; retry_n ; retry_n--)
    {
        if (gpio_get_value(GPIO_TOUCH_INT) == 0)
        {
            printk(KERN_DEBUG "TSP_INT low...OK\n") ;
            break ; /* reference data were prepared */
        }
        msleep(1) ;
    }

    if (gpio_get_value(GPIO_TOUCH_INT))
    {
        printk(KERN_ERR "Error INT HIGH!!!\n") ;
        return 0;
    }
    return 1;
}

static int melfas_init_panel(struct melfas_ts_data *ts)
{
    int ret, buf = 0x00;
    ret = i2c_master_send(ts->client, &buf, 1);
    ret = i2c_master_send(ts->client, &buf, 1);

    if (ret < 0)
    {
        printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed\n [%d]", ret);
        return 0;
    }

    return true;
}

static void melfas_ts_work_func(struct work_struct *work)
{
    struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
    int ret = 0, i;
    uint8_t buf[TS_READ_REGS_LEN];
    int touchNumber = 0, touchPosition = 0, posX = 0, posY = 0, width = 0, strength = 0;
    int keyEvent = 0, keyState = 0, keyID = 0, keystrength = 0;

#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_work_func\n");

    if (ts == NULL)
        printk(KERN_ERR "melfas_ts_work_func : TS NULL\n");
#endif


    /**
    Simple send transaction:
    	S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
    Simple recv transaction:
    	S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
    */
    buf[0] = TS_READ_START_ADDR;
    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = i2c_master_send(ts->client, buf, 1);
        if (ret >= 0)
        {
            ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);
            if (ret >= 0)
                break; // i2c success
        }
    }

    if (ret < 0)
    {
        printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
        enable_irq(ts->client->irq);
        return ;
    }
    else // Five Multi Touch Interface
    {
        touchNumber = buf[0] & 0x0F;
        touchPosition = buf[1] & 0x1F;

        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
        {
            g_Mtouch_info[i].posX = ((buf[2 + 5*i] >> 4)   << 8) + buf[3 + 5*i];
            g_Mtouch_info[i].posY = ((buf[2 + 5*i] & 0x0F) << 8) + buf[4 + 5*i];
            g_Mtouch_info[i].width = buf[5 + 5*i];
            g_Mtouch_info[i].strength = buf[6 + 5*i];

            if (g_Mtouch_info[i].width != 0)
                g_Mtouch_info[i].state = 1;
            else
                g_Mtouch_info[i].state = 0;
        }

        keyID = buf[5*MELFAS_MAX_TOUCH + 2] & 0x07;
        keyState = (buf[5*MELFAS_MAX_TOUCH + 2] >> 3) & 0x01;
        keyEvent = (buf[5*MELFAS_MAX_TOUCH + 2] >> 4) & 0x01;
        keystrength = (buf[5*MELFAS_MAX_TOUCH + 3]);

	if (keyEvent)
	{
#ifdef CONFIG_SEC_KEY_DBG
		printk("[T_KEY]ID=%d,state=%d\n", keyID, keyState);
#else
		printk("[T_KEY]%d\n", keyState);
#endif
		if (keyID == 0x1)
			input_report_key(ts->input_dev, 139, keyState ? PRESS_KEY : RELEASE_KEY);
		else if (keyID == 0x2)
			input_report_key(ts->input_dev, 102, keyState ? PRESS_KEY : RELEASE_KEY);
		else if (keyID == 0x3)
			input_report_key(ts->input_dev, 158, keyState ? PRESS_KEY : RELEASE_KEY);
		else if (keyID == 0x4)
			input_report_key(ts->input_dev, 217, keyState ? PRESS_KEY : RELEASE_KEY);
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func: keyID : %d, keyState: %d\n", keyID, keyState);
#endif
	}
	else{
	        if (touchNumber > MELFAS_MAX_TOUCH)
	        {
#if DEBUG_PRINT
	            printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d\n",  touchID);
#endif
	            enable_irq(ts->client->irq);
	            return;
	        }

	        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
	        {
	            if (g_Mtouch_info[i].posX == 0)
	                continue;

	            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
	            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
	            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
	            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
	            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
	            input_mt_sync(ts->input_dev);

#ifdef CONFIG_SEC_KEY_DBG
	            printk("x=%d,y=%d,z=%d,id=%d\n", g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, i);
#else
	            printk("z=%d\n", g_Mtouch_info[i].strength);
#endif

	        }
	}       

        input_sync(ts->input_dev);
    }

    enable_irq(ts->client->irq);
}

#ifdef TSP_TEST_MODE
void touch_screen_sleep()
{
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

void touch_screen_wakeup()
{
	gpio_set_value(GPIO_TSP_LDO_ON, 0);
	msleep(5);		
	//melfas_ts_resume(ts->client);
	gpio_set_value(GPIO_TSP_LDO_ON, 1);
	msleep(70); 
	enable_irq(ts->client->irq);
}

int touch_screen_get_tsp_info(touch_screen_info_t *tsp_info)
{
    int ret = 0;

    /* chipset independent */
    tsp_info->driver = touch_screen.tsp_info.driver;
    tsp_info->reference.bad_point = touch_screen.tsp_info.reference.bad_point;
    tsp_info->reference.table = touch_screen.tsp_info.reference.table;

    /* chipset dependent */
    /* melfas */
    tsp_info->inspection.bad_point = touch_screen.tsp_info.inspection.bad_point;
    tsp_info->inspection.table = touch_screen.tsp_info.inspection.table;
    return ret;
}

//****************************************************************************
//
// Function Name:   touch_screen_ctrl_testmode
//
// Description:
//
// Notes:
//
//****************************************************************************
int touch_screen_ctrl_testmode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num)
{
    int ret = 0;
    bool prev_device_state = FALSE;
    touch_screen.driver = &melfas_test_mode;
    touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);

    if (touch_screen.device_state == FALSE)
    {
        touch_screen_wakeup();
        touch_screen.device_state = TRUE;
        msleep(100);
        prev_device_state = TRUE;
    }

    switch (cmd)
    {
    case TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_LOW:
    {
       touch_screen.tsp_info.driver->reference_spec_low = test_info->reference;
        break;
    }

    case TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_HIGH:
    {
       touch_screen.tsp_info.driver->reference_spec_high = test_info->reference;
        break;
    }

    case TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_LOW:
    {
        touch_screen.tsp_info.driver->inspection_spec_low = test_info->reference;
        break;
    }

    case TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_HIGH:
    {
        touch_screen.tsp_info.driver->inspection_spec_high = test_info->reference;
        break;
    }

    case TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST:
    {
        break;
    }
    
    default:
    {
        if (test_info != NULL)
        {
            printk(KERN_DEBUG "DEFAULT\n");
            ret = touch_screen.driver->test_mode(cmd, test_info, test_info_num);
        }
        else
        {
            ret = -1;
        }
        break;
    }
    }

    if (prev_device_state == TRUE)
    {
        touch_screen_sleep();
    }

    return ret;
}

//****************************************************************************
//
// Function Name:   ts_melfas_test_mode
//
// Description:
//
// Notes:
//
//****************************************************************************
static int ts_melfas_test_mode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num)
{
    int i, j, ret = 0;
    uint8_t buf[TS_MELFAS_SENSING_CHANNEL_NUM*2];

    switch (cmd)
    {
    case TOUCH_SCREEN_TESTMODE_ENTER:
    {
        break;
    }

    case TOUCH_SCREEN_TESTMODE_EXIT:
    {
        break;
    }

    case TOUCH_SCREEN_TESTMODE_GET_OP_MODE:
    {
        break;
    }

    case TOUCH_SCREEN_TESTMODE_GET_THRESHOLD:
    {
        ret = melfas_i2c_read(ts->client, TS_MELFAS_TESTMODE_TSP_THRESHOLD_REG, buf, 1);
        test_info->threshold = buf[0];
        break;
    }

    case TOUCH_SCREEN_TESTMODE_GET_DELTA:
    {
        break;
    }

    case TOUCH_SCREEN_TESTMODE_GET_REFERENCE:
    {
        melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;
        printk(KERN_DEBUG "REFERENCE\n");
        if (melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE)
        {
            for (j = 0; j < TS_MELFAS_EXCITING_CHANNEL_NUM; j++)
        {
            for (i = 0; i < TS_MELFAS_SENSING_CHANNEL_NUM; i++)
            {
                    buf[0] = TS_MELFAS_TESTMODE_DATA_CTRL_REG;
                    buf[1] = 0x44;
                    buf[2] = j;		// exciting ch
                    buf[3] = i;		// sensing ch
                    ret = melfas_i2c_write(ts->client, buf, 4);

                if( 0 == melfas_int_low_level_checking())
                    goto error_ret ;

                    ret |= melfas_i2c_read(ts->client, TS_MELFAS_TESTMODE_DATA_READ_REG, buf, 2);
                    //printk("INSPECTION RAW DATA : [%2d,%2d][%02x%02x]\n",buf[2],buf[3],buf[1],buf[0]);
                    memcpy(&(test_info[i+(j*TS_MELFAS_SENSING_CHANNEL_NUM)].inspection), buf, 2);
                }
            }
        }
        else
        {
            ret = -3;
        }
        break;
    }

    case TOUCH_SCREEN_TESTMODE_GET_INSPECTION:
    {
        melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;
        printk(KERN_DEBUG "INSPECTION\n");
        if (melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE)
        {
            for (j = 0; j < TS_MELFAS_EXCITING_CHANNEL_NUM; j++)
            {
                for (i = 0; i < TS_MELFAS_SENSING_CHANNEL_NUM; i++)
                {
                    buf[0] = TS_MELFAS_TESTMODE_DATA_CTRL_REG;
                    buf[1] = 0x42;
                    buf[2] = j;		// exciting ch
                    buf[3] = i;		// sensing ch
                    ret = melfas_i2c_write(ts->client, buf, 4);

                    if( 0 == melfas_int_low_level_checking())
                        goto error_ret ;

                    ret |= melfas_i2c_read(ts->client, TS_MELFAS_TESTMODE_DATA_READ_REG, buf, 2);
                    //printk("INSPECTION RAW DATA : [%2d,%2d][%02x%02x]\n",buf[2],buf[3],buf[1],buf[0]);
                    memcpy(&(test_info[i+(j*TS_MELFAS_SENSING_CHANNEL_NUM)].inspection), buf, 2);
                }
            }
        }
        else
        {
            ret = -3;
        }
        break;
    }

    default:
    {
        ret = -2;
        break;
    }
    }

    mcsdl_vdd_off();
    mdelay(200);
    mcsdl_vdd_on();
    printk("[TOUCH] reset.\n");
    mdelay(700);
    /* enable TSP_IRQ */
	enable_irq(ts->client->irq);
    return ret;

error_ret :
    ret = -3;
    return ret;
}

static ssize_t tsp_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Manufacturer : Melfas\nChip name : MMS100\n");
}

static ssize_t tsp_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int i ;
    for (i = 0 ; i < TS_MELFAS_SENSING_CHANNEL_NUM ; i++)
    {
        printk("%d,", tsp_test_temp[i]);
    }
    return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
                   tsp_test_temp[0], tsp_test_temp[1], tsp_test_temp[2], tsp_test_temp[3],
                   tsp_test_temp[4], tsp_test_temp[5], tsp_test_temp[6], tsp_test_temp[7],
                   tsp_test_temp[8], tsp_test_temp[9]);
}

static ssize_t tsp_test_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
    touch_screen.driver = &melfas_test_mode;
    touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);
    if (strncmp(buf, "self", 4) == 0)
    {
        /* disable TSP_IRQ */
        printk(KERN_DEBUG "START %s\n", __func__) ;
        disable_irq(ts->client->irq);
        touch_screen_info_t tsp_info = {0};
        touch_screen_get_tsp_info(&tsp_info);

        uint16_t reference_table_size;
        touch_screen_testmode_info_t* reference_table = NULL;
        reference_table_size = tsp_info.driver->x_channel_num * tsp_info.driver->y_channel_num;
        reference_table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * reference_table_size, GFP_KERNEL);
        touch_screen_ctrl_testmode(TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST, reference_table, reference_table_size);

        mcsdl_vdd_off();
        mdelay(500);
        mcsdl_vdd_on();
        printk("[TOUCH] reset.\n");
        mdelay(200);

        /* enable TSP_IRQ */
        enable_irq(ts->client->irq);
    }
    else
    {
        printk("TSP Error Unknwon commad!!!\n");
    }

    return size ;
}

static ssize_t tsp_test_reference_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "Reference START %s , %d\n", __func__ , refer_y_channel_num) ;

	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d \n", 
	tsp_test_reference[refer_y_channel_num][0],tsp_test_reference[refer_y_channel_num][1],tsp_test_reference[refer_y_channel_num][2],tsp_test_reference[refer_y_channel_num][3],
	tsp_test_reference[refer_y_channel_num][4],tsp_test_reference[refer_y_channel_num][5],tsp_test_reference[refer_y_channel_num][6],tsp_test_reference[refer_y_channel_num][7],
	tsp_test_reference[refer_y_channel_num][8],tsp_test_reference[refer_y_channel_num][9]);
}

static ssize_t tsp_test_reference_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{	
	unsigned int position;
	int ret;
	
	int j,i;
	uint16_t ref_value;
	uint8_t buf1[4],buff[20];
	
	sscanf(buf,"%d\n",&position);

	if(position == 100){
		refer_test_cnt = 0;
		printk("reset_reference_value\n");
	}

	if(!refer_test_cnt){
		/* disable TSP_IRQ */
		disable_irq(ts->client->irq);
		for(i=0;i<14;i++)
		{
			for(j=0;j<10;j++)
			{
				buf1[0] = 0xA0 ;		/* register address */			
				buf1[1] = 0x44 ;
				buf1[2] = i;				
				buf1[3] = j;			

				if (melfas_i2c_write(ts->client, buf1, 4) != 0)			
				{				
					printk(KERN_ERR "Failed to enter testmode\n") ; 
				}		

				while(1)
				{
					if (gpio_get_value(GPIO_TOUCH_INT) == 0)
						break;
				}

				if( melfas_i2c_read(ts->client, 0xAE, buff, 1)!= 0)
				{
					printk(KERN_ERR "Failed to read(referece data)\n") ;
				}

				if( melfas_i2c_read(ts->client, 0xAF, buff, 2)!= 0)
				{
					printk(KERN_ERR "Failed to read(referece data)\n") ;
				}

				ref_value = (uint16_t)(buff[1] << 8) | buff[0] ;
			 	tsp_test_reference[i][j] = ref_value;		
				//printk("ref value[%d]=%d\n",i,ref_value);
			}	
		}
		mcsdl_vdd_off();
		mdelay(50);
		mcsdl_vdd_on();
		mdelay(250);
		printk("[TOUCH] reset.\n"); 
		/* enable TSP_IRQ */
		enable_irq(ts->client->irq);
		refer_test_cnt =1;
	}
	else if((position < 0 || position > 14)) {
		printk(KERN_DEBUG "Invalid values\n");
		return -EINVAL; 	   
	}
	refer_y_channel_num = (uint8_t)position;
	

	return size;
}

static ssize_t tsp_test_inspection_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "Reference START %d\n", __func__ , inspec_y_channel_num) ;
	
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d \n", 
		tsp_test_inspection[inspec_y_channel_num][0],tsp_test_inspection[inspec_y_channel_num][1],tsp_test_inspection[inspec_y_channel_num][2],tsp_test_inspection[inspec_y_channel_num][3],
		tsp_test_inspection[inspec_y_channel_num][4],tsp_test_inspection[inspec_y_channel_num][5],tsp_test_inspection[inspec_y_channel_num][6],tsp_test_inspection[inspec_y_channel_num][7],
		tsp_test_inspection[inspec_y_channel_num][8],tsp_test_inspection[inspec_y_channel_num][9]);
}

static ssize_t tsp_test_inspection_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	unsigned int position;
	int j,i,ret;
	uint16_t ref_value;
	uint8_t buf1[2],buff[20];
	
	sscanf(buf,"%d\n",&position);

	if(position == 100){
		inspec_test_cnt = 0;
		printk("reset_inspection_value\n");
	}

	if(!inspec_test_cnt){

		/* disable TSP_IRQ */
		disable_irq(ts->client->irq);
		for(i=0;i<14;i++)
		{
			for(j=0;j<10;j++)
			{
				buf1[0] = 0xA0 ;		/* register address */			
				buf1[1] = 0x42 ;
				buf1[2] = i;				
				buf1[3] = j;			

				if (melfas_i2c_write(ts->client, buf1, 4) != 0) 		
				{				
					printk(KERN_ERR "Failed to enter testmode\n") ; 
				}		

				while(1)
				{
					if (gpio_get_value(GPIO_TOUCH_INT) == 0)
						break;
				}

				if( melfas_i2c_read(ts->client, 0xAE, buff, 1)!= 0)
				{
					printk(KERN_ERR "Failed to read(referece data)\n") ;
				}

				if( melfas_i2c_read(ts->client, 0xAF, buff, 2)!= 0)
				{
					printk(KERN_ERR "Failed to read(referece data)\n") ;
				}
				
				printk("ref value0=%x\n",buff[0]);
				printk("ref value1=%x\n",buff[1]);

				ref_value = (uint16_t)(buff[1] << 8) | buff[0] ;
				tsp_test_inspection[i][j] = ref_value;		
				printk("ins value[%d]=%d\n",i,ref_value);
				inspec_test_cnt =1;
			}	
		}
		mcsdl_vdd_off();
		mdelay(50);
		mcsdl_vdd_on();
		mdelay(250);
		printk("[TOUCH] reset.\n"); 
		/* enable TSP_IRQ */
		enable_irq(ts->client->irq);
	}
	
	if (position < 0 || position > 14) {
		printk(KERN_DEBUG "Invalid values\n");
		return -EINVAL; 	   
	}

	inspec_y_channel_num = (uint8_t)position;
	
	return size;
}


static ssize_t tsp_test_sleep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (sleep_state)
        sprintf(buf, "sleep\n");
    else
        sprintf(buf, "wakeup\n");

    return sprintf(buf, "%s", buf);
}

static ssize_t tsp_test_sleep_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
    if (strncmp(buf, "sleep", 5) == 0)
    {
        touch_screen_sleep();
        sleep_state = true;
    }
    else
    {
        printk( "TSP Error Unknwon commad!!!\n");
    }

    return size ;
}

static ssize_t tsp_test_wakeup_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
    if (strncmp(buf, "wakeup", 6) == 0)
    {
        touch_screen_wakeup();
        sleep_state = false;
    }
    else
    {
        printk( "TSP Error Unknwon commad!!!\n");
    }

    return size ;
}

#endif

extern void keypad_led_control(bool onOff);
static ssize_t keypad_brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;

    sscanf(buf, "%d", &value);
    printk("keypad_brightness_store %d \n", value);

    if (value)
        keypad_led_control(1);
    else
        keypad_led_control(0);

    printk(KERN_DEBUG "[%s] brightness : %d \n", __FUNCTION__, value);

    return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
        mcsdl_vdd_on();
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
        mcsdl_vdd_off();
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
        mcsdl_vdd_off();
		mdelay(500);
        mcsdl_vdd_on();
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 buf1[2] = {0,};
	u8 buf2[2] = {0,};

	int status=0, mode_ctl=0, hw_rev=0, fw_ver=0;

	if (0 == melfas_i2c_read(ts->client, MCSTS_STATUS_REG, buf1, 2))
	{
		status = buf1[0];
		mode_ctl = buf1[1];	 
	}
	else
	{
		printk("%s : Can't find status, mode_ctl!\n", __func__); 
	}

	if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf2, 2))
	{
		hw_rev = buf2[0];
		fw_ver = buf2[1];	 
	}
	else
	{
		printk("%s : Can't find HW Ver, FW ver!\n", __func__); 
	}
	
	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver;


	if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
	{
		hw_rev = buf1[0];
		fw_ver = buf1[1];	 
		sprintf(buf,"HW Ver : 0x%02x, FW Ver : 0x%02x\n", hw_rev, fw_ver);
	}
	else
	{	 
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}

return sprintf(buf, "%s", buf); 
}


static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;

	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {

		printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_ts->hw_rev);
		printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_ts->fw_ver);

		if((melfas_ts->fw_ver != 0x24 && melfas_ts->hw_rev == 0x40)
		  ||(melfas_ts->fw_ver != 0x25 && melfas_ts->hw_rev == 0x50))
        { 
    		disable_irq(ts->client->irq);

    		printk("[F/W D/L] Entry gpio_tlmm_config\n");
    		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
    		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    		
    		printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
    		ret = mms100_ISC_download_binary_file(melfas_ts->hw_rev);
    		
    		enable_irq(ts->client->irq);
    		
            if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf, 2))
            {
                ts->hw_rev = buf[0];
                ts->fw_ver = buf[1];
                printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
            }
            else
            {
                ts->hw_rev = 0;
                ts->fw_ver = 0;
                printk("%s : Can't find HW Ver, FW ver!\n", __func__);
            }
    			
    		if(ret > 0){
    				if (melfas_ts->hw_rev < 0) {
    					printk(KERN_ERR "i2c_transfer failed\n");;
    				}
    				
    				if (melfas_ts->fw_ver < 0) {
    					printk(KERN_ERR "i2c_transfer failed\n");
    				}
    				
    				printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_ts->hw_rev, melfas_ts->fw_ver);

    		}
    		else {
                printk("[TOUCH] Firmware update failed.. RESET!\n");
                mcsdl_vdd_off();
                mdelay(500);
                mcsdl_vdd_on();
                mdelay(200);
    		}
        }
    }
#endif

	return size;
}

/* Touch Reference ************************************************************/
static ssize_t reference_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   v, exciting_ch, sensing_ch;
    unsigned char   rx_ch, tx_ch;
    unsigned char   buff[100];
    int             written_bytes = 0;  /* & error check */
    int             value ;
    int             retry_n ;

    printk(KERN_DEBUG "%s\n", __func__) ;
    
    /* sensing channle */
    if (melfas_i2c_read(ts->client, 0x2F, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(sensing_ch)\n") ;
        goto normal_ret ;
    }
    sensing_ch = v & 0x1F ;

    /* exciting channel */
    if (melfas_i2c_read(ts->client, 0x2E, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(exciting_ch)\n") ;
        goto normal_ret ;
    }
    exciting_ch = v & 0x1F ;
    printk(KERN_DEBUG "sensing_ch = %d, exciting_ch = %d\n", sensing_ch, exciting_ch) ;

    /* disable TSP_IRQ */
    disable_irq(ts->client->irq);

    /* enter test mode 0x02 */
    buff[0] = 0xA1 ;        /* register address */
    buff[1] = 0x02 ;
    if (melfas_i2c_write(ts->client, buff, 2) != 0)
        goto normal_ret ;

    /* read reference data */
    for (rx_ch = 0 ; rx_ch < sensing_ch ; rx_ch++)
    {
        /* wait IRQ low */
        for (retry_n = 20 ; retry_n ; retry_n--)
        {
            if (gpio_get_value(GPIO_TOUCH_INT) == 0)
            {
                printk(KERN_DEBUG "TSP_INT low...OK\n") ;
                break ; /* reference data were prepared */
            }
            msleep(1) ;
        }

        if (gpio_get_value(GPIO_TOUCH_INT))
        {
            printk(KERN_ERR "Error INT HIGH!!!\n") ;
            written_bytes = 0 ;
            goto normal_ret ;
        }

        if (melfas_i2c_read(ts->client, 0xB2, buff, exciting_ch*2) != 0)
        {
            printk(KERN_ERR "Failed to read the reference data.\n") ;
            written_bytes = 0 ;
            goto normal_ret ;
        }

        printk(KERN_DEBUG "SENSING_CH=%d ", rx_ch) ;
        for (exciting_ch = 0 ; exciting_ch < exciting_ch ; exciting_ch++)
        {
            value = (buff[2*exciting_ch] << 8) | buff[2*exciting_ch+1] ;
            written_bytes += sprintf(buf+written_bytes, " %d", value) ;
        }
    }

normal_ret :
    mcsdl_vdd_off();
    mdelay(200);
    mcsdl_vdd_on();
    printk("[TOUCH] reset.\n");
    mdelay(700);
    /* enable TSP_IRQ */
	enable_irq(ts->client->irq);

	printk(KERN_DEBUG "written = %d\n", written_bytes) ;
	if (written_bytes > 0)
		return written_bytes ;
    return sprintf(buf, "-1") ; // TODO:RECHECK with Platform App
}

static ssize_t raw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   v, exciting_ch, sensing_ch;
    unsigned char   rx_ch, tx_ch;
    unsigned char   buff[100];
    int             written_bytes = 0;  /* & error check */
    int             value ;
    int             retry_n ;

    printk(KERN_DEBUG "%s\n", __func__) ;

    /* sensing channle */
    if (melfas_i2c_read(ts->client, 0x2F, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(sensing_ch)\n") ;
        goto normal_ret ;
    }
    sensing_ch = v & 0x1F ;

    /* exciting channel */
    if (melfas_i2c_read(ts->client, 0x2E, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(exciting_ch)\n") ;
        goto normal_ret ;
    }
    exciting_ch = v & 0x1F ;
    printk(KERN_DEBUG "sensing_ch = %d, exciting_ch = %d\n", sensing_ch, exciting_ch) ;

    /* disable TSP_IRQ */
    disable_irq(ts->client->irq);

    /* enter test mode 0x02 */
    buff[0] = 0xA1 ;        /* register address */
    buff[1] = 0x01 ;
    if (melfas_i2c_write(ts->client, buff, 2) != 0)
        goto normal_ret ;

    for (rx_ch = 0 ; rx_ch < sensing_ch ; rx_ch++)
    {
        /* wait IRQ low */
        for (retry_n = 20 ; retry_n ; retry_n--)
        {
            if (gpio_get_value(GPIO_TOUCH_INT) == 0)
            {
                printk(KERN_DEBUG "TSP_INT low...OK\n") ;
                break ; /* reference data were prepared */
            }
            msleep(1) ;
        }

        if (gpio_get_value(GPIO_TOUCH_INT))
        {
            printk(KERN_ERR "Error INT HIGH!!!\n") ;
            written_bytes = 0 ;
            goto normal_ret ;
        }

        if (melfas_i2c_read(ts->client, 0xB2, buff, exciting_ch*2) != 0)
        {
            printk(KERN_ERR "Failed to read the reference data.\n") ;
            written_bytes = 0 ;
            goto normal_ret ;
        }

        printk(KERN_DEBUG "SENSING_CH=%d ", rx_ch) ;
        for (exciting_ch = 0 ; exciting_ch < exciting_ch ; exciting_ch++)
        {
            value = (buff[2*exciting_ch] << 8) | buff[2*exciting_ch+1] ;
            written_bytes += sprintf(buf+written_bytes, " %d", value) ;
        }
    }
	
normal_ret :
    mcsdl_vdd_off();
    mdelay(200);
    mcsdl_vdd_on();
    printk("[TOUCH] reset.\n");
    mdelay(700);
    /* enable TSP_IRQ */
	enable_irq(ts->client->irq);

	printk(KERN_DEBUG "written = %d\n", written_bytes) ;
	if (written_bytes > 0)
		return written_bytes ;
	return sprintf(buf, "-1") ; // TODO:RECHECK with Platform App
}

/* Touch Reference ************************************************************/
static ssize_t tsp_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   v,exciting_ch, sensing_ch ;

    printk(KERN_DEBUG "%s\n", __func__) ;
    /* sensing channle */
    if (melfas_i2c_read(ts->client, 0x2F, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(sensing_ch)\n") ;
    }
    sensing_ch = v & 0x1F ;

    /* exciting channel */
    if (melfas_i2c_read(ts->client, 0x2E, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(exciting_ch)\n") ;
    }
    exciting_ch = v & 0x1F ;
    printk(KERN_DEBUG "sensing_ch = %d, exciting_ch = %d\n", sensing_ch, exciting_ch) ;
	return sprintf(buf, "%d,%d\n",sensing_ch,exciting_ch);

}

static DEVICE_ATTR(brightness, 0664, NULL, keypad_brightness_store);
static DEVICE_ATTR(gpio, 0664, gpio_show, gpio_store);
static DEVICE_ATTR(registers, 0664, registers_show, registers_store);
static DEVICE_ATTR(firmware, 0664, firmware_show, firmware_store);
static DEVICE_ATTR(reference, 0664, reference_show, NULL) ;

#ifdef TSP_TEST_MODE
static DEVICE_ATTR(tsp_name, 0664, tsp_name_show, NULL);
static DEVICE_ATTR(tsp_test, 0664, tsp_test_show, tsp_test_store);
static DEVICE_ATTR(tsp_reference, 0664, tsp_test_reference_show, tsp_test_reference_store);
static DEVICE_ATTR(tsp_inspection, 0664, tsp_test_inspection_show, tsp_test_inspection_store);
static DEVICE_ATTR(tsp_sleep, 0664, tsp_test_sleep_show, tsp_test_sleep_store);
static DEVICE_ATTR(tsp_wakeup, 0664, NULL, tsp_test_wakeup_store);
static DEVICE_ATTR(tsp_channel, 0664, tsp_channel_show, NULL);
#endif

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
    struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;

    disable_irq_nosync(ts->client->irq);
    schedule_work(&ts->work);

    return IRQ_HANDLED;
}

void melfas_upgrade(INT32 hw_ver)
{
    int ret;
    unsigned char buf[2];

    printk("[F/W D/L] Entry gpio_tlmm_config\n");
    printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
    ret = mcsdl_download_binary_data(ts->hw_rev);

    if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf, 2))
    {
        ts->hw_rev = buf[0];
        ts->fw_ver = buf[1];
        printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
    }
    else
    {
        ts->hw_rev = 0;
        ts->fw_ver = 0;
        printk("%s : Can't find HW Ver, FW ver!\n", __func__);
    }

    if (ret > 0)
    {
        if ((ts->hw_rev < 0) || (ts->fw_ver < 0))
            printk(KERN_ERR "i2c_transfer failed\n");
        else
            printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", ts->hw_rev, ts->fw_ver);
    }
    else
    {
        printk("[TOUCH] Firmware update failed.. RESET!\n");
        mcsdl_vdd_off();
        mdelay(500);
        mcsdl_vdd_on();
        mdelay(200);
    }
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0, i;

    uint8_t buf[2];

    printk(KERN_DEBUG"+-----------------------------------------+\n");
    printk(KERN_DEBUG "|  Melfas Touch Driver Probe!            |\n");
    printk(KERN_DEBUG"+-----------------------------------------+\n");

    gpio_set_value(GPIO_TSP_LDO_ON, 1);
    msleep(70);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    INIT_WORK(&ts->work, melfas_ts_work_func);

    ts->client = client;
    i2c_set_clientdata(client, ts);
    ret = i2c_master_send(ts->client, &buf, 1);


#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif

    if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf, 2))
    {
        ts->hw_rev = buf[0];
        ts->fw_ver = buf[1];
        printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
    }
    else
    {
        ts->hw_rev = 0;
        ts->fw_ver = 0;
        printk("%s : Can't find HW Ver, FW ver!\n", __func__);
    }

    printk("[TOUCH] Melfas	H/W version: 0x%02x.\n", ts->hw_rev);
    printk("[TOUCH] Current F/W version: 0x%02x.\n", ts->fw_ver);

    if (ts->fw_ver < 0x15)
    {
        melfas_upgrade(ts->hw_rev);
        msleep(1);
        gpio_set_value(GPIO_TSP_LDO_ON, 0);
        msleep(1000);
        gpio_set_value(GPIO_TSP_LDO_ON, 1);
        msleep(70);
    };

    ts->input_dev = input_allocate_device();
    if (!ts->input_dev)
    {
        printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }

    ts->input_dev->name = "sec_touchscreen" ;

    ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);


    ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
    ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
    ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);
    ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);


    //	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
    //	__set_bit(EV_ABS,  ts->input_dev->evbit);
    //	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);
    //	__set_bit(EV_SYN, ts->input_dev->evbit);
    //	__set_bit(EV_KEY, ts->input_dev->evbit);


    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        printk(KERN_ERR "melfas_ts_probe: Failed to register device\n");
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }

    ts->client->irq = IRQ_TOUCH_INT;

    if (ts->client->irq)
    {
#if DEBUG_PRINT
        printk(KERN_ERR "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif
        ret = request_irq(ts->client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
        if (ret > 0)
        {
            printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

    schedule_work(&ts->work);

    ret = misc_register(&touchkey_update_device);
    if (ret)
    {
        printk("%s misc_register fail\n", __FUNCTION__);
        goto err_misc_reg;
    }

    if (device_create_file(touchkey_update_device.this_device, &dev_attr_brightness) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_gpio) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_registers) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_firmware) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_reference) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_name) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_test) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_reference) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_inspection) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_sleep) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_wakeup) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }
    if (device_create_file(touchkey_update_device.this_device, &dev_attr_tsp_channel) < 0)
    {
        printk("%s device_create_file fail dev_attr_brightness\n", __FUNCTION__);
        pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }

#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_probe: succeed to register input device\n");
#endif

#if CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = melfas_ts_early_suspend;
    ts->early_suspend.resume = melfas_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#if DEBUG_PRINT
    printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif
    return 0;

err_request_irq:
    printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
    free_irq(client->irq, ts);
err_input_register_device_failed:
    printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
    printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");
err_detect_failed:
    printk(KERN_ERR "melfas-ts: err_detect failed\n");
    kfree(ts);
err_check_functionality_failed:
    printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");
err_misc_reg:

    return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    unregister_early_suspend(&ts->early_suspend);
    free_irq(client->irq, ts);
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
    int i;
    for (i = 0; i < MELFAS_MAX_TOUCH; i++)
    {
        if (g_Mtouch_info[i].posX == 0)
            continue;

        g_Mtouch_info[i].strength = 0;
        g_Mtouch_info[i].width = 0;

        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
        input_mt_sync(ts->input_dev);

        g_Mtouch_info[i].posX = 0;
        g_Mtouch_info[i].posY = 0;
    }
    input_sync(ts->input_dev);
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    release_all_fingers(ts);

    disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret) /* if work was pending disable-count is now 2 */
        enable_irq(client->irq);

    ret = i2c_smbus_write_byte_data(client, 0x01, 0x00); /* deep sleep */

    if (ret < 0)
        printk(KERN_ERR "melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");

    return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    melfas_init_panel(ts);
    enable_irq(client->irq); // scl wave

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
    printk("%s\n", __FUNCTION__);
    keypad_led_control(0);
    release_all_fingers(ts);
    disable_irq(ts->client->irq);
    gpio_set_value(GPIO_TSP_LDO_ON, 0);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
    printk("%s\n", __FUNCTION__);
    gpio_set_value(GPIO_TSP_LDO_ON, 1);
    msleep(70);
    enable_irq(ts->client->irq);
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
    { MELFAS_TS_NAME, 0 },
    { }
};

static struct i2c_driver melfas_ts_driver =
{
    .driver = {
        .name = MELFAS_TS_NAME,
    },
    .id_table = melfas_ts_id,
                .probe = melfas_ts_probe,
                         .remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
                                   .suspend = melfas_ts_suspend,
                                              .resume = melfas_ts_resume,
#endif
                                                    };

static int __devinit melfas_ts_init(void)
{
    return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
    i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
