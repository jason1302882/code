/* 
 * drivers/input/touchscreen/gslX680.c
 *
 * Sileadinc gslX680 TouchScreen driver. 
 *
 * Copyright (c) 2012  Sileadinc
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
 * VERSION      	DATE			AUTHOR
 *   1.0		 2012-04-18		   leweihua
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>  
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>
//#include <linux/wakelock.h>

//#define IC_COMPATIBLE
#ifdef IC_COMPATIBLE
#include "gsl1688f_3670_t8273_pg_compatible.h"

static u8 gsl_cfg_index=0;
struct fw_config_type
{
   const struct fw_data *fw;
   unsigned int fw_size;
   unsigned int *data_id;
   unsigned int data_size;
};
static struct fw_config_type gsl_cfg_table[9]={
	/*1*/
	{GSL1688_FW,(sizeof(GSL1688_FW)/sizeof(struct fw_data)),gsl_config_data_id_1688,(sizeof(gsl_config_data_id_1688)/4)},
	/*2*/
	{GSL3670_FW,(sizeof(GSL3670_FW)/sizeof(struct fw_data)),gsl_config_data_id_3670,(sizeof(gsl_config_data_id_3670)/4)},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
	{NULL,0,NULL,0},
};
#endif


#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#endif
#define I2C_BOARD_INFO_METHOD   1
#ifdef INCAR_S1022A
#define MAX_FINGERS         10
#else
#define MAX_FINGERS         5
#endif
#define MAX_CONTACTS        10
#define DMA_TRANS_LEN       0x20
#define GSL_PAGE_REG        0xf0

#ifdef INCAR_P960E
#define TOUCH_VIRTUAL_KEYS 1   
#endif
//static struct wake_lock ps_lock;

#ifdef INCAR_GSL_GESTURE
#define REPORT_KEY_VALUE KEY_F1      //report key set
#define READ_DATA_LEN	8
static void gsl_irq_mode_change(struct i2c_client *client,u32 flag);
typedef enum{
	GE_DISABLE = 0,
	GE_ENABLE = 1,
	GE_WAKEUP = 2,
	GE_NOWORK =3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static unsigned int gsl_gesture_flag = 1;
static struct input_dev *gsl_power_idev;
static char gsl_gesture_c = 0;
static int power_key_status = 0;
spinlock_t resume_lock; // add 20141030
//static struct  wake_lock gsl_wake_lock;
static char gesture_data = 0;
static char gesture_key = 0;

static void gslx680_hw_reset(void);

extern void gsl_GestureExternInt(unsigned int *model,int len);
extern int irq_set_irq_type(unsigned int irq, unsigned int type);
#endif

#define PRESS_MAX           255
#define GSLX680_NAME		"gslX680"//"synaptics_i2c_rmi"//"synaptics-rmi-ts"// 
#define GSLX680_TS_DEVICE	"gslX680"
#define GSLX680_TS_NAME	    "gslX680"
#define GSLX680_TS_ADDR	    0x40

//#define GSL_DEBUG
#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)   //
#endif

static struct mutex gsl_i2c_lock;
static volatile int gsl_sw_flag = 0;
static volatile int gsl_halt_flag = 0;
#ifdef GSL_TIMER
#undef TPD_PROC_DEBUG
#define GSL_TIMER_CHECK_CIRCLE        200
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static u32 gsl_timer_data = 0;
//static u8 gsl_data_1st[4] = {0};
static volatile int gsl_timer_flag = 0;  // 0:first test  1:second test  2:doing gsl_load_fw
#endif

#ifdef GSL_COMPATIBLE_CHIP
static int gsl_compatible_flag = 0;
#endif

#ifdef INCAR_GSL_GESTURE
#define TPD_PROC_DEBUG 
#endif

#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>  //lzk
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

//#define HAVE_TOUCH_KEY

//#if defined(INCAR_TP_PSENSOR)
#define USE_TP_PSENSOR
//#endif
#ifdef USE_TP_PSENSOR
//#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "tp_psensor.h"
#define PROXIMITY_INPUT_DEV		"alps_pxy"

//#define PS_DEBUG
#ifdef PS_DEBUG
#define PS_DBG(format, ...)	\
		printk("TP_PSENSOR " format "\n", ## __VA_ARGS__)
#else
#define PS_DBG(format, ...)
#endif 

static tp_ps_t *tp_ps = 0;
static int ps_en = 0;
static int ps_supend=0;
static struct class *firmware_class;
static struct device *firmware_cmd_dev;
//static struct wake_lock prx_wake_lock_ps; 
#endif

struct sprd_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

//extern int sprd_3rdparty_gpio_tp_pwr;
//extern int sprd_3rdparty_gpio_tp_rst;
//extern int sprd_3rdparty_gpio_tp_irq;
#if defined(INCAR_SP7731E)
#define GSL_GPIO_RST   63
#define GSL_GPIO_IRQ   64
#else
#define GSL_GPIO_RST	145
#define GSL_GPIO_IRQ	144
#endif
static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;
static struct i2c_client *this_client=NULL;
//static struct sprd_i2c_setup_data gslX680_ts_setup={0, GSLX680_TS_ADDR, 0, GSLX680_TS_NAME};


#ifdef HAVE_TOUCH_KEY
static u16 key = 0;
static int key_state_flag = 0;
struct key_data {
	u16 key;
	u16 x_min;
	u16 x_max;
	u16 y_min;
	u16 y_max;	
};

const u16 key_array[]={
                      KEY_BACK,
                      KEY_HOMEPAGE,
                      KEY_MENU,
                      //KEY_SEARCH,
                     }; 
#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))

#if defined(TOUCH_MODUEL_GSL1686_HOME)
struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 2048, 2048, 2048, 2048},//192             100    1000
	{KEY_HOMEPAGE, 300, 360, 1000, 1100},//			330   1058
	{KEY_MENU, 2048, 2048, 2048, 2048},//576            300    1000
	//{KEY_SEARCH, 2048, 2048, 2048, 2048},
};
#else
struct key_data gsl_key_data[MAX_KEY_NUM] = {
	{KEY_BACK, 2048, 2048, 2048, 2048},//192             100    1000
	{KEY_HOMEPAGE, 0, 100, 1100, 1500},//			330   1058
	{KEY_MENU, 2048, 2048, 2048, 2048},//576            300    1000
	//{KEY_SEARCH, 2048, 2048, 2048, 2048},
};
#endif
#endif

struct gslX680_ts_data {
	struct input_dev	*input_dev;
	u8 touch_data[44];	
	struct i2c_client	*client;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_suspend;
#else
	struct work_struct resume_work;
#endif
	struct gslx680_ts_platform_data	*platform_data;
};

struct gslX680_ts_data *g_gslx680_ts;

#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf,
	 __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":300:1500:60:30"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":100:1500:60:30"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":500:1500:60:30"
    "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.gslX680",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;	
	
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}

#endif

static inline u16 join_bytes(u8 a, u8 b)
{
	u16 ab = 0;
	ab = ab | a;
	ab = ab << 8 | b;
	return ab;
}

static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	
	tmp_buf[0] = addr;
	bytelen++;
	
	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}
	
	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}

	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		printk("%s set data address fail!\n", __func__);
		return ret;
	}
	
	return i2c_master_recv(client, pdata, datalen);
}

#ifdef INCAR_GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	u8 buf[4], read_len;
	int i = 0;
	print_info("=======%s addr = %x, len = %d\n",__func__, addr, len);

#if 1
			buf[0] = ((addr+i*4)/0x80)&0xff;
			buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
			buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
			buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
			gsl_ts_write(this_client, 0xf0, buf, 4);
			gsl_ts_read(this_client, (((addr+i*4)%0x80+8)&0x5f), (char *)&data[i], 4);
			gsl_ts_read(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
			
	for(i=0;i<len;i++)
	{
			gsl_ts_read(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
			//print_info("data[%d] = 0x%08x\n", i, data[i]);
	}

#else
	for(i=0;i<len;i++){
		buf[0] = ((addr+i*4)/0x80)&0xff;
		buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
		buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
		buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
		gsl_ts_write(this_client, 0xf0, buf, 4);
		gsl_ts_read(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
		print_info("data[%d] = 0x%08x\n", i, data[i]);
	}
#endif

	return len;
}
#endif

static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{

	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		if(temp==0x7c){
			temp =0x78;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
			temp = 0x7c;
		}else{
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
		}
	}
	i2c_master_send(client,&temp,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&gsl_i2c_lock);
	return err;
}

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
	return err;	
//	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}
#if 1
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[DMA_TRANS_LEN*4] = {0};
	u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	print_info("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == GSL_DOWNLOAD_DATA[source_line].offset)
		{
			memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);	
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;

			memcpy((buf+send_flag*4 -4),&GSL_DOWNLOAD_DATA[source_line].val,4);	

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    		gsl_write_interface(client, addr, buf, DMA_TRANS_LEN * 4);
				send_flag = 0;
			}

			send_flag++;
		}
	}

	print_info("=============gsl_load_fw end==============\n");

}
#else 
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	print_info("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
	    	addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
	    	gsl_write_interface(client, addr, buf, 4);	
	}
}
#endif
static void gsl_start_core(struct i2c_client *client)
{
	//u8 tmp = 0x00;
	u8 buf[4] = {0};
#if 0
	buf[0]=0;
	buf[1]=0x10;
	buf[2]=0xfe;
	buf[3]=0x1;
	gsl_write_interface(client,0xf0,buf,4);
	buf[0]=0xf;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	gsl_write_interface(client,0x4,buf,4);
	msleep(20);
#endif
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	{
	#ifdef IC_COMPATIBLE
		gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
	#else 
	    gsl_DataInit(gsl_config_data_id);
	#endif
	}
#endif	
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	buf[0] = 0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	msleep(5);
}

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	msleep(5);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	//clear reg
}

static void gsl_sw_init(struct i2c_client *client)
{
	int temp;
	if(1==gsl_sw_flag)
		return;
	gsl_sw_flag = 1;
	gpio_set_value(GSL_GPIO_RST, 0);
	msleep(20);
	gpio_set_value(GSL_GPIO_RST, 1);
	msleep(20);	

	gsl_clear_reg(client);
	gsl_reset_core(client);
#ifdef IC_COMPATIBLE
	gsl_load_fw(client,gsl_cfg_table[gsl_cfg_index].fw,gsl_cfg_table[gsl_cfg_index].fw_size);
#else 
    temp = ARRAY_SIZE(GSL1680E_FW);
	gsl_load_fw(client,GSL1680E_FW,temp);
#endif
	gsl_start_core(client);
	gsl_sw_flag = 0;
}

static void check_mem_data(struct i2c_client *client)
{
	u8 read_buf[4]  = {0};
	msleep(30);
	gsl_read_interface(client,0xb0,read_buf,4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || 
		read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		print_info("!!!!!!!!!!!page: %x offset: %x val: %x %x %x %x\n",
			0x0, 0x0, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		gsl_sw_init(client);
	}
}

#ifdef GSL_COMPATIBLE_CHIP
static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4]={0};
	int err,i;
	for(i=0;i<3;i++)
	{
		err = gsl_read_interface(client,0xfc,buf,4);
		if(!(err < 0)){
			err = 1;
			break;
		}
	}
	return err;
}
#endif

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{	
	u8 buf[4] = {0};
	u32 tmp;
	int i,flag=0;
	static int timer_count;
	if(gsl_halt_flag == 1){
		return;
	}
	//buf[0] = 0x9f;
	//gsl_write_interface(ddata->client, GSL_PAGE_REG, buf, 4);
	gsl_read_interface(this_client, 0xb4, buf, 4);
	tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);

	print_info("[pre] 0xb4 = %x \n",gsl_timer_data);
	print_info("[cur] 0xb4 = %x \n",tmp);
	print_info("gsl_timer_flag=%d\n",gsl_timer_flag);
	if(0 == gsl_timer_flag)
	{
		if(tmp==gsl_timer_data)
		{
			gsl_timer_flag = 1;
			if(0==gsl_halt_flag)
			{
				queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 25);
			}
		}
		else
		{
			for(i=0;i<5;i++){
				gsl_read_interface(this_client,0xbc,buf,4);
				if(buf[0]==0&&buf[1]==0&&buf[2]==0&&buf[3]==0)
				{
					flag = 1;
					break;
				}
				flag =0;
			}
			if(flag == 0){
				gsl_reset_core(this_client);
				gsl_start_core(this_client);
			}
			gsl_timer_flag = 0;
			timer_count = 0;
			if(0 == gsl_halt_flag)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
	}
	else if(1==gsl_timer_flag){
		if(tmp==gsl_timer_data)
		{
			if(0==gsl_halt_flag)
			{
				timer_count++;
				gsl_timer_flag = 2;
				gsl_sw_init(this_client);
				gsl_timer_flag = 1;
			}
			if(0 == gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		else{
			timer_count = 0;
			if(0 == gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		gsl_timer_flag = 0;
	}
	gsl_timer_data = tmp;
}
#endif


#ifdef TPD_PROC_DEBUG
#ifdef GSL_APPLICATION
static int gsl_read_MorePage(struct i2c_client *client,u32 addr,u8 *buf,u32 num)
{
	int i;
	u8 tmp_buf[4] = {0};
	u8 tmp_addr;
	for(i=0;i<num/8;i++){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_ts_write(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),8);
	}
	if(i*8<num){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_ts_write(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_read_interface(client,tmp_addr,(buf+i*8),4);
	}
}
#endif
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
		/*	tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<gsl_cfg_table[gsl_cfg_index].data_size)
				seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]); */

			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			//ptr +=sprintf(ptr,"gsl_config_data_id[%d] = ",tmp);
			//if(tmp>=0&&tmp<512)
			//	ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]); 
#endif
		}
		else 
		{
			gsl_ts_write(this_client,0xf0,&gsl_data_proc[4],4);
			if(gsl_data_proc[0] < 0x80)
				gsl_ts_read(this_client,gsl_data_proc[0],temp_data,4);			
			gsl_read_interface(this_client,gsl_data_proc[0],temp_data,4);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
#ifdef GSL_APPLICATION
	else if('a'==gsl_read[0]&&'p'==gsl_read[1]){
		char *buf;
		int temp1;
		tmp = (unsigned int)(((gsl_data_proc[2]<<8)|gsl_data_proc[1])&0xffff);
		buf=kzalloc(tmp,GFP_KERNEL);
		if(buf==NULL)
			return -1;
		if(3==gsl_data_proc[0]){
			gsl_read_interface(this_client,gsl_data_proc[3],buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}else if(4==gsl_data_proc[0]){
			temp1=((gsl_data_proc[6]<<24)|(gsl_data_proc[5]<<16)|
				(gsl_data_proc[4]<<8)|gsl_data_proc[3]);
			gsl_read_MorePage(this_client,temp1,buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}
		kfree(buf);
	}
#endif
	return 0;
}
static ssize_t gsl_config_write_proc(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%d]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
#ifdef GSL_APPLICATION
	if('a'!=temp_buf[0]||'p'!=temp_buf[1]){
#endif
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
#ifdef GSL_APPLICATION
	}
#endif
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	#ifdef GSL_MONITOR
		cancel_delayed_work_sync(&gsl_monitor_work);
	#endif
		gsl_proc_flag = 1;
		gsl_reset_core(this_client);
		/*msleep(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
		msleep(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
		msleep(20);
		*/
		//gsl_start_core(ddata->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(this_client);
		gsl_start_core(this_client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_ts_write(this_client,buf[4],buf,4);
	}
	
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<512)
		{ 
		 #ifdef IC_COMPATIBLE
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		 #else
		    gsl_config_data_id[tmp1] = tmp;
		 #endif
		}
	}
#endif
#ifdef GSL_APPLICATION
	else if('a'==temp_buf[0]&&'p'==temp_buf[1]){
		if(1==path_buf[3]){
			tmp=((path_buf[5]<<8)|path_buf[4]);
			gsl_ts_write(this_client,path_buf[6],&path_buf[10],tmp);
		}else if(2==path_buf[3]){
			tmp = ((path_buf[5]<<8)|path_buf[4]);
			tmp1=((path_buf[9]<<24)|(path_buf[8]<<16)|(path_buf[7]<<8)
				|path_buf[6]);
			buf[0]=(char)((tmp1/0x80)&0xff);
			buf[1]=(char)(((tmp1/0x80)>>8)&0xff);
			buf[2]=(char)(((tmp1/0x80)>>16)&0xff);
			buf[3]=(char)(((tmp1/0x80)>>24)&0xff);
			buf[4]=(char)(tmp1%0x80);
			gsl_ts_write(this_client,0xf0,buf,4);
			gsl_ts_write(this_client,buf[4],&path_buf[10],tmp);
		}else if(3==path_buf[3]||4==path_buf[3]){
			memcpy(gsl_read,temp_buf,4);
			memcpy(gsl_data_proc,&path_buf[3],7);
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,
	.owner = THIS_MODULE,
};
#endif
#ifdef  USE_TP_PSENSOR
/*customer implement: do something like read data from TP IC*/
static int tp_ps_getdata(char *data)
{
	unsigned char read_buf[4];
	gsl_ts_read(this_client, 0xac, read_buf, sizeof(read_buf));
	*data = !(read_buf[0]);
	PS_DBG("read_buf[0]=%d\n\n",read_buf[0]);

	return 0;
}

static int tp_ps_enable(void)
{
	u8 buf[4]={0};
	
	PS_DBG("%s\n", __func__);

	gpio_set_value(GSL_GPIO_RST, 1);
	mdelay(20);
	//wake_lock(&prx_wake_lock_ps);
	
	buf[3] = 0x00;
	buf[2] = 0x00;
	buf[1] = 0x00;
	buf[0] = 0x04;
	gsl_ts_write(this_client, 0xf0, buf, 4);
	buf[3] = 0x00;
	buf[2] = 0x00;
	buf[1] = 0x00;
	
	buf[0] = 0x02;  
	gsl_ts_write(this_client, 0x00, buf, 4);
	
	ps_en = 1;
	return 0;
}

static int tp_ps_disable(void)
{
	u8 buf[4]={0};
	
	PS_DBG("%s\n", __func__);

	//wake_unlock(&prx_wake_lock_ps);
	buf[3] = 0x00;
	buf[2] = 0x00;
	buf[1] = 0x00;
	buf[0] = 0x04;
	gsl_ts_write(this_client, 0xf0, buf, 4);
	buf[3] = 0x00; 
	buf[2] = 0x00; 
	buf[1] = 0x00; 
	buf[0] = 0x00; 
	gsl_ts_write(this_client, 0x00, buf, 4);
		
	ps_en = 0;
	return 0;
}

static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("tp get prox ver\n");
	if (buf != NULL)
		sprintf(buf, "tp prox version\n");
    return 0;
}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	unsigned int on_off = simple_strtoul(buf, NULL, 10);

//	printk("gsl cjx:qiao_store_proximity_sensor buf=%d,size=%d,on_off=%d\n", *buf, size, on_off);
	if(buf != NULL && size != 0){
		if (0 == on_off)//fear
			tp_ps_disable();
		else if (1 == on_off) //near
			tp_ps_enable();
	}

    return size;
}

static DEVICE_ATTR(proximity, S_IRUGO | S_IWUSR, show_proximity_sensor, store_proximity_sensor);


/* call this function in tp work func*/
static int tp_ps_report_dps(unsigned int touches)
{
	unsigned char dps_data = 0;
	
	tp_ps_getdata(&dps_data);

	PS_DBG("%s: proximity=%d", __func__, dps_data);
	
	input_report_abs(tp_ps->input, ABS_DISTANCE, dps_data);
	input_sync(tp_ps->input);

	return dps_data;
}

static int tp_ps_init(struct i2c_client *client)
{
	int err = 0;
	struct input_dev *input_dev;

	ps_en = 0;
	
	tp_ps = kzalloc(sizeof(tp_ps_t), GFP_KERNEL);
	if (!tp_ps)
	{
		PS_DBG("%s: request memory failed\n", __func__);
		err= -ENOMEM;
		goto exit_mem_fail;
	}
		
	//register device
	firmware_class = class_create(THIS_MODULE,"sprd-tpd");//client->name  
//firmware_class = class_create(THIS_MODULE,"xr-pls");//client->name   wqj


	if(IS_ERR(firmware_class))
		printk("Failed to create class(firmware)!\n");
	firmware_cmd_dev = device_create(firmware_class, NULL, 0, NULL, "device");//device

	if(IS_ERR(firmware_cmd_dev))
		printk("Failed to create device(firmware_cmd_dev)!\n");

	if(device_create_file(firmware_cmd_dev, &dev_attr_proximity) < 0) // /sys/class/sprd-tpd/device/proximity
	{
		printk("Failed to create device file(%s)!\n", dev_attr_proximity.attr.name);
	}

	//wake_lock_init(&prx_wake_lock_ps, WAKE_LOCK_SUSPEND, "prx_wake_lock");
	
	// register input device 
	input_dev = input_allocate_device();
	if (!input_dev) 
	{
		PS_DBG("%s: input allocate device failed\n", __func__);
		err = -ENOMEM;
		goto exit_input_register_failed;
	}

	tp_ps->input = input_dev;

	input_dev->name = PROXIMITY_INPUT_DEV;
	input_dev->phys  = PROXIMITY_INPUT_DEV;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);	
	//for proximity
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(input_dev);
	if (err < 0)
	{
	    PS_DBG("%s: input device regist failed\n", __func__);
	    goto exit_input_register_failed;
	}

	PS_DBG("%s: Probe Success!\n",__func__);
	return 0;

exit_input_register_failed:
	input_free_device(input_dev);
	//wake_lock_destroy(&prx_wake_lock_ps);
	kfree(tp_ps);
exit_mem_fail:
	return err;
}

static int tp_ps_uninit(void)
{
	//free input
	input_unregister_device(tp_ps->input);
	input_free_device(tp_ps->input);
	//free alloc
	kfree(tp_ps);
	tp_ps = 0;
	return 0;
}

#endif
/*
static void gsl_report_point(struct gslX680_ts_data *ts, u16 x, u16 y, u8 pressure, u8 id)
{
	//printk("********id = %d, x = %d, y = %d \n",id, y, x);
//	input_report_abs(ts->input_dev, BTN_TOUCH, 1);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, pressure);
	#if 1
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, y);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, x);	
	#else
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 600-x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);	
	#endif
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 5);
	input_mt_sync(ts->input_dev);
}

static void gsl_report_work(struct work_struct *work)
{
	int rc,tmp;
	//u8 read_buf[4] = {0};
	u8 buf[44] = {0};
	int tmp1=0;
	static u8 gsl_up_flag = 0;
	struct gsl_touch_info cinfo={0};
	struct gslX680_ts_data *ts = i2c_get_clientdata(this_client);
	
#ifdef GSL_TIMER 
	if(2==gsl_timer_flag){
		goto schedule;
	}
#endif

#ifdef TPD_PROC_DEBUG
	if(1==gsl_proc_flag){
		goto schedule;
	}
#endif
//	print_info("gsl_report_work \n");

	 read data from DATA_REG 
	//rc = gsl_read_interface(this_client, 0x80, buf, 44);
	rc = gsl_ts_read(this_client, 0x80, buf, 44);

	if (rc < 0) 
	{
		dev_err(&this_client->dev, "read failed\n");
		goto schedule;
	}

	if (buf[0] == 0xff) {
		goto schedule;
	}

	cinfo.finger_num = buf[0];
	for(tmp=0;tmp<(cinfo.finger_num>10 ? 10:cinfo.finger_num);tmp++)
	{
		cinfo.y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo.x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo.id[tmp] = buf[tmp*4+7] >> 4;
		print_info("tp-gsl  x = %d y = %d \n",cinfo.x[tmp],cinfo.y[tmp]);
	}
	//printk("111 finger_num= %d\n",cinfo.finger_num);
#ifdef GSL_ALG_ID
	cinfo.finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(&cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
	//	gsl_write_interface(this_client,0xf0,buf,4);
		gsl_ts_write(this_client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		//gsl_write_interface(this_client,0x8,buf,4);
		gsl_ts_write(this_client,0x8,buf,4);
	}
#endif
	//printk("222 finger_num= %d\n",cinfo.finger_num);
	if(cinfo.finger_num>0 && cinfo.finger_num<6)
	{
		gsl_up_flag = 0;
		for(tmp=0;tmp<cinfo.finger_num;tmp++)
		{
			gsl_report_point(ts,cinfo.x[tmp],cinfo.y[tmp],10,cinfo.id[tmp]);
		}
	}
	else if(cinfo.finger_num == 0)
	{
		if(gsl_up_flag == 1)
		{
			goto schedule;
		}
		gsl_up_flag = 1;
	//	input_report_abs(ts->input_dev, BTN_TOUCH, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->input_dev);
	}
	input_sync(ts->input_dev);
	
schedule:
	enable_irq(this_client->irq);

}
*/
	
#ifdef FILTER_POINT
	static void filter_point(u16 x, u16 y , u8 id)
	{
		u16 x_err =0;
		u16 y_err =0;
		u16 filter_step_x = 0, filter_step_y = 0;
		
		id_sign[id] = id_sign[id] + 1;
		if(id_sign[id] == 1)
		{
			x_old[id] = x;
			y_old[id] = y;
		}
		
		x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
		y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);
	
		if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
		{
			filter_step_x = x_err;
			filter_step_y = y_err;
		}
		else
		{
			if(x_err > FILTER_MAX)
				filter_step_x = x_err; 
			if(y_err> FILTER_MAX)
				filter_step_y = y_err;
		}
	
		if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
		{
			filter_step_x >>= 2; 
			filter_step_y >>= 2;
		}
		else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
		{
			filter_step_x >>= 1; 
			filter_step_y >>= 1;
		}	
		else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
		{
			filter_step_x = filter_step_x*3/4; 
			filter_step_y = filter_step_y*3/4;
		}
		
		x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
		y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);
	
		x_old[id] = x_new;
		y_old[id] = y_new;
	}
#else
	
	static void record_point(u16 x, u16 y , u8 id)
	{
		u16 x_err =0;
		u16 y_err =0;
	
		id_sign[id]=id_sign[id]+1;
		
		if(id_sign[id]==1){
			x_old[id]=x;
			y_old[id]=y;
		}
	
		x = (x_old[id] + x)/2;
		y = (y_old[id] + y)/2;
			
		if(x>x_old[id]){
			x_err=x -x_old[id];
		}
		else{
			x_err=x_old[id]-x;
		}
	
		if(y>y_old[id]){
			y_err=y -y_old[id];
		}
		else{
			y_err=y_old[id]-y;
		}
	
		if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
			x_new = x;	   x_old[id] = x;
			y_new = y;	   y_old[id] = y;
		}
		else{
			if(x_err > 3){
				x_new = x;	   x_old[id] = x;
			}
			else
				x_new = x_old[id];
			if(y_err> 3){
				y_new = y;	   y_old[id] = y;
			}
			else
				y_new = y_old[id];
		}
	
		if(id_sign[id]==1){
			x_new= x_old[id];
			y_new= y_old[id];
		}
		
	}
#endif

#ifdef HAVE_TOUCH_KEY
	static void report_key(struct gslX680_ts_data *ts, u16 x, u16 y)
	{
		u16 i = 0;
		//printk("***********report_key\n");
		for(i = 0; i < MAX_KEY_NUM; i++) 
		{
			if((gsl_key_data[i].x_min < x) && (x < gsl_key_data[i].x_max)&&(gsl_key_data[i].y_min < y) && (y < gsl_key_data[i].y_max))
			{
				key = gsl_key_data[i].key;	
				print_info("**********key is %d\n",key);
				input_report_key(ts->input_dev, key, 1);
				//input_sync(ts->input);		
				key_state_flag = 1;
				//turn_keylight_on(&ts->lightwork);
				break;
			}
		}
	}
#endif

static void report_data(struct gslX680_ts_data *ts, u16 x, u16 y, u8 pressure, u8 id)
{
	//printk("bruce report_data: id %d, x %d, y %d \n",id, x, y);
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, pressure);
	
   	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);	
	 
	//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 5);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#ifdef HAVE_TOUCH_KEY
	if(y > TS_HEIGHT_MAX)
	{
		report_key(ts,x,y);
	}
#endif
}
 
#ifdef INCAR_GSL_GESTURE
static void gslX680_ts_report_powerkey(struct gslX680_ts_data *ts)
{
    input_report_key(ts->input_dev,KEY_POWER,1); //REPORT_KEY_VALUE
    input_sync(ts->input_dev);
    input_report_key(ts->input_dev,KEY_POWER,0);  //REPORT_KEY_VALUE
    input_sync(ts->input_dev);
    power_key_status = 1;
    gsl_gesture_status = GE_NOWORK; 
}
#endif	
static void gslX680_ts_worker(struct work_struct *work)
{
		int rc;
		//u8 read_buf[4] = {0};
		u8 id, touches;
		u16 x, y;
		int i = 0;
		
		struct gslX680_ts_data *ts = i2c_get_clientdata(this_client);
#ifdef GSL_ALG_ID
	struct gsl_touch_info cinfo;	//={0};
	u32 tmp1=0;
	u8 buf[4]={0};
#endif
	
		//printk("gslX680_ts_pen_irq_work \n");
	
#ifdef GSL_MONITOR
		if(i2c_lock_flag != 0)
			goto i2c_lock_schedule;
		else
			i2c_lock_flag = 1;
#endif
	
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1)
		goto schedule;
#endif
	
#ifdef INCAR_GSL_GESTURE
	if(gsl_gesture_flag==1 && GE_NOWORK == gsl_gesture_status){
		goto schedule;
	}
#endif
		
	
	rc = gsl_ts_read(this_client, 0x80, ts->touch_data, sizeof(ts->touch_data));
		if (rc < 0) 
		{
			dev_err(&this_client->dev, "read failed\n");
			goto schedule;
		}
	
		touches = ts->touch_data[0];
#ifdef GSL_ALG_ID
		cinfo.finger_num = touches;
		print_info("tp-gsl	finger_num = %d\n",cinfo.finger_num);
		for(i = 0; i < (touches < MAX_CONTACTS ? touches : MAX_CONTACTS); i ++)
		{
			cinfo.x[i] = join_bytes( ( ts->touch_data[4 * i + 7] & 0xf),ts->touch_data[4 * i + 6]);
			cinfo.y[i] = join_bytes(ts->touch_data[4 * i + 5],ts->touch_data[4 * i +4]);
			cinfo.id[i] = ((ts->touch_data[4 * i + 7]  & 0xf0)>>4);
			print_info("tp-gsl	before:  x[%d] = %d, y[%d] = %d, id[%d] = %d \n",i,cinfo.x[i],i,cinfo.y[i],i,cinfo.id[i]);
		}
		cinfo.finger_num=(ts->touch_data[3]<<24)|(ts->touch_data[2]<<16)
			|(ts->touch_data[1]<<8)|(ts->touch_data[0]);
		gsl_alg_id_main(&cinfo);
		tmp1=gsl_mask_tiaoping();
		print_info("[tp-gsl] tmp1=%x\n",tmp1);
		if(tmp1>0&&tmp1<0xffffffff)
		{
			buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
			gsl_ts_write(this_client,0xf0,buf,4);
			buf[0]=(u8)(tmp1 & 0xff);
			buf[1]=(u8)((tmp1>>8) & 0xff);
			buf[2]=(u8)((tmp1>>16) & 0xff);
			buf[3]=(u8)((tmp1>>24) & 0xff);
			print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
				tmp1,buf[0],buf[1],buf[2],buf[3]);
			gsl_ts_write(this_client,0x8,buf,4);
		}
		touches = cinfo.finger_num;
#endif

#ifdef INCAR_GSL_GESTURE
	print_info("--luwl--gsl_gesture_status=%d  gsl_gesture_flag=%d--\n",gsl_gesture_status,gsl_gesture_flag);
	if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
//	if(gsl_gesture_state == 2 && gsl_gesture_flag == 1){

		u8 key_data=0;
		int tmp_c = 0;
		int flag = 0;	
#ifdef INCAR_GSL_GESTURE
	//wake_lock_timeout(&gsl_wake_lock, msecs_to_jiffies(200));
#endif
		tmp_c = gsl_obtain_gesture();

    	print_info("--luwl---tmp_c=%c,power_key_status=%d.--\n", tmp_c, power_key_status);
    	if((tmp_c != 0) && (power_key_status == 0)){	
    		print_info("tp-gsl gesture %c;\n",(char)(tmp_c & 0xff));
    		gsl_gesture_c = (char)(tmp_c & 0xff);
    	}

        switch(tmp_c){
            case (int)'C':             
                print_info("tmp_c=%c.\n",tmp_c);
                gslX680_ts_report_powerkey(ts);
                input_report_key(ts->input_dev,KEY_CAMERA,1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev,KEY_CAMERA,0);
                input_sync(ts->input_dev);	
                break;
            case (int)'E':
                print_info("tmp_c=%c.\n",tmp_c);
                gslX680_ts_report_powerkey(ts);
                input_report_key(ts->input_dev,KEY_MAIL,1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev,KEY_MAIL,0);
                input_sync(ts->input_dev);
                break;
            case (int)'W':
                gslX680_ts_report_powerkey(ts);
                print_info("tmp_c=%c.\n",tmp_c);
                input_report_key(ts->input_dev,KEY_WWW,1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev,KEY_WWW,0);
                input_sync(ts->input_dev);
                break;
            case (int)'O':
                break;
            case (int)'M':
                break;
            case (int)'V':
            case (int)'N':
            case (int)'*':
                 gslX680_ts_report_powerkey(ts);
                break;	
            default:
                break;
        }

		print_info("--luwl---tmp_c=0x%x-key_data=%d,power_key_status=%d.--\n",tmp_c,key_data,power_key_status);
		goto schedule;
	}
#endif


#ifdef USE_TP_PSENSOR
			if(ps_en)
			{
				if(0 == tp_ps_report_dps(touches))
				{
					goto schedule;
				}
			}
#endif
	
		for(i = 1; i <= MAX_CONTACTS; i ++)
		{
			if(touches == 0)
				id_sign[i] = 0; 
			id_state_flag[i] = 0;
		}
		for(i = 0; i < (touches > MAX_FINGERS ? MAX_FINGERS : touches); i ++)
		{
	#ifdef GSL_ALG_ID
			id = cinfo.id[i];
			x =  cinfo.x[i];
			y =  cinfo.y[i];	
	#else	
			id = ts->touch_data[4 * i + 7] >> 4;
			x = join_bytes( ( ts->touch_data[4 * i + 7] & 0xf),ts->touch_data[4 * i + 6]);
			y = join_bytes(ts->touch_data[4 * i + 5],ts->touch_data[4 * i +4]);
	#endif
		
			if(1 <= id && id <= MAX_CONTACTS)
			{
		#ifdef FILTER_POINT
				filter_point(x, y ,id);
		#else
            #ifdef TOUCH_MODUEL_S7069A_ONCELL_XINPENG
            #else
                record_point(x, y , id);
            #endif
		#endif

            #ifdef TOUCH_MODUEL_S7069A_ONCELL_XINPENG
                report_data(ts, x, y, 3, id);  
            #else
                report_data(ts, x_new, y_new, 3, id); 
            #endif
				id_state_flag[id] = 1;
			}
		}
		for(i = 1; i <= MAX_CONTACTS; i ++)
		{	
			if( (0 == touches) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
			{
				id_sign[i]=0;
			}
			id_state_old_flag[i] = id_state_flag[i];
		}
		if(0 == touches)
		{
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
//			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
//			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_mt_sync(ts->input_dev);
	#ifdef HAVE_TOUCH_KEY
			if(key_state_flag)
			{
				input_report_key(ts->input_dev, key, 0);
				//input_sync(ts->input_dev);
				key_state_flag = 0;
			}
	#endif
		}	
		input_sync(ts->input_dev);
	
	schedule:
#ifdef GSL_MONITOR
		i2c_lock_flag = 0;
	i2c_lock_schedule:
#endif
		enable_irq(this_client->irq);
		}

static irqreturn_t gslX680_ts_interrupt(int irq, void *dev_id)
{

	struct gslX680_ts_data *gslX680_ts = (struct gslX680_ts_data *)dev_id;

	//printk("gslX680_ts_interrupt\n");

    disable_irq_nosync(this_client->irq);
	if (!work_pending(&gslX680_ts->pen_event_work)) {
		queue_work(gslX680_ts->ts_workqueue, &gslX680_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef INCAR_GSL_GESTURE
static void gsl_enter_doze(struct i2c_client *client)
{
	u8 buf[4] = {0};

	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0x1;
	buf[3] = 0x5a;
	gsl_ts_write(client,0x8,buf,4);
	gsl_gesture_status = GE_NOWORK;
	mdelay(5);
	gsl_gesture_status = GE_ENABLE;

}
static void gsl_quit_doze(struct gslX680_ts_data *ts)
{
	u8 buf[4] = {0};
	u32 tmp;

	gsl_gesture_status = GE_DISABLE;

	gslx680_hw_reset();
			
	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(this_client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x5a;
	gsl_ts_write(this_client,0x8,buf,4);
	mdelay(10);

}


static void gsl_irq_mode_change(struct i2c_client *client,u32 flag)
{
	u8 buf[4]={0};
	buf[0] = 0x6;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_ts_write(client,0xf0,buf,4);
	if(flag == 1){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
	}else if(flag == 0){
		buf[0] = 1;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
	}else{
		return;
	}
	gsl_ts_write(client,0x1c,buf,4);
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t len=0;
#if 0
	sprintf(&buf[len],"%s\n","tp gesture is on/off:");
	len += (strlen("tp gesture is on/off:")+1);
	if(gsl_gesture_flag == 1){
		sprintf(&buf[len],"%s\n","  on  ");
		len += (strlen("  on  ")+1);
	}else if(gsl_gesture_flag == 0){
		sprintf(&buf[len],"%s\n","  off  ");
		len += (strlen("  off  ")+1);
	}

	sprintf(&buf[len],"%s\n","tp gesture:");
	len += (strlen("tp gesture:")+1);
#endif
	sprintf(&buf[len],"%c\n",gsl_gesture_c);
	len += 2;	
    return len;
}
//wuhao start
static ssize_t gsl_sysfs_tpgesture_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char tmp_buf[16];
	
#if 0
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}
#endif
    return count;
}

static void gsl_request_power_idev(void)
{
	int rc;
	struct input_dev *idev;
	idev = input_allocate_device();
	if(!idev){
		return;
	}
	gsl_power_idev = idev;
	idev->name = "gsl_gesture";
	idev->id.bustype = BUS_I2C;
	input_set_capability(idev,EV_KEY,REPORT_KEY_VALUE);
	input_set_capability(idev,EV_KEY,KEY_END);

	rc = input_register_device(idev);
	if(rc){
		input_free_device(idev);
		gsl_power_idev = NULL;
	}
}
static DEVICE_ATTR(tpgesture, 0666, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesture_store);

//add by lijin 2014.9.1
static ssize_t gsl_sysfs_tpgesture_func_show(struct device *dev,struct device_attribute *attr, char *buf)
{
#if 1 
	ssize_t len=0;
#if 1
	sprintf(&buf[len],"%s\n","tp gesture is on/off:");
	len += (strlen("tp gesture is on/off:")+1);
	if(gsl_gesture_flag == 1){
		sprintf(&buf[len],"%s\n","  on  ");
		len += (strlen("  on  ")+1);
	}else if(gsl_gesture_flag == 0){
		sprintf(&buf[len],"%s\n","  off  ");
		len += (strlen("  off  ")+1);
	}

	//sprintf(&buf[len],"%s\n","tp gesture:");
	//len += (strlen("tp gesture:")+1);
#endif
	//sprintf(&buf[len],"%c\n",gsl_gesture_c);
	//len += 2;	
    return len;


#else
	//ssize_t len=0;
	//return sprintf(&buf[len],"%c\n",gsl_gesture_flag);
	return sprintf(buf,"%c\n",gsl_gesture_flag);
	//len += 2;	
    //return len;
#endif	
	
}
//wuhao start
static ssize_t gsl_sysfs_tpgesture_func_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char tmp_buf[16];
	
#if 1
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}
#endif
    return count;
}

static DEVICE_ATTR(tpgesture_func, 0666, gsl_sysfs_tpgesture_func_show, gsl_sysfs_tpgesture_func_store);

//end


static unsigned int gsl_gesture_init(void)
{
	int ret;
	struct kobject *gsl_debug_kobj;
	gsl_debug_kobj = kobject_create_and_add("sileadinc", NULL) ;
	if (gsl_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
#if 1
	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture.attr);
	//ret = device_create_file(gsl_debug_kobj, &dev_attr_tpgesture);
    if (ret)
    {
        printk("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }

	ret = sysfs_create_file(gsl_debug_kobj, &dev_attr_tpgesture_func.attr);
        //ret = device_create_file(gsl_debug_kobj, &dev_attr_tpgesture_fun);
    if (ret)
    {
        printk("%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
#else
//add by lijin 2014.9.1
	ret = sysfs_create_group(gsl_debug_kobj, &gsl_tp_attribute_group);
	if (ret < 0) 
	{
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

//end
#endif
    //gsl_request_power_idev();
	return 1;
}

#endif
static int gslX680_ts_suspend(void)
{
	u32 tmp;
	print_info("==gslX680_ts_suspend=\n");

#ifdef USE_TP_PSENSOR
    if (ps_en == 1)
    {
		printk("==gslX680_ts_suspend=USE_TP_PSENSOR,  do nothing.\n");
		ps_supend=1;
		return 0;
    }
	ps_supend=0;
#endif
	
	//version info
	//printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
	//version info
	gsl_halt_flag = 1;
#ifdef INCAR_GSL_GESTURE
	spin_lock(&resume_lock); // add 20141111
	if(power_key_status == 0){
		gsl_gesture_c = '*';
	}
	power_key_status = 0;
	if(gsl_gesture_flag == 1){
		ret = enable_irq_wake(this_client->irq);
		print_info("set_irq_wake(1) result = %d\n",ret);
		gsl_irq_mode_change(this_client,1);		
		irq_set_irq_type(this_client->irq,IRQF_TRIGGER_HIGH|IRQF_NO_SUSPEND); //IRQ_TYPE_EDGE_BOTH IRQ_TYPE_LEVEL_LOW
		gsl_enter_doze(this_client);
		spin_unlock(&resume_lock); // add 20141111		
		return 0;
	}
	spin_unlock(&resume_lock); // add 20141111		
#endif

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return 0;
	}
#endif
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
	if(2==gsl_timer_flag){
		return 0;
	}
#endif

	disable_irq_nosync(this_client->irq);

#ifdef INCAR_GSL_GESTURE
	if(gsl_gesture_flag == 1)
	{
		gpio_set_value(GSL_GPIO_RST, 1); //add by bit. 2015.5.8
	}
	else
	{
		gpio_set_value(GSL_GPIO_RST, 0);
	}
#else
	//gpio_direction_output(GSL_GPIO_RST, 0);
	//gpio_set_value(GSL_GPIO_RST, 0);
	//printk("------>gslX680_ts_suspend=4,RST=%d, RST1=%d\n", GSL_GPIO_RST, g_gslx680_ts->platform_data->reset_gpio_number);
	
	gpio_direction_output(g_gslx680_ts->platform_data->reset_gpio_number, 0);
	gpio_set_value(g_gslx680_ts->platform_data->reset_gpio_number, 0);
#endif
	return 0;
}

static int gslX680_ts_resume(void)
{
	print_info("==gslX680_ts_resume=\n");
	//int ret;
#ifdef USE_TP_PSENSOR
		if (ps_en == 1)
		{
			if(ps_supend==1)
			{
				printk("==gslX680_ts_resume=USE_TP_PSENSOR,  do nothing.\n");
				ps_supend=0;
				return 0;
			}
		}
#endif
	
#ifdef INCAR_GSL_GESTURE
	spin_lock(&resume_lock); // add 20141111
		if(gsl_gesture_flag == 1){
			ret =disable_irq_wake(this_client->irq);
			print_info("set_irq_wake(1) result = %d\n",ret);
			irq_set_irq_type(this_client->irq,IRQF_TRIGGER_RISING);
			gsl_quit_doze(this_client);
			gsl_irq_mode_change(this_client,0);
		}
		//msleep(2);
		power_key_status = 0;
	spin_unlock(&resume_lock);// add 20141111
#endif
	//printk("------>gslX680_ts_resume,RST=%d\n", GSL_GPIO_RST);
	//gpio_direction_output(GSL_GPIO_RST, 1);
	//gpio_set_value(GSL_GPIO_RST, 1);
	gpio_direction_output(g_gslx680_ts->platform_data->reset_gpio_number, 1);
	gpio_set_value(g_gslx680_ts->platform_data->reset_gpio_number, 1);
	//mdelay(20);
#ifdef GSL_TIMER
	if(2==gsl_timer_flag)
	{
		gsl_halt_flag=0;
		enable_irq(this_client->irq);
		return 0;
	}
#endif
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return 0;
	}
#endif

	gsl_reset_core(this_client);
	gsl_start_core(this_client);
	msleep(20);
	check_mem_data(this_client);
	enable_irq(this_client->irq);
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
	gsl_timer_flag = 0;
#endif
	gsl_halt_flag = 0;
	return 0;

}

#ifdef INCAR_GSL_GESTURE
static void gslx680_hw_reset(void)
{
	//struct gslx680_ts_platform_data *pdata = g_gslx680_ts->platform_data;

	gpio_set_value(GSL_GPIO_RST, 1);
	mdelay(1);
	gpio_set_value(GSL_GPIO_RST, 0);
	mdelay(10);
	gpio_set_value(GSL_GPIO_RST, 1);
	mdelay(200);
}
#endif



static void gslx680_ts_hw_init(struct gslX680_ts_data *gslx680_ts)
{
	//struct regulator *reg_vdd;
	//struct i2c_client *client = gslx680_ts->client;
	struct gslx680_ts_platform_data *pdata = gslx680_ts->platform_data;

//	printk(KERN_INFO "%s [irq=%d];[rst=%d]\n",__func__,pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_set_value(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);
	//vdd power on
	/*reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	regulator_set_voltage(reg_vdd, 2850000, 2850000);
	regulator_enable(reg_vdd);   
	msleep(100);*/
	//reset
//	ft5x0x_ts_reset();
}

#ifdef CONFIG_OF
static struct gslx680_ts_platform_data *gsl168x_ts_parse_dt(struct device *dev)
{
	struct gslx680_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int gslX680_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gslX680_ts_data *gslX680_ts;
	struct input_dev *input_dev;
	struct gslx680_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	char temp1[4]={0};
#ifdef HAVE_TOUCH_KEY
	int retry1=0;
#endif
	
	
	
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
#endif		
	//unsigned char uc_reg_value=0; 
	//u16 i = 0;
	print_info("%s\n",__func__);
#ifdef CONFIG_OF
	//struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = gsl168x_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_check_functionality_failed;
		}
	}
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}


	print_info("==kzalloc=");
	gslX680_ts = kzalloc(sizeof(*gslX680_ts), GFP_KERNEL);
	if (!gslX680_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	if(gsl_read_interface(client,0xfc,temp1,4) < 0) {
		printk("gslX680_ts_probe: gsl read interface failed\n");
		goto exit_gsl_read_interface_failed;
	}
	printk("gsl buf[0] = %x buf[1] = %x buf[2] = %x buf[3] = %x err = %d\n",temp1[0],temp1[1],temp1[2],temp1[3],err);

	mutex_init(&gsl_i2c_lock);

#ifdef USE_TP_PSENSOR
	tp_ps_init(client);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
	gsl_ts_virtual_keys_init();
#endif

#if 0 //I2C_BOARD_INFO_METHOD
	gpio_direction_output(GSL_GPIO_RST, 0);
	msleep(100);
	LDO_SetVoltLevel(LDO_LDO_SIM2, LDO_VOLT_LEVEL0);
	LDO_TurnOnLDO(LDO_LDO_SIM2);
	msleep(100);	
	gpio_direction_output(GSL_GPIO_RST, 1);
	msleep(20);

	gpio_direction_input(GSL_GPIO_IRQ);	
	gpio_set_value(GSL_GPIO_RST, 1);
	client->irq = sprd_alloc_gpio_irq(GSL_GPIO_IRQ);
	msleep(10);
#endif
  g_gslx680_ts = gslX680_ts;
	gslX680_ts->platform_data = pdata;
	this_client = client;
	i2c_set_clientdata(client, gslX680_ts);
	gslX680_ts->client = client;
	print_info("I2C addr=%x", client->addr);
	gslx680_ts_hw_init(gslX680_ts);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	INIT_WORK(&gslX680_ts->pen_event_work, gslX680_ts_worker);

	gslX680_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!gslX680_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	
	print_info("%s: ==request_irq=\n",__func__);
	print_info("%s IRQ number is %d", client->name, client->irq);
	err = request_irq(client->irq, gslX680_ts_interrupt, IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND, client->name, gslX680_ts);
	if (err < 0) {
		dev_err(&client->dev, "gslX680_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	print_info("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	gslX680_ts->input_dev = input_dev;

//
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#ifdef HAVE_TOUCH_KEY
		for(retry1 = 0; retry1 < MAX_KEY_NUM; retry1++)
		{
			input_set_capability(gslX680_ts->input_dev , EV_KEY, key_array[retry1]);
		}
#endif

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOME,  input_dev->keybit);
	__set_bit(KEY_SEARCH,  input_dev->keybit);
#ifdef INCAR_GSL_GESTURE
	__set_bit(KEY_POWER,  input_dev->keybit);
#endif
//
/*
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
*/
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, TS_WIDTH_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, TS_HEIGHT_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

#ifdef INCAR_GSL_GESTURE
	__set_bit(KEY_MAIL,  input_dev->keybit);
	__set_bit(KEY_CAMERA,  input_dev->keybit);
	__set_bit(KEY_WWW,  input_dev->keybit);
	//__set_bit(KEY_SEARCH,  input_dev->keybit);
	//__set_bit(KEY_POWER,  input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, REPORT_KEY_VALUE);		
#endif
	input_dev->name = GSLX680_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"gslX680_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#ifdef IC_COMPATIBLE
    u8 temp1[4]={0};
	gsl_read_interface(client,0xfc,temp1,4);
	if((0xb4==temp1[3])&&(0x82==temp1[2]))
		{
		gsl_cfg_index=0;//gsl1686
		}
	else if ((0x50==temp1[3])&&(0x91==temp1[2]))
		{
		gsl_cfg_index=1;//gsl3670
		}
#endif
	gsl_sw_init(this_client);
	msleep(20);
	check_mem_data(this_client);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	print_info("==register_early_suspend =");
	gslX680_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gslX680_ts->early_suspend.suspend = gslX680_ts_suspend;
	gslX680_ts->early_suspend.resume	= gslX680_ts_resume;
	register_early_suspend(&gslX680_ts->early_suspend);
#endif	

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_esd_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

#ifdef TPD_PROC_DEBUG
#if 0
    gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
    printk("[tp-gsl] [%s] gsl_config_proc = %x \n",__func__,gsl_config_proc);
    if (gsl_config_proc == NULL)
    {
        print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
    }
    else
    {
        gsl_config_proc->read_proc = gsl_config_read_proc;
        gsl_config_proc->write_proc = gsl_config_write_proc;
    }
#else
    proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
#endif
	gsl_proc_flag = 0;
#endif

#ifdef GSL_ALG_ID
    #ifdef IC_COMPATIBLE
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
	#else
	gsl_DataInit(gsl_config_data_id);
	#endif
#endif

#ifdef INCAR_GSL_GESTURE

	gsl_FunIICRead(gsl_read_oneframe_data);
	gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);

	#if 1
	//wake_lock_init(&gsl_wake_lock, WAKE_LOCK_SUSPEND, "gsl_wake_lock");	
	gsl_gesture_init();
	spin_lock_init(&resume_lock);  // add 20141030	
	#else
	ret= sysfs_create_group(&client->dev.kobj, &gslx680_gesture_group);
	if (ret < 0) {		
		printk("--luwl--sysfs_create_group fail--\n");
		return -ENOMEM;													
	}
	#endif
#endif

	//wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

  enable_irq(client->irq);
	print_info("%s: ==probe over =\n",__func__);
  return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, gslX680_ts);
exit_irq_request_failed:
	cancel_work_sync(&gslX680_ts->pen_event_work);
	destroy_workqueue(gslX680_ts->ts_workqueue);
exit_create_singlethread:
exit_gsl_read_interface_failed:
	print_info("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(gslX680_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	//sprd_free_gpio_irq(gslX680_ts_setup.irq);
	return err;
}

static int  gslX680_ts_remove(struct i2c_client *client)
{

	struct gslX680_ts_data *gslX680_ts = i2c_get_clientdata(client);

	print_info("==gslX680_ts_remove=\n");

#ifdef USE_TP_PSENSOR
	tp_ps_uninit();
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gslX680_ts->early_suspend);
#endif
	free_irq(client->irq, gslX680_ts);
	//sprd_free_gpio_irq(gslX680_ts_setup.irq);
	input_unregister_device(gslX680_ts->input_dev);
	kfree(gslX680_ts);
	cancel_work_sync(&gslX680_ts->pen_event_work);
	destroy_workqueue(gslX680_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

	//LDO_TurnOffLDO(LDO_LDO_SIM2);

	return 0;
}

#if defined(CONFIG_PM_SLEEP)
static int gslX680_pm_suspend(struct device *dev)
{
    return gslX680_ts_suspend();
}

static int gslX680_pm_resume(struct device *dev)
{
	return gslX680_ts_resume();
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gslX680_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gslX680_pm_suspend, gslX680_pm_resume)
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id gsl_of_match[] = {
       { .compatible = "gslx680,gslx680_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, gsl_of_match);
#endif
static const struct i2c_device_id gslX680_ts_id[] = {
	{ GSLX680_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, gslX680_ts_id);

static struct i2c_driver gslX680_ts_driver = {
	.probe		= gslX680_ts_probe,
	.remove		= gslX680_ts_remove,
	.id_table	= gslX680_ts_id,
	.driver	= {
		.name	= GSLX680_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gsl_of_match,
#endif
#if defined(CONFIG_PM_SLEEP)
		.pm = &gslX680_pm_ops,
#endif
	},
};

#if I2C_BOARD_INFO_METHOD
static int __init gslX680_ts_init(void)
{
	int ret;
	print_info("==gslX680_ts_init==\n");
	ret = i2c_add_driver(&gslX680_ts_driver);
	return ret;
}

static void __exit gslX680_ts_exit(void)
{
	print_info("==gslX680_ts_exit==\n");
	i2c_del_driver(&gslX680_ts_driver);
}
#else //register i2c device&driver dynamicly

int sprd_add_i2c_device(struct sprd_i2c_setup_data *i2c_set_data, struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret,err;


	print_info("%s : i2c_bus=%d; slave_address=0x%x; i2c_name=%s",__func__,i2c_set_data->i2c_bus, \
		    i2c_set_data->i2c_address, i2c_set_data->type);

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = i2c_set_data->i2c_address;
	strlcpy(info.type, i2c_set_data->type, I2C_NAME_SIZE);
	if(i2c_set_data->irq > 0)
		info.irq = i2c_set_data->irq;

	adapter = i2c_get_adapter( i2c_set_data->i2c_bus);
	if (!adapter) {
		print_info("%s: can't get i2c adapter %d\n",
			__func__,  i2c_set_data->i2c_bus);
		err = -ENODEV;
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		print_info("%s:  can't add i2c device at 0x%x\n",
			__func__, (unsigned int)info.addr);
		err = -ENODEV;
		goto err_driver;
	}

	i2c_put_adapter(adapter);

	ret = i2c_add_driver(driver);
	if (ret != 0) {
		print_info("%s: can't add i2c driver\n", __func__);
		err = -ENODEV;
		goto err_driver;
	}	

	return 0;

err_driver:
	return err;
}

void sprd_del_i2c_device(struct i2c_client *client, struct i2c_driver *driver)
{
	print_info("%s : slave_address=0x%x; i2c_name=%s",__func__, client->addr, client->name);
	i2c_unregister_device(client);
	i2c_del_driver(driver);
}

static int __init gslX680_ts_init(void)
{
	int gslX680_irq;

	print_info("%s\n", __func__);

	//reset
	gpio_direction_output(GSL_GPIO_RST, 0);
	msleep(100);
	LDO_SetVoltLevel(LDO_LDO_SIM2, LDO_VOLT_LEVEL0);
	LDO_TurnOnLDO(LDO_LDO_SIM2);
	msleep(100);	
	gpio_direction_output(GSL_GPIO_RST, 1);
	msleep(20);

	gpio_direction_input(GSL_GPIO_IRQ);	
	gpio_set_value(GSL_GPIO_RST, 1);
	gslX680_irq=sprd_alloc_gpio_irq(GSL_GPIO_IRQ);
	msleep(10);
	
	gslX680_ts_setup.i2c_bus = 3;
	gslX680_ts_setup.i2c_address = GSLX680_TS_ADDR;
	strcpy (gslX680_ts_setup.type,GSLX680_TS_NAME);
	gslX680_ts_setup.irq = gslX680_irq;
	return sprd_add_i2c_device(&gslX680_ts_setup, &gslX680_ts_driver);
}

static void __exit gslX680_ts_exit(void)
{
	print_info("%s\n", __func__);
	
	sprd_del_i2c_device(this_client, &gslX680_ts_driver);
}
#endif

module_init(gslX680_ts_init);
module_exit(gslX680_ts_exit);

MODULE_AUTHOR("leweihua");
MODULE_DESCRIPTION("GSLX680 TouchScreen Driver");
MODULE_LICENSE("GPL");

