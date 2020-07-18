/* hshcal001_i2c.c
 *
 * Humidity device driver for I2C (HSHCAL001)
 *
 * Copyright (C) 2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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
 
#include "lt8911exb_i2c.h"

static DEFINE_MUTEX(hshcal_lock);

static struct platform_device *pdev;
//static struct input_dev *hshcal_idev;
static struct i2c_client *client_hshcal = NULL;
//static struct delayed_work hshcal_work;

int gpio_avdd  =0;
int gpio_avee  =0;
int gpio_vdd   =0;
int gpio_bl    =0;
int gpio_reset =0;

u8 Read_DPCD010A = 0x00;

bool ScrambleMode = 0;

u8 swing_req = 0x00;

//#define _Test_Pattern_ // 输出test pattern

//#define  _read_edid_ // 读取EDP屏的EDID

#define _Msa_Active_Only_

#define _1080P_eDP_Panel_
//#define _1366x768_eDP_Panel_

// 设置输入的MIPI信号的Lane数
#define _MIPI_Lane_	4 // MIPI Lane 1,2,3,4

//******************************************************//
#ifdef _1080P_eDP_Panel_

// 根据前端MIPI信号的Timing，修改以下参数：
static int MIPI_Timing[] = 
	//H_act V_act	H_total V_total H_BP	H_sync	V_sync	V_BP
//	{1920,	1080,	2200,	1125,	148,	44, 	5,		36};// 1080P  Vesa Timing
//	{1920,	1080,	2080,	1111,	80,		32,		5,		16};
	{1920,	1080,	2080,	1111,	48,		32,		5,		3};
// 根据eDP屏的规格书，定义LT8911B eDP输出的Lane数。
#define _2_Lane_

// 根据eDP屏的色深，定义LT8911B 的色深设置
#define _8_Bit_ColorDepth_ // eDP panel Color Depth，16.7M color
//#define _6_Bit_ColorDepth_ // eDP panel Color Depth，262K color

#define eDP_lane		2
#define PCR_PLL_PREDIV	0x40

// 根据前端MIPI信号的Timing，修改以下参数：
static int EXB_MIPI_Timing[] =
// hfp,	hs,	hbp,	hact,	htotal,	vfp,	vs,	vbp,	vact,	vtotal,	pixel_CLK/10000
{ 80, 32, 48, 1920, 2080, 23, 5, 3, 1080, 1111, 13850 };
//{ 88, 44, 148, 1920, 2200, 4,  5, 36,  1080, 1125, 12800};

//#define _6bit_ // eDP panel Color Depth，262K color
#define _8bit_ // eDP panel Color Depth，16.7M color

#endif

//******************************************************//
#ifdef _1366x768_eDP_Panel_

// 根据前端MIPI信号的Timing，修改以下参数：
static int MIPI_Timing[] = 
	//H_act V_act	H_total V_total H_BP	H_sync	V_sync	V_BP
	{1366,	768,	1500,	800,	64,	56,	3,	28};// 1366x768 VESA Timing
//	{1366,	768,	1592,	800,	104,	96,	3,	26};// N11BGE-E32 76.42MHz
//	{1366,	768,	1542,	800,	54,	96,	3,	26};// N11BGE-E32 74MHz

// 根据eDP屏的规格书，定义LT8911B eDP输出的Lane数。
#define _1_Lane_

// 根据eDP屏的色深，定义LT8911B 的色深设置
#define _6_Bit_ColorDepth_ // eDP panel Color Depth，262K color

#endif

//******************************************************//
//#define _Test_Pattern_ 1

//#define _SSC_En_ // Spread-Spectrum Clock 
#ifdef _SSC_En_
#define _SSC_	0x0f
#else
#define _SSC_	0x07
#endif


#define Swing_Level_Close 0x00

#define Swing_Level_0_H 0x00
#define Swing_Level_0_L 0xa0

#define Swing_Level_1_H 0x00
#define Swing_Level_1_L 0xf0

#define Swing_Level_2_H 0x01
#define Swing_Level_2_L 0x40

#define Swing_Level_3_H 0x02
#define Swing_Level_3_L 0xb4

static u8	DPCD0000H;
static u8	DPCD0001H;
static u8	DPCD0002H;
static u8	DPCD0003H;
static u8	DPCD0004H;
static u8	DPCD0005H;
static u8	DPCD0006H;
static u8	DPCD0007H;
static u8	DPCD0008H;
static u8	DPCD0009H;
static u8	DPCD000aH;
static u8	DPCD000bH;
static u8	DPCD0200H;
static u8	DPCD0201H;
u8			DPCD0202H;
static u8	DPCD0203H;
static u8	DPCD0204H;
u8			DPCD0205H;
u8			DPCD0206H;

u8			Count	   = 0x00;
u8			Cur_Temp   = 0x00;
u8			Pre_Temp   = 0x00;
u8			D_value	   = 0x00;
//static int flgEnaH = 0;
//static int flgEnaT = 0;
//static int delay = 200;
//static int flgSuspend = 0;
//static int flgSkip = 1;



static int hshcal_parse(struct device *dev)
{
	struct device_node *np = dev->of_node;
	
	LT_INFO("--->>>> hshcal_parse!\n ");
	
	gpio_avdd = of_get_named_gpio(np, "avdd-gpio", 0);
	if (gpio_avdd < 0)
	{
		LT_ERR("--->>>> Unable to get avdd-gpio!\n ");
		return -ENODEV;
	}
	
	gpio_avee = of_get_named_gpio(np, "avee-gpio", 0);
	if (gpio_avee < 0)
	{
		LT_ERR("--->>>> Unable to get avee-gpio!\n ");
		return -ENODEV;
	}	
	
	gpio_vdd = of_get_named_gpio(np, "vdd-gpio", 0);
	if (gpio_vdd < 0)
	{
		LT_ERR("--->>>> Unable to get vdd-gpio!\n ");
		return -ENODEV;
	}

	gpio_bl = of_get_named_gpio(np, "bl-gpio", 0);
	if (gpio_bl < 0)
	{
		LT_ERR("--->>>> Unable to get bl-gpio!\n ");
		return -ENODEV;
	}
	
	gpio_reset = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_reset < 0)
	{
		LT_ERR("--->>>> Unable to get reset-gpio!\n ");
		return -ENODEV;
	}
	
  if (!gpio_is_valid(gpio_avdd))
  {
  	LT_ERR("--->>>> avdd-gpio is not valid!\n ");
  	return -ENODEV;
  }
  
  if (!gpio_is_valid(gpio_avee))
  {
  	LT_ERR("--->>>> avee-gpio is not valid!\n ");
  	return -ENODEV;
  }
  
  if (!gpio_is_valid(gpio_vdd))
  {
  	LT_ERR("--->>>> vdd-gpio is not valid!\n ");
  	return -ENODEV;
  }
  
  if (!gpio_is_valid(gpio_bl))
  {
  	LT_ERR("--->>>> bl-gpio is not valid!\n ");
  	return -ENODEV;
  }
  
  if (!gpio_is_valid(gpio_reset))
  {
  	LT_ERR("--->>>> reset-gpio is not valid!\n ");
  	return -ENODEV;
  }

	return 0;
}

static int reset_init(void)
{
	gpio_request(gpio_vdd, "LCD VDD");
	gpio_direction_output(gpio_vdd, 1);
	//gpio_free(gpio_vdd);
	
	msleep(5);
	
	gpio_request(gpio_avee, "LCD AVDD1");
	gpio_direction_output(gpio_avee, 1);	
	//gpio_free(gpio_avdd);
	//gpio_free(gpio_avee);
	
	msleep(5);

	gpio_request(gpio_reset, "LCD_RST");
	gpio_direction_output(gpio_reset, 1);
	msleep(40);
	gpio_direction_output(gpio_reset, 0);
	msleep(40);
	gpio_direction_output(gpio_reset, 1);
	msleep(100);
	//gpio_free(gpio_reset);
	
	LT_INFO("reset_init!\n");
	return 0;
}
	
static u8 HDMI_WriteI2C_Byte(u8 reg,u8 data)
{

	struct i2c_msg xfer_msg[1];
    u8 buf[2];
	buf[0] = reg;
    buf[1]= data;
	xfer_msg[0].addr = client_hshcal->addr;
	xfer_msg[0].len = 2;
	xfer_msg[0].flags = client_hshcal->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;

	return i2c_transfer(client_hshcal->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}
static int HDMI_ReadI2C_Byte(u8 reg)
{
/*
    int err;
    int tries = 0;
	u8 data=0;

    struct i2c_msg msgs[] = {
        {
            .addr  = client_hshcal->addr,
            .flags = 0,
            .len   = 1,
            .buf   = reg,
        },
        {
            .addr  = client_hshcal->addr,
            .flags = I2C_M_RD,
            .len   = 1,
            .buf   = &data,
        },
    };

    do {
        err = i2c_transfer(client_hshcal->adapter, msgs, 2);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        dev_err(&client_hshcal->adapter->dev, "read transfer error\n");
        return  -1;
    } else {
     return data;
    }*/
	
	
	 return i2c_smbus_read_byte_data(client_hshcal,reg);
}

void DpcdWrite( u32 Address, u8 Data )
{
	u8	AddressH   = 0x0f & ( Address >> 16 );
	u8	AddressM   = 0xff & ( Address >> 8 );
	u8	AddressL   = 0xff & Address;

	HDMI_WriteI2C_Byte( 0xff, 0x80 );

	HDMI_WriteI2C_Byte( 0x62, 0xbd );
	HDMI_WriteI2C_Byte( 0x62, 0xbf );   // ECO(AUX reset)

	HDMI_WriteI2C_Byte( 0x36, 0x00 );
	HDMI_WriteI2C_Byte( 0x30, 0x0f );//0x10 );   // 0x0f , 0x10
	HDMI_WriteI2C_Byte( 0x33, AddressL );
	HDMI_WriteI2C_Byte( 0x34, AddressM );
	HDMI_WriteI2C_Byte( 0x35, AddressH );
	HDMI_WriteI2C_Byte( 0x37, Data );
	HDMI_WriteI2C_Byte( 0x36, 0x20 );
}






u8 DpcdRead( u32 Address )
{
	u8	read_cnt   = 0x03;
	u8	DpcdValue  = 0x00;
	u8	AddressH   = 0x0f & ( Address >> 16 );
	u8	AddressM   = 0xff & ( Address >> 8 );
	u8	AddressL   = 0xff & Address;

	HDMI_WriteI2C_Byte( 0xff, 0x80 );

	HDMI_WriteI2C_Byte( 0x62, 0xbd );
	HDMI_WriteI2C_Byte( 0x62, 0xbf );   // ECO(AUX reset)

	HDMI_WriteI2C_Byte( 0x36, 0x00 );
	HDMI_WriteI2C_Byte( 0x30, 0x8f );//0x90 );   //0x8f , 0x90
	HDMI_WriteI2C_Byte( 0x33, AddressL );
	HDMI_WriteI2C_Byte( 0x34, AddressM );
	HDMI_WriteI2C_Byte( 0x35, AddressH );
	HDMI_WriteI2C_Byte( 0x36, 0x20 );

	mdelay( 2 );                      //必要的

	if( HDMI_ReadI2C_Byte( 0x39 ) == 0x01 )
	{
		DpcdValue = HDMI_ReadI2C_Byte( 0x38 );
	}else
	{
		while( ( HDMI_ReadI2C_Byte( 0x39 ) != 0x01 ) && ( read_cnt > 0 ) )
		{
			HDMI_WriteI2C_Byte( 0x36, 0x00 );
			HDMI_WriteI2C_Byte( 0x36, 0x20 );
			read_cnt--;
			mdelay( 2 );
		}

		if( HDMI_ReadI2C_Byte( 0x39 ) == 0x01 )
		{
			DpcdValue = HDMI_ReadI2C_Byte( 0x38 );
		}
	}
	return DpcdValue;
}

void adj_swing( void )
{
	u8 ret = 0;

	swing_req = DPCD0206H & 0x0f;   //lane 0
	HDMI_WriteI2C_Byte( 0xff, 0x81 );

	switch( swing_req )
	{
		case 0x00:                  //0dB_400mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0xa0 );
			HDMI_WriteI2C_Byte( 0x11, 0x00 );
			ret = 0x01;//0x00;
			break;

		case 0x01:                  //0dB_600mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0xd0 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x00 );
			ret = 0x01;
			break;

		case 0x02:                  //0dB_800mV
			HDMI_WriteI2C_Byte( 0x18, 0x01 );
			HDMI_WriteI2C_Byte( 0x19, 0x20 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x00 );
			ret = 0x02;
			break;

		case 0x03:                  //0dB_1200mV(max 1000mV)
			HDMI_WriteI2C_Byte( 0x18, 0x01 );
			HDMI_WriteI2C_Byte( 0x19, 0xa0 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x00 );
			ret = 0x07;
			break;

		case 0x04:                  //3.5dB_400mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0x98 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11,  0x08 );//0x28 );
			ret = 0x08;
			break;

		case 0x05:                  //3.5dB_600mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0xf0 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x18 );//0x38 );
			ret = 0x09;
			break;

		case 0x06:                  //3.5dB_800mV
			HDMI_WriteI2C_Byte( 0x18, 0x01 );
			HDMI_WriteI2C_Byte( 0x19, 0xa0 );
			                        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x50 );//0x70 );
			ret = 0x0a;
			break;

		case 0x07:
			break;

		case 0x08:  //6dB_400mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0xa0 );
			        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x24 );//0x44 );
			ret = 0x10;
			break;

		case 0x09:  //6dB_800mV
			HDMI_WriteI2C_Byte( 0x18, 0x01 );
			HDMI_WriteI2C_Byte( 0x19, 0x00 );
			        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x38 );//0x58 );
			ret = 0x11;
			break;

		case 0x0a:
			break;

		case 0x0b:
			ret = 0x17;
			break;

		case 0x0c:  //9.5dB_400mV
			HDMI_WriteI2C_Byte( 0x18, 0x00 );
			HDMI_WriteI2C_Byte( 0x19, 0xc0 );
			        //HDMI_WriteI2C_Byte(0x10,0x00);
			HDMI_WriteI2C_Byte( 0x11, 0x58 );//0x78 );
			ret = 0x79;//0x78;
			break;

		case 0x0d:
			break;

		case 0x0e:
			ret = 0x3a;
			break;

		case 0x0f:
			break;

		default:  break;
	}

	DpcdWrite( 0x0103, ret );

#ifdef _2_Lane_

	ret = 0x00;

	swing_req = DPCD0206H & 0xf0;   //lane 1
	HDMI_WriteI2C_Byte( 0xff, 0x81 );

	switch( swing_req )
	{
		case 0x00:                  //0dB_400mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0xa0 );
			HDMI_WriteI2C_Byte( 0x13, 0x00 );
			ret = 0x01;//0x00;
			break;

		case 0x10:                  //0dB_600mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0xd0 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x00 );
			ret = 0x01;
			break;

		case 0x20:                  //0dB_800mV
			HDMI_WriteI2C_Byte( 0x1a, 0x01 );
			HDMI_WriteI2C_Byte( 0x1b, 0x20 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x00 );
			ret = 0x02;
			break;

		case 0x30:                  //0dB_1200mV(max 1000mV)
			HDMI_WriteI2C_Byte( 0x1a, 0x01 );
			HDMI_WriteI2C_Byte( 0x1b, 0xa0 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x00 );
			ret = 0x07;
			break;

		case 0x40:                  //3.5dB_400mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0x98 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x08 );//0x28 );
			ret = 0x08;
			break;

		case 0x50:                  //3.5dB_600mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0xf0 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x18 );//0x38 );
			ret = 0x09;
			break;

		case 0x60:                  //3.5dB_800mV
			HDMI_WriteI2C_Byte( 0x1a, 0x01 );
			HDMI_WriteI2C_Byte( 0x1b, 0xa0 );
			                        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x50 );//0x70 );
			ret = 0x0a;
			break;

		case 0x70:
			break;

		case 0x80:  //6dB_400mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0xa0 );
			        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x24 );//0x44 );
			ret = 0x12;
			break;

		case 0x90:  //6dB_800mV
			HDMI_WriteI2C_Byte( 0x1a, 0x01 );
			HDMI_WriteI2C_Byte( 0x1b, 0x00 );
			        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x38 );//0x58 );
			ret = 0x13;
			break;

		case 0xa0:
			break;

		case 0xb0:
			ret = 0x17;
			break;

		case 0xc0:  //9.5dB_400mV
			HDMI_WriteI2C_Byte( 0x1a, 0x00 );
			HDMI_WriteI2C_Byte( 0x1b, 0xc0 );
			        //HDMI_WriteI2C_Byte(0x12,0x00);
			HDMI_WriteI2C_Byte( 0x13, 0x58 );//0x78 );
			ret = 0x79;//0x78;
			break;

		case 0xd0:
			break;

		case 0xe0:
			ret = 0x3a;
			break;

		case 0xf0:
			break;

		default:  break;
	}

	DpcdWrite( 0x0104, ret );

#endif
}

void LT8911_AUX_Training( void )
{
	u8 swing_adj_cnt = 0x00;
	DPCD0202H  = 0x00;
	DPCD0000H  = DpcdRead( 0x0000 );
	DPCD0200H  = DpcdRead( 0x0200 );
	DPCD0201H  = DpcdRead( 0x0201 );
	DPCD0202H  = DpcdRead( 0x0202 );
	DPCD0203H  = DpcdRead( 0x0203 );
	DPCD0204H  = DpcdRead( 0x0204 );
	DPCD0205H  = DpcdRead( 0x0205 );
	DPCD0000H  = DpcdRead( 0x0000 );
	DPCD0001H  = DpcdRead( 0x0001 );
	DPCD0002H  = DpcdRead( 0x0002 );
	DPCD0003H  = DpcdRead( 0x0003 );
	DPCD0004H  = DpcdRead( 0x0004 );
	DPCD0005H  = DpcdRead( 0x0005 );
	DPCD0006H  = DpcdRead( 0x0006 );
	DPCD0007H  = DpcdRead( 0x0007 );
	DPCD0008H  = DpcdRead( 0x0008 );
	DPCD0009H  = DpcdRead( 0x0009 );
	DPCD000aH  = DpcdRead( 0x000a );
	DPCD000bH  = DpcdRead( 0x000b );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
	HDMI_WriteI2C_Byte( 0x62, 0x3f );   //Reset dp video

#ifdef _2_Lane_
	HDMI_WriteI2C_Byte( 0x03, 0x42 );   //41-1lane,42-2lane,44-4lane
#endif

#ifdef _1_Lane_
	HDMI_WriteI2C_Byte( 0x03, 0x41 );   //41-1lane,42-2lane,44-4lane
#endif

	HDMI_WriteI2C_Byte( 0x65, 0xf1 );
	mdelay( 5 );
	HDMI_WriteI2C_Byte( 0x65, 0xf3 );

	HDMI_WriteI2C_Byte( 0x04, 0x14 );

	HDMI_WriteI2C_Byte( 0xff, 0x84 ); //register bank
	//  HDMI_WriteI2C_Byte(0x14,0x01);
	HDMI_WriteI2C_Byte( 0x14, 0x81 );
	HDMI_WriteI2C_Byte( 0x14, 0x82 );

	DpcdWrite( 0x0600, 0x01 );

	if( DpcdRead( 0x0600 ) != 0x01 )
	{
		DpcdWrite( 0x0600, 0x01 );
	}

	DpcdWrite( 0x0100, 0x0a );

#ifdef _2_Lane_
	DpcdWrite( 0x0101, 0x82 );  // 2 lane
#endif

#ifdef _1_Lane_
	DpcdWrite( 0x0101, 0x81 );  // 1 lane
#endif

	DpcdWrite( 0x010a, 0x00 );
	
#ifdef _SSC_En_
	DpcdWrite(0x0107,0x10);// Main-Link signal is down-spread 
#endif

	//  DpcdWrite(0x0107,0x00);
	//  DpcdWrite(0x0108,0x01);

	if( DpcdRead( 0x0100 ) != 0x0a )
	{
		DpcdWrite( 0x0100, 0x0a );
	}

#ifdef _2_Lane_
	if( DpcdRead( 0x0101 ) != 0x82 )    // 2 Lane
	{
		DpcdWrite( 0x0101, 0x82 );
	}
#endif

#ifdef _1_Lane_
	if( DpcdRead( 0x0101 ) != 0x81 )    // 1 Lane
	{
		DpcdWrite( 0x0101, 0x81 );
	}
#endif

	if( DpcdRead( 0x010a ) != 0x00 )
	{
		DpcdWrite( 0x010a, 0x00 );
	}

	//  DpcdWrite(0x0102,0x00);
	DpcdWrite( 0x0102, 0x01 ); // sent TPS1
	DpcdWrite( 0x0103, 0x01 );//0x00 );

#ifdef _2_Lane_
	DpcdWrite( 0x0104, 0x01 );//0x00 );
#endif

	if( DpcdRead( 0x0102 ) != 0x01 )
	{
		DpcdWrite( 0x0102, 0x01 );
	}

	mdelay( 16 );
	DPCD0204H  = DpcdRead( 0x0204 );
	DPCD0202H  = DpcdRead( 0x0202 );

	swing_adj_cnt = 0x05;

#ifdef _2_Lane_
	DPCD0202H = DPCD0202H & 0x11;                                       // 2 Lane 0x11 ; 1 Lane 0x01

	while( ( ( DPCD0202H & 0x11 ) != 0x11 ) && ( swing_adj_cnt > 0 ) )  // 1080P 0x11 ; 1366 0x01
#endif

#ifdef _1_Lane_
	DPCD0202H = DPCD0202H & 0x01;                                       // 2 Lane 0x11 ; 1 Lane 0x01

	while( ( ( DPCD0202H & 0x01 ) != 0x01 ) && ( swing_adj_cnt > 0 ) )  // 1080P 0x11 ; 1366 0x01
#endif
	{
		DPCD0206H = DpcdRead( 0x0206 );
		adj_swing( );
		swing_adj_cnt--;
		mdelay( 1 );
		DPCD0202H = DpcdRead( 0x0202 );
#ifdef _2_Lane_
		DPCD0202H = DPCD0202H & 0x11;   // 2 Lane 0x11 ; 1 Lane 0x01
#endif

#ifdef _1_Lane_
		DPCD0202H = DPCD0202H & 0x01;   // 2 Lane 0x11 ; 1 Lane 0x01
#endif
	}

	//  HDMI_WriteI2C_Byte(0xff,0x82);   //for debug
	//  HDMI_WriteI2C_Byte(0x1b,DPCD0202H);

#ifdef _2_Lane_
	if( DPCD0202H == 0x11 )                 // 2 Lane 0x11 ; 1 Lane 0x01
#endif

#ifdef _1_Lane_
	if( DPCD0202H == 0x01 )                 // 2 Lane 0x11 ; 1 Lane 0x01
#endif
	{
		HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
		HDMI_WriteI2C_Byte( 0x04, 0x18 );

		HDMI_WriteI2C_Byte( 0xff, 0x84 );   //register bank
		//  HDMI_WriteI2C_Byte(0x14,0x04);
		HDMI_WriteI2C_Byte( 0x14, 0x84 );
		HDMI_WriteI2C_Byte( 0x14, 0x88 );   //0x88

		DpcdWrite( 0x0102, 0x02 );          // sent TPS2
		if( DpcdRead( 0x0102 ) != 0x02 )
		{
			DpcdWrite( 0x0102, 0x02 );
		}

		mdelay( 16 );
		DPCD0204H  = DpcdRead( 0x0204 );
		DPCD0202H  = DpcdRead( 0x0202 );

		swing_adj_cnt = 0x05;

#ifdef _2_Lane_
		while( ( ( DPCD0202H & 0x77 ) != 0x77 ) && ( swing_adj_cnt > 0 ) )  // 2 Lane 0x77 ; 1 Lane 0x07
#endif

#ifdef _1_Lane_
		while( ( ( DPCD0202H & 0x07 ) != 0x07 ) && ( swing_adj_cnt > 0 ) )  // 2 Lane 0x77 ; 1 Lane 0x07
#endif
		{
			DPCD0206H = DpcdRead( 0x0206 );
			HDMI_WriteI2C_Byte( 0xff, 0x84 );                               //register bank
			HDMI_WriteI2C_Byte( 0x14, 0x08 );
			HDMI_WriteI2C_Byte( 0x14, 0x88 );
			adj_swing( );
			swing_adj_cnt--;
			mdelay( 1 );
			DPCD0202H  = DpcdRead( 0x0202 );
			DPCD0204H  = DpcdRead( 0x0204 );
		}
	}

	//  HDMI_WriteI2C_Byte(0xff,0x82);//register bank
	//  HDMI_WriteI2C_Byte(0x1c,DPCD0202H);
	
	DpcdWrite( 0x0102, 0x00 );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );       //register bank
	HDMI_WriteI2C_Byte( 0x04, 0x10 );

	HDMI_WriteI2C_Byte( 0xff, 0x84 );       //register bank
	HDMI_WriteI2C_Byte( 0x14, 0x80 );
	HDMI_WriteI2C_Byte( 0x14, 0xc0 );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );       //register bank
	HDMI_WriteI2C_Byte( 0x62, 0xbf );
	HDMI_WriteI2C_Byte( 0xff, 0x88 );       //register bank

	if( HDMI_ReadI2C_Byte( 0x24 ) != 0xc0 )
	{
		HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
		HDMI_WriteI2C_Byte( 0x62, 0x3f );
		HDMI_WriteI2C_Byte( 0x62, 0xbf );
	}

	//HDMI_WriteI2C_Byte( 0xff, 0x80 );       //register bank
	//HDMI_WriteI2C_Byte( 0x62, 0xbf );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );       //register bank
	HDMI_WriteI2C_Byte( 0x65, 0xf1 );
	mdelay( 5 );
	HDMI_WriteI2C_Byte( 0x65, 0xf3 );

	DpcdWrite( 0x0102, 0x00 );              // sent data

	if( DpcdRead( 0x0102 ) != 0x00 )
	{
		DpcdWrite( 0x0102, 0x00 );
	}

	if( DpcdRead( 0x0600 ) != 0x01 )
	{
		DpcdWrite( 0x0600, 0x01 );  //
	}

	if( DpcdRead( 0x010a ) != 0x00 )
	{
		DpcdWrite( 0x010a, 0x00 );  //
	}

	DPCD0202H = DpcdRead( 0x0202 );
}

void LT8911B_config( void )
{
	// 刷寄存器之前，先Reset LT8911/B ,用GPIO 先拉低LT8911B的复位脚 100ms左右，再拉高，保持100ms

                                // IIC 地址
	HDMI_WriteI2C_Byte( 0xff, 0x81 );               //register bank
	HDMI_WriteI2C_Byte( 0x00, 0x04 );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );               //register bank
	HDMI_WriteI2C_Byte( 0x62, 0x3f );
	
	HDMI_WriteI2C_Byte( 0x7a, _SSC_ );//0x07 );
	HDMI_WriteI2C_Byte( 0x71, 0x36 );
	HDMI_WriteI2C_Byte( 0x72, 0x00 );
	HDMI_WriteI2C_Byte( 0x73, 0x00 );

	HDMI_WriteI2C_Byte( 0x63, 0x7f );
	HDMI_WriteI2C_Byte( 0x63, 0xff );

	///////////txpll_analog///////
	HDMI_WriteI2C_Byte( 0xff, 0x81 );   //register bank
	HDMI_WriteI2C_Byte( 0x0e, 0x37 );
	HDMI_WriteI2C_Byte( 0x01, 0x18 );
	HDMI_WriteI2C_Byte( 0x02, 0x42 );
	HDMI_WriteI2C_Byte( 0x04, 0x00 );
	HDMI_WriteI2C_Byte( 0x04, 0x01 );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
	HDMI_WriteI2C_Byte( 0x61, 0x7f );
	HDMI_WriteI2C_Byte( 0x61, 0xff );

	HDMI_WriteI2C_Byte( 0xff, 0x81 );   //register bank
	HDMI_WriteI2C_Byte( 0x05, 0x13 );

	//////////txpll_digtal////////
	HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
	HDMI_WriteI2C_Byte( 0x74, 0x41 );
	HDMI_WriteI2C_Byte( 0x75, 0x03 );
	HDMI_WriteI2C_Byte( 0x76, 0x0a );
	HDMI_WriteI2C_Byte( 0x78, 0x0a );

	//-------------------------------------------//
	HDMI_WriteI2C_Byte( 0xff, 0x81 );   //register bank
	HDMI_WriteI2C_Byte( 0x0e, 0x37 );

#ifdef _1_Lane_

	// 1 Lane eDP Output
	HDMI_WriteI2C_Byte( 0x22, 0x11 );   // 关闭 LANE1 / LANE2 / LANE3 SWING 的电流开关
	HDMI_WriteI2C_Byte( 0x23, 0x3e );   // 关闭 LANE1 / LANE2 / LANE3 pre-emphase 的电流开关
	HDMI_WriteI2C_Byte( 0x25, 0x08 );

	HDMI_WriteI2C_Byte( 0x18, Swing_Level_0_H );
	HDMI_WriteI2C_Byte( 0x19, Swing_Level_0_L );

#else
	// 2 Lane eDP Output
	HDMI_WriteI2C_Byte( 0x22, 0x33 );   // 关闭 LANE2 / LANE3 SWING的电流开关
	HDMI_WriteI2C_Byte( 0x23, 0x3c );   // 关闭 LANE2 / LANE3 pre-emphase的电流开关
	HDMI_WriteI2C_Byte( 0x25, 0x08 );

	HDMI_WriteI2C_Byte( 0x18, Swing_Level_0_H );
	HDMI_WriteI2C_Byte( 0x19, Swing_Level_0_L );

	HDMI_WriteI2C_Byte( 0x1a, Swing_Level_0_H );
	HDMI_WriteI2C_Byte( 0x1b, Swing_Level_0_L );
	//printk("------------2lane--edp--out\n");

#endif

	//---------------------------------- //

#ifdef _1080P_eDP_Panel_
	HDMI_WriteI2C_Byte( 0xff, 0x90 );                                                       //register bank
	HDMI_WriteI2C_Byte( 0x4a, 0x33 );                                                       // 148.5MHz
	HDMI_WriteI2C_Byte( 0x4b, 0x33 );
	HDMI_WriteI2C_Byte( 0x4c, 0xd3 );
	HDMI_WriteI2C_Byte( 0x4d, 0x10 );
	//printk("--------1080--edp----\n");
#endif

#ifdef _1366x768_eDP_Panel_
	HDMI_WriteI2C_Byte( 0xff, 0x90 );                                                       //register bank
	//  HDMI_WriteI2C_Byte(0x4a,0x66); // 72MHz
	//  HDMI_WriteI2C_Byte(0x4b,0x66);
	//  HDMI_WriteI2C_Byte(0x4c,0x66);

	HDMI_WriteI2C_Byte( 0x4a, 0xab );                                                       // 76.42MHz
	HDMI_WriteI2C_Byte( 0x4b, 0xaf );
	HDMI_WriteI2C_Byte( 0x4c, 0x6c );

	HDMI_WriteI2C_Byte( 0x4d, 0x10 );
#endif

	HDMI_WriteI2C_Byte( 0xff, 0x81 );                                                       //register bank
	HDMI_WriteI2C_Byte( 0x09, 0x01 );
	HDMI_WriteI2C_Byte( 0x0b, 0x0b );
	HDMI_WriteI2C_Byte( 0x08, 0x13 );

	HDMI_WriteI2C_Byte( 0xff, 0x80 );                                                       //register bank
	HDMI_WriteI2C_Byte( 0x63, 0x7f );
	HDMI_WriteI2C_Byte( 0x63, 0xff );

	//-----------------Main Link---------------------//

	HDMI_WriteI2C_Byte( 0xff, 0x88 );                                                       //register bank
	HDMI_WriteI2C_Byte( 0x00, 0x6a );
	HDMI_WriteI2C_Byte( 0x04, 0xff );

	HDMI_WriteI2C_Byte( 0x05, (u8)( MIPI_Timing[H_tol] / 256 ) );                           //RG_HTOTAL[15:0]
	HDMI_WriteI2C_Byte( 0x06, (u8)( MIPI_Timing[H_tol] % 256 ) );                           //RG_HTOTAL[7:0]
	HDMI_WriteI2C_Byte( 0x07, (u8)( ( MIPI_Timing[H_bp] + MIPI_Timing[H_sync] ) / 256 ) );  //RG_HSTART [15:8]
	HDMI_WriteI2C_Byte( 0x08, (u8)( ( MIPI_Timing[H_bp] + MIPI_Timing[H_sync] ) % 256 ) );  //RG_HSTART[7:0]=110
#ifdef _Test_Pattern_
	HDMI_WriteI2C_Byte( 0x09, (u8)( MIPI_Timing[H_sync] / 256 ) );                          //[7]RG_HSPOL;[6:0]RG_HSYNC_WIDTH[14:8]  0x80-->0x00
	HDMI_WriteI2C_Byte( 0x0a, (u8)( MIPI_Timing[H_sync] % 256 ) );                          //RG_HSYNC_WIDTH[7:0]=60
#else
	HDMI_WriteI2C_Byte( 0x09, 0x00 );                                                       //[7]RG_HSPOL;[6:0]RG_HSYNC_WIDTH[14:8]	 0x80-->0x00
	HDMI_WriteI2C_Byte( 0x0a, 0x00 );                                                       //RG_HSYNC_WIDTH[7:0]=60
#endif
	HDMI_WriteI2C_Byte( 0x0b, (u8)( MIPI_Timing[H_act] / 256 ) );                           //RG_HWIDTH[15:8]
	HDMI_WriteI2C_Byte( 0x0c, (u8)( MIPI_Timing[H_act] % 256 ) );                           //RG_HWIDTH[7:0]
	HDMI_WriteI2C_Byte( 0x0d, (u8)( MIPI_Timing[V_tol] / 256 ) );                           //RG_VTOTAL [15:8]
	HDMI_WriteI2C_Byte( 0x0e, (u8)( MIPI_Timing[V_tol] % 256 ) );                           //RG_VTOTAL[7:0]

	HDMI_WriteI2C_Byte( 0x0f, 0x00 );                                                       //RG_TOP_VTOTAL[15:8] //fiexd
	HDMI_WriteI2C_Byte( 0x10, 0x00 );                                                       //RG_TOP_VTOTAL[7:0]  //fixed

	HDMI_WriteI2C_Byte( 0x11, (u8)( ( MIPI_Timing[V_bp] + MIPI_Timing[V_sync] ) / 256 ) );  //RG_VSTART[15:8]
	HDMI_WriteI2C_Byte( 0x12, (u8)( ( MIPI_Timing[V_bp] + MIPI_Timing[V_sync] ) % 256 ) );  //RG_VSTART[7:0]
#ifdef _Test_Pattern_
	HDMI_WriteI2C_Byte( 0x13, (u8)( MIPI_Timing[V_sync] / 256 ) );                          //RG_VSPOL;RG_VSYNC_WIDTH[14:8]  0x80-->0x00
	HDMI_WriteI2C_Byte( 0x14, (u8)( MIPI_Timing[V_sync] % 256 ) );                          //RG_VSYNC_WIDTH[7:0]
#else
	HDMI_WriteI2C_Byte( 0x13, 0x00 );                                                       //RG_VSPOL;RG_VSYNC_WIDTH[14:8]	 0x80-->0x00
	HDMI_WriteI2C_Byte( 0x14, 0x00 );                                                       //RG_VSYNC_WIDTH[7:0]
#endif
	HDMI_WriteI2C_Byte( 0x15, (u8)( MIPI_Timing[V_act] / 256 ) );                           //RG_VHEIGTH[15:8]
	HDMI_WriteI2C_Byte( 0x16, (u8)( MIPI_Timing[V_act] % 256 ) );                           //RG_VHEIGTH[7:0]

#ifdef _6_Bit_ColorDepth_
	HDMI_WriteI2C_Byte( 0x17, 0x00 );                                                       // LVDS Color Depth:   6 bit: 0x00 ;   8 bit: 0x08
	HDMI_WriteI2C_Byte( 0x18, 0x00 );                                                       // LVDS Color Depth:   6 bit: 0x00 ;   8 bit: 0x20
#endif

#ifdef _8_Bit_ColorDepth_
	HDMI_WriteI2C_Byte( 0x17, 0x08 );                                                       // LVDS Color Depth:   6 bit: 0x00 ;   8 bit: 0x08
	HDMI_WriteI2C_Byte( 0x18, 0x20 );                                                       // LVDS Color Depth:   6 bit: 0x00 ;   8 bit: 0x20
#endif

	HDMI_WriteI2C_Byte( 0x19, 0x00 );
	HDMI_WriteI2C_Byte( 0x1a, 0x80 );
	HDMI_WriteI2C_Byte( 0x1e, 0x30 );
	HDMI_WriteI2C_Byte( 0x21, 0x00 );
#ifdef _Test_Pattern_
	HDMI_WriteI2C_Byte( 0x2c, 0xdf );
#else
	HDMI_WriteI2C_Byte( 0x2c, 0xd0 );
#endif
	HDMI_WriteI2C_Byte( 0x2d, 0x00 );

	HDMI_WriteI2C_Byte( 0x4b, 0xfe );

	HDMI_WriteI2C_Byte( 0x2e, (u8)( ( MIPI_Timing[V_bp] + MIPI_Timing[V_sync] ) % 256 ) );                      //RG_GCM_DE_TOP[6:0]
	HDMI_WriteI2C_Byte( 0x2f, (u8)( ( MIPI_Timing[H_bp] + MIPI_Timing[H_sync] ) / 256 ) );                      //RG_GCM_DE_DLY[11:8]
	HDMI_WriteI2C_Byte( 0x30, (u8)( ( MIPI_Timing[H_bp] + MIPI_Timing[H_sync] ) % 256 ) );                      //RG_GCM_DE_DLY[7:0]
	HDMI_WriteI2C_Byte( 0x31, (u8)( MIPI_Timing[H_act] / 256 ) );                                               //RG_GCM_DE_CNT[11:8]
	HDMI_WriteI2C_Byte( 0x32, (u8)( MIPI_Timing[H_act] % 256 ) );                                               //RG_GCM_DE_CNT[7:0]
	HDMI_WriteI2C_Byte( 0x33, (u8)( MIPI_Timing[V_act] / 256 ) );                                               //RG_GCM_DE_LIN[10:8]
	HDMI_WriteI2C_Byte( 0x34, (u8)( MIPI_Timing[V_act] % 256 ) );                                               //RG_GCM_DE_LIN[7:0]
	HDMI_WriteI2C_Byte( 0x35, (u8)( MIPI_Timing[H_tol] / 256 ) );                                               //RG_GCM_HTOTAL[11:8]
	HDMI_WriteI2C_Byte( 0x36, (u8)( MIPI_Timing[H_tol] % 256 ) );                                               //RG_GCM_HTOTAL[7:0]

#ifdef _Test_Pattern_

	HDMI_WriteI2C_Byte( 0x37, 0x18 + (u8)( MIPI_Timing[V_tol] / 256 ) );                                        //1c:pre-pattern,0c:mipi pattern;RG_GCM_VTOTAL[10:8]
#else

	HDMI_WriteI2C_Byte( 0x37, 0x18 + (u8)( MIPI_Timing[V_tol] / 256 ) );                                        //1c:pre-pattern,0c:mipi pattern;RG_GCM_VTOTAL[10:8]
#endif

	HDMI_WriteI2C_Byte( 0x38, (u8)( MIPI_Timing[V_tol] % 256 ) );                                               //RG_GCM_VTOTAL[7:0]
	HDMI_WriteI2C_Byte( 0x39, 0x00 );                                                                           //reseve
	HDMI_WriteI2C_Byte( 0x3a, ( (u8)( MIPI_Timing[V_sync] % 256 ) ) * 4 + (u8)( MIPI_Timing[H_sync] / 256 ) );  //RG_GCM_VWIDTH[5:0];RG_GCM_HWIDTH[9:8]
	HDMI_WriteI2C_Byte( 0x3b, (u8)( MIPI_Timing[H_sync] % 256 ) );                                              //RG_GCM_HWIDTH[7:0]

	////////////////////Nvid//////////////
	HDMI_WriteI2C_Byte( 0xff, 0x8c );                                                                           //register bank
	HDMI_WriteI2C_Byte( 0x00, 0x00 );
	HDMI_WriteI2C_Byte( 0x01, 0x80 );
	HDMI_WriteI2C_Byte( 0x02, 0x00 );

	//-----------------Training-----------------------------//

	LT8911_AUX_Training( );
	//printk("---------lt8911_aux---\n");
#ifdef _2_Lane_
	if( DPCD0202H != 0x77 )
#endif

#ifdef _1_Lane_
	if( DPCD0202H != 0x07 )
#endif
	LT8911_AUX_Training( );

	//  HDMI_WriteI2C_Byte(0xff,0x88);//register bank
	//  HDMI_WriteI2C_Byte(0x1e,0x30);
	//  HDMI_WriteI2C_Byte(0x4b,0xfe);

	//-----------------------------------------------//

	HDMI_WriteI2C_Byte( 0xff, 0x81 );   //register bank
	HDMI_WriteI2C_Byte( 0x32, 0x40 );   // 0x40
	HDMI_WriteI2C_Byte( 0x27, 0x80 );
	HDMI_WriteI2C_Byte( 0x28, 0xa4 );
	HDMI_WriteI2C_Byte( 0x29, 0x66 );   // 0xd2
	HDMI_WriteI2C_Byte( 0x2a, 0x04 );
	HDMI_WriteI2C_Byte( 0x2b, 0x7e );//0x7f );   // 0x7e
	HDMI_WriteI2C_Byte( 0x2c, 0x02 );
	HDMI_WriteI2C_Byte( 0x2d, 0x02 );//0x7c );   // 0x02
	HDMI_WriteI2C_Byte( 0x2e, 0xaa );//0x00 );   // 0xaa
	HDMI_WriteI2C_Byte( 0x2f, 0x02 );
	HDMI_WriteI2C_Byte( 0x30, 0xaa );
	HDMI_WriteI2C_Byte( 0x31, 0x4b );
	HDMI_WriteI2C_Byte( 0x32, 0x43 );   // 0x43
	HDMI_WriteI2C_Byte( 0x33, 0x20 );   // 0x20
	HDMI_WriteI2C_Byte( 0x34, 0x01 );   // MIPI Port B power down
	HDMI_WriteI2C_Byte( 0x35, 0x80 );
	HDMI_WriteI2C_Byte( 0x36, 0xa4 );
	HDMI_WriteI2C_Byte( 0x37, 0xd2 );
	HDMI_WriteI2C_Byte( 0x38, 0x00 );
	HDMI_WriteI2C_Byte( 0x39, 0x36 );
	HDMI_WriteI2C_Byte( 0x3a, 0x00 );

	//--------------------------------------------//

	HDMI_WriteI2C_Byte( 0xff, 0x90 );               //register bank
	HDMI_WriteI2C_Byte( 0x01, 0x01 );
	HDMI_WriteI2C_Byte( 0x02, 0x08 );               // 0x04
	HDMI_WriteI2C_Byte( 0x03, 0x04 );
	HDMI_WriteI2C_Byte( 0x04, 0xc8 );
	HDMI_WriteI2C_Byte( 0x05, 0x00 );
	HDMI_WriteI2C_Byte( 0x06, 0x0b );
	HDMI_WriteI2C_Byte( 0x0b, _MIPI_Lane_ % 4 );    // 00:4 Lane;01:1 Lane;02:2 Lane;03:3 Lane
	HDMI_WriteI2C_Byte( 0x0c, 0x00 );               // 3210
	HDMI_WriteI2C_Byte( 0x10, 0x03 );
	HDMI_WriteI2C_Byte( 0x11, 0x03 );

	HDMI_WriteI2C_Byte( 0x12, (u8)( MIPI_Timing[H_sync] % 256 ) );
	HDMI_WriteI2C_Byte( 0x13, (u8)( MIPI_Timing[V_sync] % 256 ) );
	HDMI_WriteI2C_Byte( 0x14, (u8)( MIPI_Timing[H_act] % 256 ) );
	HDMI_WriteI2C_Byte( 0x15, (u8)( MIPI_Timing[H_act] / 256 ) );
	HDMI_WriteI2C_Byte( 0x16, (u8)( MIPI_Timing[H_act] % 256 ) );
	HDMI_WriteI2C_Byte( 0x17, (u8)( MIPI_Timing[H_act] / 256 ) );

	HDMI_WriteI2C_Byte( 0x18, 0x00 );
	HDMI_WriteI2C_Byte( 0x19, 0x01 );
	HDMI_WriteI2C_Byte( 0x1a, 0x17 );
	HDMI_WriteI2C_Byte( 0x2b, 0x0b );
	HDMI_WriteI2C_Byte( 0x2c, 0x0c );

	HDMI_WriteI2C_Byte( 0x31, (u8)( MIPI_Timing[H_tol] % 256 ) );
	HDMI_WriteI2C_Byte( 0x32, (u8)( MIPI_Timing[H_tol] / 256 ) );
	HDMI_WriteI2C_Byte( 0x33, (u8)( MIPI_Timing[V_tol] % 256 ) );
	HDMI_WriteI2C_Byte( 0x34, (u8)( MIPI_Timing[V_tol] / 256 ) );
	HDMI_WriteI2C_Byte( 0x35, (u8)( MIPI_Timing[V_bp] % 256 ) );
	HDMI_WriteI2C_Byte( 0x36, (u8)( MIPI_Timing[V_bp] / 256 ) );
	HDMI_WriteI2C_Byte( 0x37, (u8)( ( MIPI_Timing[V_tol] - MIPI_Timing[V_act] - MIPI_Timing[V_bp] - MIPI_Timing[V_sync] ) % 256 ) );
	HDMI_WriteI2C_Byte( 0x38, (u8)( ( MIPI_Timing[V_tol] - MIPI_Timing[V_act] - MIPI_Timing[V_bp] - MIPI_Timing[V_sync] ) / 256 ) );
	HDMI_WriteI2C_Byte( 0x39, (u8)( MIPI_Timing[H_bp] % 256 ) );
	HDMI_WriteI2C_Byte( 0x3a, (u8)( MIPI_Timing[H_bp] / 256 ) );
	HDMI_WriteI2C_Byte( 0x3b, (u8)( ( MIPI_Timing[H_tol] - MIPI_Timing[H_act] - MIPI_Timing[H_bp] - MIPI_Timing[H_sync] ) % 256 ) );
	HDMI_WriteI2C_Byte( 0x3c, (u8)( ( MIPI_Timing[H_tol] - MIPI_Timing[H_act] - MIPI_Timing[H_bp] - MIPI_Timing[H_sync] ) / 256 ) );
	LT_INFO("---------11111--\n");
	HDMI_WriteI2C_Byte( 0x1b, 0x5e );
	HDMI_WriteI2C_Byte( 0x1c, 0x01 );
	HDMI_WriteI2C_Byte( 0x1d, 0x2c );
	HDMI_WriteI2C_Byte( 0x1e, 0x01 );
	HDMI_WriteI2C_Byte( 0x1f, 0xfa );
	HDMI_WriteI2C_Byte( 0x20, 0x00 );
	HDMI_WriteI2C_Byte( 0x21, 0xc8 );
	HDMI_WriteI2C_Byte( 0x22, 0x00 );
	HDMI_WriteI2C_Byte( 0x23, 0x5e );
	HDMI_WriteI2C_Byte( 0x24, 0x01 );
	HDMI_WriteI2C_Byte( 0x25, 0x2c );
	HDMI_WriteI2C_Byte( 0x26, 0x01 );
	HDMI_WriteI2C_Byte( 0x27, 0xfa );
	HDMI_WriteI2C_Byte( 0x28, 0x00 );
	HDMI_WriteI2C_Byte( 0x29, 0xc8 );
	HDMI_WriteI2C_Byte( 0x2a, 0x00 );
	HDMI_WriteI2C_Byte( 0x3d, 0x64 );   //
	HDMI_WriteI2C_Byte( 0x3f, 0x00 );   //
	//LT_INFO("---------22222--\n");
	HDMI_WriteI2C_Byte( 0x40, 0x04 );
	HDMI_WriteI2C_Byte( 0x41, 0x00 );
	HDMI_WriteI2C_Byte( 0x42, 0x59 );
	HDMI_WriteI2C_Byte( 0x43, 0x00 );
	HDMI_WriteI2C_Byte( 0x44, 0xf2 );
	HDMI_WriteI2C_Byte( 0x45, 0x06 );
	HDMI_WriteI2C_Byte( 0x46, 0x00 );
	HDMI_WriteI2C_Byte( 0x47, 0x72 );
	HDMI_WriteI2C_Byte( 0x48, 0x45 );
	HDMI_WriteI2C_Byte( 0x49, 0x00 );
	LT_INFO("---------33333--\n");
	HDMI_WriteI2C_Byte( 0x60, 0x08 );
	HDMI_WriteI2C_Byte( 0x61, 0x00 );
	HDMI_WriteI2C_Byte( 0x62, 0xb2 );
	HDMI_WriteI2C_Byte( 0x63, 0x00 );
	HDMI_WriteI2C_Byte( 0x64, 0xe4 );
	HDMI_WriteI2C_Byte( 0x65, 0x0d );
	HDMI_WriteI2C_Byte( 0x66, 0x00 );
	HDMI_WriteI2C_Byte( 0x67, 0xe4 );
	HDMI_WriteI2C_Byte( 0x68, 0x8a );
	HDMI_WriteI2C_Byte( 0x69, 0x00 );
	HDMI_WriteI2C_Byte( 0x6a, 0x0b );   //
	HDMI_WriteI2C_Byte( 0x1a, 0x4f );   //
	HDMI_WriteI2C_Byte( 0x6b, 0x04 );   //
	//LT_INFO("[yujian]---------44444--\n");
#ifdef _Test_Pattern_

	// 前面已经设置过了，这里不需设置。
	//  HDMI_WriteI2C_Byte(0x4d,0x10);

#else
	HDMI_WriteI2C_Byte( 0xff, 0x90 );   //register bank
	HDMI_WriteI2C_Byte( 0x4d, 0x00 );
#endif
	//LT_INFO("---------555555--\n");
	//---------------------------------------//
	HDMI_WriteI2C_Byte( 0xff, 0x80 );   //register bank
	HDMI_WriteI2C_Byte( 0x62, 0x3f );

	//HDMI_WriteI2C_Byte( 0x63, 0x3f );
	//HDMI_WriteI2C_Byte( 0x63, 0xbf );

	//HDMI_WriteI2C_Byte( 0x60, 0xde );
	//HDMI_WriteI2C_Byte( 0x60, 0xff );

	//------------------------------------------//
	//LT_INFO("---------66666--\n");

	/**********************
	   52 ff 90 00
	   52 75 01 ff //bit[23:16]
	   52 76 01 ff //bit[15:8]
	   52 77 01 ff //bit[7:0]
	 ***********************/

	//*
/*	
#ifndef _Test_Pattern_
	// 连续读5次0x9076寄存器，如果值相差小于3，说明DDS已经调稳.
	Count = 0;
	while( Count < 3 )
	{
		Count++;
		HDMI_WriteI2C_Byte( 0xff, 0x90 );
		mdelay( 20 );
		Pre_Temp = HDMI_ReadI2C_Byte( 0x76 );
		mdelay( 20 );
		Cur_Temp = HDMI_ReadI2C_Byte( 0x76 );
		//LT_INFO("---------77777--\n");
		D_value = ( Cur_Temp >= Pre_Temp ) ? ( Cur_Temp - Pre_Temp ) : ( Pre_Temp - Cur_Temp );

		// 连续读两次0x9076寄存器，如果值相差大于8，复位一下0x8063/0x8060寄存器。
		while( D_value >= 0x08 )
		{
			Count = 0x00;
			HDMI_WriteI2C_Byte( 0xff, 0x80 ); //register bank
			HDMI_WriteI2C_Byte( 0x63, 0x7f );//0x3f );
			HDMI_WriteI2C_Byte( 0x63, 0xff );//0xbf );

			HDMI_WriteI2C_Byte( 0x60, 0xde );
			HDMI_WriteI2C_Byte( 0x60, 0xff );
			
			//HDMI_WriteI2C_Byte( 0xff, 0x81 ); //register bank
			//HDMI_WriteI2C_Byte( 0x32, 0x40 );
			//mdelay( 10 );
			//HDMI_WriteI2C_Byte( 0x32, 0x43 );

			mdelay( 40 );
			//LT_INFO("---------88888--\n");
			HDMI_WriteI2C_Byte( 0xff, 0x90 ); //register bank

			Pre_Temp = HDMI_ReadI2C_Byte( 0x76 );
			mdelay( 20 );
			Cur_Temp = HDMI_ReadI2C_Byte( 0x76 );

			D_value = ( Cur_Temp >= Pre_Temp ) ? ( Cur_Temp - Pre_Temp ) : ( Pre_Temp - Cur_Temp );
		}
		
		//如果D_value小于8直接返回
		if (1 == Count)
		{
			break;
		}
	}
#endif
*/
	HDMI_WriteI2C_Byte( 0xff, 0x80 ); //register bank
	HDMI_WriteI2C_Byte( 0x63, 0x7f );//0x3f );
	HDMI_WriteI2C_Byte( 0x63, 0xff );//0xbf );

	HDMI_WriteI2C_Byte( 0x60, 0xde );
	HDMI_WriteI2C_Byte( 0x60, 0xff );
	
	//HDMI_WriteI2C_Byte( 0xff, 0x81 );
	//HDMI_WriteI2C_Byte( 0x32, 0x40 );
	//mdelay( 10 );
	//HDMI_WriteI2C_Byte( 0x32, 0x43 );
	
	//LT_INFO("---------99999--\n");
#ifndef _Test_Pattern_ 

	//mdelay( 100 );
	HDMI_WriteI2C_Byte( 0xff, 0x88 );
	HDMI_WriteI2C_Byte( 0x37, 0x08 + (u8)( MIPI_Timing[V_tol] / 256 ) );
#endif

	mdelay( 10 );
	//LT_INFO("---------AAAAA--\n");
	HDMI_WriteI2C_Byte( 0xff, 0x80 );
	HDMI_WriteI2C_Byte( 0x62, 0xbf );

	//------------------------------------------//
	//  For test
	//  HDMI_WriteI2C_Byte(0xff,0x90);//register bank
	//  HDMI_WriteI2C_Byte(0x07,0xc0);

	//  HDMI_WriteI2C_Byte(0xff,0x80);//register bank
	//  HDMI_WriteI2C_Byte(0x94,0x00);
	//  HDMI_WriteI2C_Byte(0x95,0x00);

	//  HDMI_WriteI2C_Byte(0xff,0x81);//register bank
	//  HDMI_WriteI2C_Byte(0x3f,0x02);
	//  HDMI_WriteI2C_Byte(0x3e,0xff);
	//  HDMI_WriteI2C_Byte(0x3d,0x03);
	//  HDMI_WriteI2C_Byte(0x2b,0x7f);
	//----------------------------------------//

	HDMI_WriteI2C_Byte( 0xff, 0x00 ); //register bank
}

void LT8911EXB_ChipID( void )                                                      // read Chip ID
{
	LT_INFO( "\r\n###################start#####################" );
	HDMI_WriteI2C_Byte( 0xff, 0x81 );                                           //register bank
	HDMI_WriteI2C_Byte( 0x08, 0x7f );
	//LT_INFO( "\r\nLT8911EXB chip ID:", HDMI_ReadI2C_Byte( 0x00 ) );    // 0x17
	//LT_INFO( ", ", HDMI_ReadI2C_Byte( 0x01 ) );                        // 0x05
	//LT_INFO( ", ", HDMI_ReadI2C_Byte( 0x02 ) );                        // 0xE0
}

void LT8911EXB_MIPI_Video_Timing( void )                               // ( struct video_timing *video_format )
{
	HDMI_WriteI2C_Byte( 0xff, 0xd0 );
	HDMI_WriteI2C_Byte( 0x0d, (u8)( EXB_MIPI_Timing[vtotal] / 256 ) );
	HDMI_WriteI2C_Byte( 0x0e, (u8)( EXB_MIPI_Timing[vtotal] % 256 ) );  //vtotal
	HDMI_WriteI2C_Byte( 0x0f, (u8)( EXB_MIPI_Timing[vact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x10, (u8)( EXB_MIPI_Timing[vact] % 256 ) );    //vactive
	HDMI_WriteI2C_Byte( 0x11, (u8)( EXB_MIPI_Timing[htotal] / 256 ) );
	HDMI_WriteI2C_Byte( 0x12, (u8)( EXB_MIPI_Timing[htotal] % 256 ) );  //htotal
	HDMI_WriteI2C_Byte( 0x13, (u8)( EXB_MIPI_Timing[hact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x14, (u8)( EXB_MIPI_Timing[hact] % 256 ) );    //hactive
	HDMI_WriteI2C_Byte( 0x15, (u8)( EXB_MIPI_Timing[vs] % 256 ) );      //vsa
	HDMI_WriteI2C_Byte( 0x16, (u8)( EXB_MIPI_Timing[hs] % 256 ) );      //hsa
	HDMI_WriteI2C_Byte( 0x17, (u8)( EXB_MIPI_Timing[vfp] / 256 ) );
	HDMI_WriteI2C_Byte( 0x18, (u8)( EXB_MIPI_Timing[vfp] % 256 ) );     //vfp
	HDMI_WriteI2C_Byte( 0x19, (u8)( EXB_MIPI_Timing[hfp] / 256 ) );
	HDMI_WriteI2C_Byte( 0x1a, (u8)( EXB_MIPI_Timing[hfp] % 256 ) );     //hfp
}

void LT8911EXB_eDP_Video_cfg( void )                                   // ( struct video_timing *video_format )
{
	HDMI_WriteI2C_Byte( 0xff, 0xa8 );
	HDMI_WriteI2C_Byte( 0x2d, 0x88 );                               // MSA from register

#ifdef _Msa_Active_Only_
	HDMI_WriteI2C_Byte( 0x05, 0x00 );
	HDMI_WriteI2C_Byte( 0x06, 0x00 );                               //htotal
	HDMI_WriteI2C_Byte( 0x07, 0x00 );
	HDMI_WriteI2C_Byte( 0x08, 0x00 );                               //h_start
	HDMI_WriteI2C_Byte( 0x09, 0x00 );
	HDMI_WriteI2C_Byte( 0x0a, 0x00 );                               //hsa
	HDMI_WriteI2C_Byte( 0x0b, (u8)( EXB_MIPI_Timing[hact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x0c, (u8)( EXB_MIPI_Timing[hact] % 256 ) );    //hactive
	HDMI_WriteI2C_Byte( 0x0d, 0x00 );
	HDMI_WriteI2C_Byte( 0x0e, 0x00 );                               //vtotal
	HDMI_WriteI2C_Byte( 0x11, 0x00 );
	HDMI_WriteI2C_Byte( 0x12, 0x00 );
	HDMI_WriteI2C_Byte( 0x14, 0x00 );
	HDMI_WriteI2C_Byte( 0x15, (u8)( EXB_MIPI_Timing[vact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x16, (u8)( EXB_MIPI_Timing[vact] % 256 ) );    //vactive

#else

	HDMI_WriteI2C_Byte( 0x05, (u8)( EXB_MIPI_Timing[htotal] / 256 ) );
	HDMI_WriteI2C_Byte( 0x06, (u8)( EXB_MIPI_Timing[htotal] % 256 ) );
	HDMI_WriteI2C_Byte( 0x07, (u8)( ( EXB_MIPI_Timing[hs] + EXB_MIPI_Timing[hbp] ) / 256 ) );
	HDMI_WriteI2C_Byte( 0x08, (u8)( ( EXB_MIPI_Timing[hs] + EXB_MIPI_Timing[hbp] ) % 256 ) );
	HDMI_WriteI2C_Byte( 0x09, (u8)( EXB_MIPI_Timing[hs] / 256 ) );
	HDMI_WriteI2C_Byte( 0x0a, (u8)( EXB_MIPI_Timing[hs] % 256 ) );
	HDMI_WriteI2C_Byte( 0x0b, (u8)( EXB_MIPI_Timing[hact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x0c, (u8)( EXB_MIPI_Timing[hact] % 256 ) );
	HDMI_WriteI2C_Byte( 0x0d, (u8)( EXB_MIPI_Timing[vtotal] / 256 ) );
	HDMI_WriteI2C_Byte( 0x0e, (u8)( EXB_MIPI_Timing[vtotal] % 256 ) );
	HDMI_WriteI2C_Byte( 0x11, (u8)( ( EXB_MIPI_Timing[vs] + EXB_MIPI_Timing[vbp] ) / 256 ) );
	HDMI_WriteI2C_Byte( 0x12, (u8)( ( EXB_MIPI_Timing[vs] + EXB_MIPI_Timing[vbp] ) % 256 ) );
	HDMI_WriteI2C_Byte( 0x14, (u8)( EXB_MIPI_Timing[vs] % 256 ) );
	HDMI_WriteI2C_Byte( 0x15, (u8)( EXB_MIPI_Timing[vact] / 256 ) );
	HDMI_WriteI2C_Byte( 0x16, (u8)( EXB_MIPI_Timing[vact] % 256 ) );
#endif
}

void LT8911EXB_InterruptEnable( void )
{
	HDMI_WriteI2C_Byte( 0xff, 0x85 );
	HDMI_WriteI2C_Byte( 0x08, 0x3f );   //fm clr

	HDMI_WriteI2C_Byte( 0x65, 0x7f );   //vid chk hact clr
	HDMI_WriteI2C_Byte( 0x07, 0x7f );   //vid chk clr
}

void LT8911EXT_init( void )
{
	u8	i;
	u8	pcr_pll_postdiv;
	u8	pcr_m;
	u16 Temp16;

	/* init */
	HDMI_WriteI2C_Byte( 0xff, 0x81 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x08, 0x7f );   //i2c over aux issue
	HDMI_WriteI2C_Byte( 0x49, 0xff );   //enable 0x87xx

	HDMI_WriteI2C_Byte( 0xff, 0x87 );
	HDMI_WriteI2C_Byte( 0x19, 0x31 );
	HDMI_WriteI2C_Byte( 0x1a, 0x32 );   // sync m --27M; 0x36--25M
	HDMI_WriteI2C_Byte( 0x1b, 0x00 );   // sync_k [7:0]
	HDMI_WriteI2C_Byte( 0x1c, 0x00 );   // sync_k [13:8]

	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x5a, 0x0e );   //GPIO test output

	//for power consumption//
	HDMI_WriteI2C_Byte( 0xff, 0x81 );
	HDMI_WriteI2C_Byte( 0x05, 0x06 );
	HDMI_WriteI2C_Byte( 0x43, 0x00 );
	HDMI_WriteI2C_Byte( 0x44, 0x1f );
	HDMI_WriteI2C_Byte( 0x45, 0xf7 );
	HDMI_WriteI2C_Byte( 0x46, 0xf6 );
	HDMI_WriteI2C_Byte( 0x49, 0x7f );

	HDMI_WriteI2C_Byte( 0xff, 0x82 );
	HDMI_WriteI2C_Byte( 0x12, 0x33 );

	/* mipi Rx analog */
	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x32, 0x51 );
	HDMI_WriteI2C_Byte( 0x35, 0x22 );   //EQ current
	HDMI_WriteI2C_Byte( 0x3a, 0x77 );   //EQ 12.5db
	HDMI_WriteI2C_Byte( 0x3b, 0x77 );   //EQ 12.5db

	HDMI_WriteI2C_Byte( 0x4c, 0x0c );
	HDMI_WriteI2C_Byte( 0x4d, 0x00 );

	/* dessc_pcr  pll analog */
	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x6a, 0x43 );
	HDMI_WriteI2C_Byte( 0x6b, PCR_PLL_PREDIV );

	Temp16 = EXB_MIPI_Timing[pclk_10khz];

	if( EXB_MIPI_Timing[pclk_10khz] < 8800 )
	{
		HDMI_WriteI2C_Byte( 0x6e, 0x82 );   //0x44:pre-div = 2 ,pixel_clk=44~ 88MHz
		pcr_pll_postdiv = 0x08;
	}else
	{
		HDMI_WriteI2C_Byte( 0x6e, 0x81 );   //0x40:pre-div = 1, pixel_clk =88~176MHz
		pcr_pll_postdiv = 0x04;
	}
	pcr_m = (u8)( Temp16 * pcr_pll_postdiv / 25 / 100 );

	/* dessc pll digital */
	HDMI_WriteI2C_Byte( 0xff, 0x85 );       // Change Reg bank
	HDMI_WriteI2C_Byte( 0xa9, 0x31 );
	HDMI_WriteI2C_Byte( 0xaa, 0x17 );
	HDMI_WriteI2C_Byte( 0xab, 0xba );
	HDMI_WriteI2C_Byte( 0xac, 0xe1 );
	HDMI_WriteI2C_Byte( 0xad, 0x47 );
	HDMI_WriteI2C_Byte( 0xae, 0x01 );
	HDMI_WriteI2C_Byte( 0xae, 0x11 );

	/* Digital Top */
	HDMI_WriteI2C_Byte( 0xff, 0x85 );               // Change Reg bank
	HDMI_WriteI2C_Byte( 0xc0, 0x01 );               //select mipi Rx
#ifdef _6bit_
	HDMI_WriteI2C_Byte( 0xb0, 0xd0 );               //enable dither
#else
	HDMI_WriteI2C_Byte( 0xb0, 0x00 );               // disable dither
#endif

	/* mipi Rx Digital */
	HDMI_WriteI2C_Byte( 0xff, 0xd0 );               // Change Reg bank
	HDMI_WriteI2C_Byte( 0x00, _MIPI_Lane_ % 4 );    // 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane
	HDMI_WriteI2C_Byte( 0x02, 0x08 );               //settle
	HDMI_WriteI2C_Byte( 0x08, 0x00 );
//	HDMI_WriteI2C_Byte( 0x0a, 0x12 );               //pcr mode

	HDMI_WriteI2C_Byte( 0x0c, 0x80 );               //fifo position
	HDMI_WriteI2C_Byte( 0x1c, 0x40 );               //fifo position
	HDMI_WriteI2C_Byte( 0x24, 0x31 );               //pcr mode( de hs vs)

	HDMI_WriteI2C_Byte( 0x31, 0x0a );

	/*stage2 hs mode*/
	HDMI_WriteI2C_Byte( 0x23, 0x80 );               //CAPITAL_SND

	/*stage2 de mode*/
	HDMI_WriteI2C_Byte( 0x0a, 0x02 );               //de adjust pre line
	HDMI_WriteI2C_Byte( 0x38, 0x02 );               //de_threshold 1
	HDMI_WriteI2C_Byte( 0x39, 0x04 );               //de_threshold 2
	HDMI_WriteI2C_Byte( 0x3a, 0x08 );               //de_threshold 3
	HDMI_WriteI2C_Byte( 0x3b, 0x10 );               //de_threshold 4

	HDMI_WriteI2C_Byte( 0x3f, 0x04 );               //de_step 1
	HDMI_WriteI2C_Byte( 0x40, 0x08 );               //de_step 2
	HDMI_WriteI2C_Byte( 0x41, 0x10 );               //de_step 3
	HDMI_WriteI2C_Byte( 0x42, 0x60 );               //de_step 4

	HDMI_WriteI2C_Byte( 0x2b, 0xa0 );               //stable out

#ifdef _Test_Pattern_
	HDMI_WriteI2C_Byte( 0x26, ( pcr_m | 0x80 ) );
#else
	HDMI_WriteI2C_Byte( 0x26, pcr_m );
#endif
	HDMI_WriteI2C_Byte( 0x27, 0xf6 );   //PCR reset
	HDMI_WriteI2C_Byte( 0x28, 0x64 );


	LT8911EXB_MIPI_Video_Timing( );        //defualt setting is 1080P

	HDMI_WriteI2C_Byte( 0xff, 0x81 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x03, 0x7b );   //PCR reset
	HDMI_WriteI2C_Byte( 0x03, 0xff );

	/* Txpll 2.7G*/
	HDMI_WriteI2C_Byte( 0xff, 0x87 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x19, 0x31 );

	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x02, 0x42 );
	HDMI_WriteI2C_Byte( 0x03, 0x00 );
	HDMI_WriteI2C_Byte( 0x03, 0x01 );

	HDMI_WriteI2C_Byte( 0xff, 0x81 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x09, 0xfc );
	HDMI_WriteI2C_Byte( 0x09, 0xfd );

	HDMI_WriteI2C_Byte( 0xff, 0x87 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x0c, 0x11 );
	HDMI_WriteI2C_Byte( 0xff, 0x87 );
 	HDMI_WriteI2C_Byte( 0x13, 0x83 );
 	HDMI_WriteI2C_Byte( 0x14, 0x41 );
 	HDMI_WriteI2C_Byte( 0x16, 0x0a );
 	HDMI_WriteI2C_Byte( 0x18, 0x0a );
 	HDMI_WriteI2C_Byte( 0x19, 0x33 );

	for( i = 0; i < 5; i++ )            //Check Tx PLL
	{
		mdelay( 5 );
		if( HDMI_ReadI2C_Byte( 0x37 ) & 0x02 )
		{
			LT_INFO( "\r\nLT8911 tx pll locked" );
			break;
		}else
		{
			LT_INFO( "\r\nLT8911 tx pll unlocked" );
			HDMI_WriteI2C_Byte( 0xff, 0x81 );
			HDMI_WriteI2C_Byte( 0x09, 0xfc );
			HDMI_WriteI2C_Byte( 0x09, 0xfd );

			HDMI_WriteI2C_Byte( 0xff, 0x87 );
			HDMI_WriteI2C_Byte( 0x0c, 0x10 );
			HDMI_WriteI2C_Byte( 0x0c, 0x11 );
		}
	}
	/* tx phy */
	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x11, 0x00 );
	HDMI_WriteI2C_Byte( 0x13, 0x10 );
	HDMI_WriteI2C_Byte( 0x14, 0x0c );
	HDMI_WriteI2C_Byte( 0x14, 0x08 );
	HDMI_WriteI2C_Byte( 0x13, 0x20 );

	HDMI_WriteI2C_Byte( 0xff, 0x82 );   // Change Reg bank
	HDMI_WriteI2C_Byte( 0x0e, 0x25 );
//	HDMI_WriteI2C_Byte( 0x12, 0xff );
//	HDMI_WriteI2C_Byte( 0xff, 0x80 );
//	HDMI_WriteI2C_Byte( 0x40, 0x22 );

	/*eDP Tx Digital */
	HDMI_WriteI2C_Byte( 0xff, 0xa8 );   // Change Reg bank
#ifdef _Test_Pattern_
	HDMI_WriteI2C_Byte( 0x24, 0x50 );   // bit2 ~ bit 0 : test panttern image mode
	HDMI_WriteI2C_Byte( 0x25, 0x70 );   // bit6 ~ bit 4 : test Pattern color
	HDMI_WriteI2C_Byte( 0x27, 0x50 );   //0x50:Pattern; 0x10:mipi video
#else
	HDMI_WriteI2C_Byte( 0x27, 0x10 );   //0x50:Pattern; 0x10:mipi video
#endif

#ifdef _6bit_
	HDMI_WriteI2C_Byte( 0x17, 0x00 );
	HDMI_WriteI2C_Byte( 0x18, 0x00 );
#else
	// _8bit_
	HDMI_WriteI2C_Byte( 0x17, 0x10 );
	HDMI_WriteI2C_Byte( 0x18, 0x20 );
#endif

	HDMI_WriteI2C_Byte( 0xff, 0xa0 ); // Change Reg bank
	HDMI_WriteI2C_Byte( 0x00, 0x08 );
	HDMI_WriteI2C_Byte( 0x01, 0x00 );
}

void LT8911EXB_video_check( void )
{
	u32 reg = 0x00;
	/* mipi byte clk check*/
	HDMI_WriteI2C_Byte( 0xff, 0x85 );       // Change Reg bank
	HDMI_WriteI2C_Byte( 0x1d, 0x00 );       //FM select byte clk
	HDMI_WriteI2C_Byte( 0x40, 0xf7 );
	HDMI_WriteI2C_Byte( 0x41, 0x30 );

	//#ifdef _eDP_scramble_
	if( ScrambleMode )
	{
		HDMI_WriteI2C_Byte( 0xa1, 0x82 );   //eDP scramble mode;
	}
	//#else
	else
	{
		HDMI_WriteI2C_Byte( 0xa1, 0x02 );   // DP scramble mode;
	}
	//#endif

//	HDMI_WriteI2C_Byte( 0x17, 0xf0 ); // 0xf0:Close scramble; 0xD0 : Open scramble

	HDMI_WriteI2C_Byte( 0xff, 0x81 );
	HDMI_WriteI2C_Byte( 0x09, 0x7d );
	HDMI_WriteI2C_Byte( 0x09, 0xfd );

	HDMI_WriteI2C_Byte( 0xff, 0x85 );
	mdelay( 200 );
	if( HDMI_ReadI2C_Byte( 0x50 ) == 0x03 )
	{
		reg	   = HDMI_ReadI2C_Byte( 0x4d );
		reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x4e );
		reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x4f );

		LT_INFO( "\r\nvideo check: mipi byteclk = %d ", reg ); // mipi byteclk = reg * 1000
	}else
	{
		LT_INFO( "\r\nvideo check: mipi clk unstable" );
	}

	/* mipi vtotal check*/
	reg	   = HDMI_ReadI2C_Byte( 0x76 );
	reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x77 );

	LT_INFO( "\r\nvideo check: Vtotal =  %d", reg );

	/* mipi word count check*/
	HDMI_WriteI2C_Byte( 0xff, 0xd0 );
	reg	   = HDMI_ReadI2C_Byte( 0x82 );
	reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x83 );
	reg	   = reg / 3;

	LT_INFO( "\r\nvideo check: Hact(word counter) = %d ", reg );

	/* mipi Vact check*/
	reg	   = HDMI_ReadI2C_Byte( 0x85 );
	reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x86 );

	LT_INFO( "\r\nvideo check: Vact = %d ", reg );
}

void LT8911EXB_TxSwingPreSet( void )
{
	HDMI_WriteI2C_Byte( 0xFF, 0x82 );
	HDMI_WriteI2C_Byte( 0x22, 0x82 );   //lane 0 tap0
	HDMI_WriteI2C_Byte( 0x23, 0x00 );
	HDMI_WriteI2C_Byte( 0x24, 0x80 );   //lane 0 tap1
	HDMI_WriteI2C_Byte( 0x25, 0x00 );
	HDMI_WriteI2C_Byte( 0x26, 0x82 );   //lane 1 tap0
	HDMI_WriteI2C_Byte( 0x27, 0x00 );
	HDMI_WriteI2C_Byte( 0x28, 0x80 );   //lane 1 tap1
	HDMI_WriteI2C_Byte( 0x29, 0x00 );
}

void DpcdEXBWrite( u32 Address, u8 Data )
{
	/***************************
	   注意大小端的问题!
	   这里默认是大端模式
	   Big-Endian
	 ****************************/
	u8	AddressH   = 0x0f & ( Address >> 16 );
	u8	AddressM   = 0xff & ( Address >> 8 );
	u8	AddressL   = 0xff & Address;

	u8	reg;

	HDMI_WriteI2C_Byte( 0xff, 0xa6 );
	HDMI_WriteI2C_Byte( 0x2b, ( 0x80 | AddressH ) );    //CMD
	HDMI_WriteI2C_Byte( 0x2b, AddressM );               //addr[15:8]
	HDMI_WriteI2C_Byte( 0x2b, AddressL );               //addr[7:0]
	HDMI_WriteI2C_Byte( 0x2b, 0x00 );                   //data lenth
	HDMI_WriteI2C_Byte( 0x2b, Data );                   //data
	HDMI_WriteI2C_Byte( 0x2c, 0x00 );                   //start Aux

	mdelay( 20 );                                     //more than 10ms
	reg = HDMI_ReadI2C_Byte( 0x25 );

	if( ( reg & 0x0f ) == 0x0c )
	{
		return;
	}
}

u8 DpcdEXBRead( u32 Address )
{
	/***************************
	   注意大小端的问题!
	   这里默认是大端模式
	   Big-Endian
	 ****************************/

	u8	DpcdValue  = 0x00;
	u8	AddressH   = 0x0f & ( Address >> 16 );
	u8	AddressM   = 0xff & ( Address >> 8 );
	u8	AddressL   = 0xff & Address;
	u8	reg;

	HDMI_WriteI2C_Byte( 0xff, 0xac );
	HDMI_WriteI2C_Byte( 0x00, 0x20 );                   //Soft Link train
	HDMI_WriteI2C_Byte( 0xff, 0xa6 );
	HDMI_WriteI2C_Byte( 0x2a, 0x01 );

	HDMI_WriteI2C_Byte( 0xff, 0xa6 );
	HDMI_WriteI2C_Byte( 0x2b, ( 0x90 | AddressH ) );    //CMD
	HDMI_WriteI2C_Byte( 0x2b, AddressM );               //addr[15:8]
	HDMI_WriteI2C_Byte( 0x2b, AddressL );               //addr[7:0]
	HDMI_WriteI2C_Byte( 0x2b, 0x00 );                   //data lenth
	HDMI_WriteI2C_Byte( 0x2c, 0x00 );                   //start Aux read edid

	mdelay( 50 );                                     //more than 10ms
	reg = HDMI_ReadI2C_Byte( 0x25 );
	if( ( reg & 0x0f ) == 0x0c )
	{
		if( HDMI_ReadI2C_Byte( 0x39 ) == 0x22 )
		{
			HDMI_ReadI2C_Byte( 0x2b );
			DpcdValue = HDMI_ReadI2C_Byte( 0x2b );
		}
		/*
		else
		{
			//	goto no_reply;
			DpcdValue = 0xff;
			return DpcdValue;
		}*/
	}

	return DpcdValue;
}

void LT8911EXB_link_train( void )
{
	HDMI_WriteI2C_Byte( 0xff, 0x85 );

//#ifdef _eDP_scramble_
	if( ScrambleMode )
	{
		HDMI_WriteI2C_Byte( 0xa1, 0x82 );   // eDP scramble mode;

		/* Aux operater init */
		HDMI_WriteI2C_Byte( 0xff, 0xac );
		HDMI_WriteI2C_Byte( 0x00, 0x20 );   //Soft Link train
		HDMI_WriteI2C_Byte( 0xff, 0xa6 );
		HDMI_WriteI2C_Byte( 0x2a, 0x01 );

		DpcdEXBWrite( 0x010a, 0x01 );
		mdelay( 10 );
		DpcdEXBWrite( 0x0102, 0x00 );
		mdelay( 10 );
		DpcdEXBWrite( 0x010a, 0x01 );

		mdelay( 200 );
	}
//#else
	else
	{
		HDMI_WriteI2C_Byte( 0xa1, 0x02 );   // DP scramble mode;
	}
//#endif

	/* Aux setup */
	HDMI_WriteI2C_Byte( 0xff, 0xac );
	HDMI_WriteI2C_Byte( 0x00, 0x60 );       //Soft Link train
	HDMI_WriteI2C_Byte( 0xff, 0xa6 );
	HDMI_WriteI2C_Byte( 0x2a, 0x00 );

	HDMI_WriteI2C_Byte( 0xff, 0x81 );
	HDMI_WriteI2C_Byte( 0x07, 0xfe );
	HDMI_WriteI2C_Byte( 0x07, 0xff );
	HDMI_WriteI2C_Byte( 0x0a, 0xfc );
	HDMI_WriteI2C_Byte( 0x0a, 0xfe );

	/* link train */
#ifndef _Test_Pattern_ // Normal display
	HDMI_WriteI2C_Byte( 0xff, 0xa8 );
	HDMI_WriteI2C_Byte( 0x2d, 0x80 ); //edp output video ;
#endif

	HDMI_WriteI2C_Byte( 0xff, 0x85 );
	HDMI_WriteI2C_Byte( 0x1a, eDP_lane );

	HDMI_WriteI2C_Byte( 0xff, 0xac );
	HDMI_WriteI2C_Byte( 0x00, 0x64 );
	HDMI_WriteI2C_Byte( 0x01, 0x0a );
	HDMI_WriteI2C_Byte( 0x0c, 0x85 );
	HDMI_WriteI2C_Byte( 0x0c, 0xc5 );
	mdelay( 200 );   //500
}

void LT8911EXB_link_train_result( void )
{
	u8 i, reg;
	HDMI_WriteI2C_Byte( 0xff, 0xac );
	for( i = 0; i < 10; i++ )
	{
		reg = HDMI_ReadI2C_Byte( 0x82 );
		//LT_INFO( "\r\n0x82 = ", reg );
		if( reg & 0x20 )
		{
			if( ( reg & 0x1f ) == 0x1e )
			{
				LT_INFO( "\r\nLink train success, 0x82 = %d", reg );
			} else
			{
				LT_INFO( "\r\nLink train fail, 0x82 = %d", reg );
			}

			//LT_INFO( "\r\npanel link rate: ", HDMI_ReadI2C_Byte( 0x83 ) );
			//LT_INFO( "\r\npanel link count: ", HDMI_ReadI2C_Byte( 0x84 ) );
			return;
		}else
		{
			LT_INFO( "\r\nlink trian on going..." );
		}
		mdelay( 100 );
	}
}

void LT8911EXB_MainLoop( void )
{
	u16		reg;
	bool	flag_mipi_on = 0;


	HDMI_WriteI2C_Byte( 0xff, 0x85 );
	//HDMI_WriteI2C_Byte(0x1d,0x00); //FM select byte clk
	//HDMI_WriteI2C_Byte(0x40,0xf7);
	//HDMI_WriteI2C_Byte(0x41,0x30);
	HDMI_WriteI2C_Byte( 0xa1, 0x02 );   //video check from mipi

	HDMI_WriteI2C_Byte( 0xff, 0x81 );   //video check rst
	HDMI_WriteI2C_Byte( 0x09, 0x7d );
	HDMI_WriteI2C_Byte( 0x09, 0xfd );
	mdelay( 50 );

	HDMI_WriteI2C_Byte( 0xff, 0x85 );
	reg	   = HDMI_ReadI2C_Byte( 0x76 );
	reg	   = reg * 256 + HDMI_ReadI2C_Byte( 0x77 );

//	if( reg == EXB_MIPI_Timing[vtotal] )
	if( ( reg > ( EXB_MIPI_Timing[vtotal] - 5 ) ) && ( reg < ( EXB_MIPI_Timing[vtotal] + 5 ) ) )
	{
		if( !flag_mipi_on )
		{
			HDMI_WriteI2C_Byte( 0xff, 0x81 ); //PCR reset
			HDMI_WriteI2C_Byte( 0x03, 0x7b );
			HDMI_WriteI2C_Byte( 0x03, 0xff );

			HDMI_WriteI2C_Byte( 0xff, 0xa8 );
			HDMI_WriteI2C_Byte( 0x2d, 0x88 );
			flag_mipi_on = 1;
			LT_INFO( "\r\nPCR reset" );
		}
	}else
	{
		HDMI_WriteI2C_Byte( 0xff, 0xa8 );
		HDMI_WriteI2C_Byte( 0x2d, 0x8c ); //edp output idle pattern;
		flag_mipi_on = 0;
	}
}

void LT8911EXB_config( void )
{
	
//	Reset_LT8911EXB( ); // 先Reset LT8911EXB ,用GPIO 先拉低LT8911EXB的复位脚 100ms左右，再拉高，保持100ms。

	LT8911EXB_ChipID( );   // read Chip ID

	LT8911EXB_eDP_Video_cfg( );
	LT8911EXT_init( );

	Read_DPCD010A = DpcdEXBRead( 0x010A ) & 0x01;

	if( Read_DPCD010A )
	{
		ScrambleMode = 1;
	}else
	{
		ScrambleMode = 0;
	}

	LT8911EXB_link_train( );

	LT8911EXB_link_train_result( );    // for debug

	LT8911EXB_InterruptEnable( );

	LT8911EXB_video_check( );          // just for Check MIPI Input
	
	//mdelay( 2000 );
	
	//LT8911EXB_video_check( );          // just for Check MIPI Input
	
	LT8911EXB_TxSwingPreSet();

#if 0
	while( 1 )
	{
	// 如果MIPI信号有断续，需要跑下面的函数。
		LT8911EXB_MainLoop( );

		mdelay( 1000 );
	}
#endif	
}

void Chip_delimitation(void)
{
	u8 ID0,ID1;
	
	//Reset_LT8911EXB( ); // 先Reset LT8911B / LT8911EXB ,用GPIO 先拉低LT8911EXB的复位脚 100ms左右，再拉高，保持100ms
	
	HDMI_WriteI2C_Byte( 0xff, 0x81 );
	HDMI_WriteI2C_Byte( 0x08, 0x7f );
	ID0 = HDMI_ReadI2C_Byte( 0x00 ) ;
	ID1 = HDMI_ReadI2C_Byte( 0x01 ) ;
	LT_INFO("Chip_delimitation 1,ID0:0x%x, ID1:0x%x!\n", ID0, ID1);

	if(ID0 == 0x17 && ID1 == 0x05)
	{
		LT_INFO("lt8911ext init!\n");
		LT8911EXB_config();// LT8911EXB
		mdelay( 20 );  //200
		LT8911EXB_link_train( );
		
	}
	else
	{
		LT_INFO("lt8911b init!\n");
		HDMI_WriteI2C_Byte( 0xff, 0x80 );    
		ID0 = HDMI_ReadI2C_Byte( 0x00 ) ;
		ID1 = HDMI_ReadI2C_Byte( 0x01 ) ;
		
		if(ID0 == 0x11 && ID1 == 0x89)
		{
			LT8911B_config();// LT8911B
		}
	}
}

int it8911b_stop(void)
{
	LT_INFO("it8911b_stop\n");
	gpio_request(gpio_avdd, "LCD AVDD");
	gpio_direction_output(gpio_avdd, 0);
	gpio_free(gpio_avdd);
	gpio_request(gpio_bl, "BL_ENABLE");
	gpio_direction_output(gpio_bl, 0);
	gpio_free(gpio_bl);

	gpio_request(gpio_avee, "LCD AVDD1");
	gpio_direction_output(gpio_avee, 0);
	gpio_free(gpio_avee);
	msleep(100);
	
	gpio_request(gpio_vdd, "LCD VDD");
	gpio_direction_output(gpio_vdd, 0);
	gpio_free(gpio_vdd);
	msleep(300);
	
	return  0;
}

int it8911b_run(void)
{
  LT_INFO("it8911b run\n");
  if (!i2c_check_functionality(client_hshcal->adapter, I2C_FUNC_I2C))
  {
  	dev_err(&client_hshcal->adapter->dev, "[lt8911b]client not i2c capable\n");
    LT_INFO("client not i2c capable\n");
    return  0;
  }
  
	gpio_request(gpio_bl, "BL_ENABLE");
	gpio_direction_output(gpio_bl, 0);

	reset_init();

	msleep(30);   //300
	Chip_delimitation();
	
	gpio_request(gpio_avdd, "LCD AVDD");
	gpio_direction_output(gpio_avdd, 1);	
	gpio_request(gpio_bl, "BL_ENABLE");
	gpio_direction_output(gpio_bl, 1);
	
	return  0;
}

static int hshcal_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  LT_INFO("hshcal_probe\n");
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
      dev_err(&client->adapter->dev, "[lt8911b]client not i2c capable\n");
      return -ENOMEM;
  }

  client_hshcal = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
  if (!client_hshcal) {
      dev_err(&client->adapter->dev, "[lt8911b]failed to allocate memory for module data\n");
      return -ENOMEM;
  }

  client_hshcal = client;
	LT_INFO("hshcal_probe IIC:0x%x\n", client_hshcal->addr);
  dev_info(&client->adapter->dev, "detected lt8911b edp\n");
  
	hshcal_parse(&client->dev);   //wqj  
  return 0;
}

static int hshcal_remove(struct i2c_client *client)
{
#ifdef ALPS_HMD_DEBUG
	LT_INFO(" hshcal_remove\n");
#endif

  kfree(client_hshcal);
  
  return 0;
}

static const struct of_device_id it8911b_ts_ids[] = {
	{.compatible = "IT,it8911b"},
	{ }
};

static const struct i2c_device_id hshcal_id[] = {
    { HSHCAL_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver hshcal_driver = {
    .probe     = hshcal_probe,
    .remove    = hshcal_remove,
    .id_table  = hshcal_id,
    .driver    = {
        .name  = HSHCAL_DRIVER_NAME,
        .of_match_table = of_match_ptr(it8911b_ts_ids),
    },
};
/*-----------------------------------------------------------------------------------------------*/
/* device driver                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
static int __init hshcal_init(void)
{
		int rc=0;
  
#ifdef ALPS_HMD_DEBUG
    LT_INFO(" hshcal_init\n");
#endif

    rc = i2c_add_driver(&hshcal_driver);
    if (rc != 0) {
        LT_INFO("can't add i2c driver\n");
        rc = -ENOTSUPP;
        goto out_region;
    }

    pdev = platform_device_register_simple(HSHCAL_DEVICE_NAME, -1, NULL, 0);
    if (IS_ERR(pdev)) {
        rc = PTR_ERR(pdev);
        goto out_driver;
    }
    LT_INFO("hshcal_init: platform_device_register_simple\n");

    return 0;

    platform_device_unregister(pdev);
    LT_INFO("hshcal_init: platform_device_unregister\n");
out_driver:
    i2c_del_driver(&hshcal_driver);
    LT_INFO("hshcal_init: i2c_del_driver\n");
out_region:

    return rc;
}

static void __exit hshcal_exit(void)
{
#ifdef ALPS_HMD_DEBUG
    LT_INFO(" hshcal_exit\n");
#endif
    i2c_del_driver(&hshcal_driver);
}

module_init(hshcal_init);
module_exit(hshcal_exit);

MODULE_DESCRIPTION("Alps Humidity Input Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
