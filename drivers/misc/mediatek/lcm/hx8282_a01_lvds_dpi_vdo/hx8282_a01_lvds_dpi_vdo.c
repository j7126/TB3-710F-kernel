/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>

#elif (defined BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
//#include "../multibridge/ti_sn65dsi8x_driver.h"
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
//#define SN65DSI_DEBUG
#define FRAME_WIDTH  (1024)
#define FRAME_HEIGHT (600)


#define LVDS_LCM_MIPI_EN    (GPIO18 | 0x80000000)
#define GPIO_LCD_PWR_EN      (GPIO15 | 0x80000000)
#define GPIO_LCD_RST_EN      (GPIO70 | 0x80000000)
#define LVDS_LCM_STBY       (GPIO68 | 0x80000000)
#define GPIO_LDO_EN		(GPIO21 | 0x80000000)

#define LCM_DSI_6589_PLL_CLOCK_201_5 0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
//add by zym
static int g_first_suspend = 0;

int lcm_source = 0;

#define SET_RESET_PIN(v)    (mt_set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY 0xAB

typedef unsigned char    kal_uint8;



static struct sn65dsi8x_setting_table {
    unsigned char cmd;
    unsigned char data;
};
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------


 //i2c hardware type -start
#define SN65DSI8X_I2C_ID	I2C2
static struct mt_i2c_t sn65dis8x_i2c;

#ifdef BUILD_LK
#define I2C_CH                2
#define sn65dsi8x_I2C_ADDR       0x5a  //0x2d

 U32 sn65dsi8x_reg_i2c_read (U8 addr, U8 *dataBuffer)
 {
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    sn65dis8x_i2c.id = SN65DSI8X_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    sn65dis8x_i2c.addr = (sn65dsi8x_I2C_ADDR >> 1);
    sn65dis8x_i2c.mode = ST_MODE;
    sn65dis8x_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&sn65dis8x_i2c, dataBuffer, len, len);
   
#if 0
	 U32 ret_code = I2C_OK;
	 U8 write_data = addr;
 
	 /* set register command */
	 ret_code = mt_i2c_write(I2C_CH, sn65dsi8x_I2C_ADDR, &write_data, 1, 0); // 0:I2C_PATH_NORMAL
 
	 if (ret_code != I2C_OK)
		 return ret_code;
 
	 ret_code = mt_i2c_read(I2C_CH, sn65dsi8x_I2C_ADDR, dataBuffer, 1,0); // 0:I2C_PATH_NORMAL
#endif 
	 return ret_code;
 }
 

 U32 sn65dsi8x_reg_i2c_write(U8 addr, U8 value)
 {
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    sn65dis8x_i2c.id = SN65DSI8X_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set sn65dis8x I2C address to >>1 */
    sn65dis8x_i2c.addr = (sn65dsi8x_I2C_ADDR >> 1);
    sn65dis8x_i2c.mode = ST_MODE;
    sn65dis8x_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&sn65dis8x_i2c, write_data, len);
     
#if 0
	 U32 ret_code = I2C_OK;
	 U8 write_data[2];
 
	 write_data[0]= addr;
	 write_data[1] = value;
 	
	 ret_code = mt_i2c_write(I2C_CH, sn65dsi8x_I2C_ADDR, write_data, 2,0); // 0:I2C_PATH_NORMAL
	 printf("sn65dsi8x_reg_i2c_write cmd=0x%x  data=0x%x ret_code=0x%x\n",addr,value,ret_code);
#endif	 
	 return ret_code;
 }

 //end
 
 /******************************************************************************
 *IIC drvier,:protocol type 2 end
 ******************************************************************************/
#else
static DEFINE_MUTEX(sn65dsi83_i2c_access);
extern int sn65dsi83_read_byte(kal_uint8 cmd, kal_uint8 *returnData);
extern int sn65dsi83_write_byte(kal_uint8 cmd, kal_uint8 Data);


#endif
//sn65dis83 chip init table
static struct sn65dsi8x_setting_table sn65dis83_init_table[]=
{
#if 0
	{0x09,  0x00},
	{0x0A,  0x05},
	{0x0B,  0x10},
	{0x0D,  0x00},
	{0x10,  0x26},
	{0x11,  0x00},
	{0x12,  0x29},
	{0x13,  0x00},
	{0x18,  0x78},
	{0x19,  0x00},
	{0x1A,  0x03},
	{0x1B,  0x00},
	{0x20,  0x00},
	{0x21,  0x05},
	{0x22,  0x00},
	{0x23,  0x00},
	{0x24,  0x20},
	{0x25,  0x03},
	{0x26,  0x00},
	{0x27,  0x00},
	{0x28,  0x21},
	{0x29,  0x00},
	{0x2A,  0x00},
	{0x2B,  0x00},
	{0x2C,  0x2a},
	{0x2D,  0x00},
	{0x2E,  0x00},
	{0x2F,  0x00},
	{0x30,  0x04},
	{0x31,  0x00},
	{0x32,  0x00},
	{0x33,  0x00},
	{0x34,  0x2b},
	{0x35,  0x00},
	{0x36,  0x04},
	{0x37,  0x00},
	{0x38,  0x2b},
	{0x39,  0x00},
	{0x3A,  0x04},
	{0x3B,  0x00},
	{0x3C,  0x10},
	{0x3D,  0x00},
	{0x3E,  0x00},
#endif

#if 0 //RGB888
	{0x09, 0x00},
	{0x0A, 0x05},
	{0x0B, 0x10},
	{0x0D, 0x00},
	{0x10, 0x26},
	{0x11, 0x00},
	{0x12, 0x29},
	{0x13, 0x00},
	{0x18, 0x78},
	{0x19, 0x00},
	{0x1A, 0x03},
	{0x1B, 0x00},
	{0x20, 0x00},
	{0x21, 0x05},
	{0x22, 0x00},
	{0x23, 0x00},
	{0x24, 0x00},
	{0x25, 0x00},
	{0x26, 0x00},
	{0x27, 0x00},
	{0x28, 0x21},
	{0x29, 0x00},
	{0x2A, 0x00},
	{0x2B, 0x00},
	{0x2C, 0x2a},
	{0x2D, 0x00},
	{0x2E, 0x00},
	{0x2F, 0x00},
	{0x30, 0x04},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x2b},
	{0x35, 0x00},
	{0x36, 0x00},
	{0x37, 0x00},
	{0x38, 0x00},
	{0x39, 0x00},
	{0x3A, 0x00},
	{0x3B, 0x00},
	{0x3C, 0x00},
	{0x3D, 0x00},
	{0x3E, 0x00},
#endif

#if 1

{0x09,              0x00},
{0x0A,              0x03},
{0x0B,              0x18},
{0x0D,              0x00},
{0x10,              0x2e},
{0x11,              0x00},
{0x12,              0x28},
{0x13,              0x00},
{0x18,              0x78},
{0x19,              0x08}, //0x00
{0x1A,              0x01}, //0x03
{0x1B,              0x00},
{0x20,              0x00},
{0x21,              0x04},
{0x22,              0x00},
{0x23,              0x00},
{0x24,              0x00},
{0x25,              0x00},
{0x26,              0x00},
{0x27,              0x00},
{0x28,              0x21},
{0x29,              0x00},
{0x2A,              0x00},
{0x2B,              0x00},
{0x2C,              0x14},
{0x2D,              0x00},
{0x2E,              0x00},
{0x2F,              0x00},
{0x30,              0x06},
{0x31,              0x00},
{0x32,              0x00},
{0x33,              0x00},
{0x34,              0x96},
{0x35,              0x00},
{0x36,              0x00},
{0x37,              0x00},
{0x38,             0x00},
{0x39,              0x00},
{0x3A,             0x00},
{0x3B,              0x00},
{0x3C,              0x00},
{0x3D,              0x00},
{0x3E,              0x00},
#endif

};
static void push_table(struct sn65dsi8x_setting_table *table, unsigned int count)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {	
            case REGFLAG_DELAY :
            MDELAY(table[i].data);
                break;		
            case 0xFF:
                break;
				
            default:
		#ifdef BUILD_LK
		sn65dsi8x_reg_i2c_write(cmd, table[i].data);//TI_Sensor_Write(cmd, table[i].data);
		#else
		sn65dsi83_write_byte(cmd, table[i].data);
		#endif
       	}
    }
	
}
static void dump_reg_table(struct sn65dsi8x_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned char data;
	
    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {	
            case REGFLAG_DELAY :
            MDELAY(table[i].data);
                break;		
            case 0xFF:
                break;
				
            default:
		#ifdef BUILD_LK
		sn65dsi8x_reg_i2c_read(cmd,&data);	
		printf("dump cmd=0x%x  data=0x%x \n",cmd,data);
		#else
		sn65dsi83_read_byte(cmd,&data);
		printk("dump cmd=0x%x  data=0x%x \n",cmd,data);
		#endif
       	}
    }
	
}

void init_sn65dsi8x(void)
{
	unsigned char data;

    push_table(sn65dis83_init_table, sizeof(sn65dis83_init_table)/sizeof(struct sn65dsi8x_setting_table));
	#if 0
	#ifdef BUILD_LK
        sn65dsi8x_reg_i2c_write(0x0D,0x01);//enable pll clock
   		int i=30;
		while(i){
			sn65dsi8x_reg_i2c_read(0x0A,&data);
			printf("gongbo reg(0x0A) =%d\n", data);
			if((data >> 7) == 0x01){
				break;
			}
			i--;
			MDELAY(10);
		}
		sn65dsi8x_reg_i2c_write(0x09,0x01); 
	#else
		sn65dsi83_write_byte(0x0D,0x01);//enable pll clock

		int i=30;
		while(i){
			sn65dsi83_read_byte(0x0A,&data);
			printk("gongbo reg(0x0A) =%d\n", data);
			if((data >> 7) == 0x01){
				break;
			}
			i--;
			MDELAY(10);
		}
		sn65dsi83_write_byte(0x09,0x01);//soft reset
	#endif
	#endif
    
	//MDELAY(5);
	//sn65dsi8x_reg_i2c_write(0x09,1);//soft reset
	//MDELAY(5);
	
	#ifdef SN65DSI_DEBUG//add for debug
    #ifdef BUILD_LK
	sn65dsi8x_reg_i2c_write(0xe0,1);//
	sn65dsi8x_reg_i2c_write(0xe1,0xff);//
	MDELAY(5);
	sn65dsi8x_reg_i2c_read(0xe5, &data);  
	printf("dump cmd=0xe5  data=0x%x \n",data);
	
	dump_reg_table(sn65dis83_init_table, sizeof(sn65dis83_init_table)/sizeof(struct sn65dsi8x_setting_table)); //for debug
    #endif
	#endif//debug end
}

/************************************************************************
*power fuction
*************************************************************************/
#ifdef BUILD_LK
extern	void upmu_set_rg_vgp1_vosel(U32 val);
extern	void upmu_set_rg_vgp1_en(U32 val);
void lvds_power_init(void)
{
    upmu_set_rg_vgp1_vosel(0x5);
    upmu_set_rg_vgp1_en(0x1);
}
#else //for kernel
//extern bool hwPowerOn(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt, char *mode_name);
#if 0
extern bool hwPowerOn(MT_POWER powerId, MT_POWER_VOLTAGE powerVolt, char *mode_name);

void lvds_kernel_power_init(void)
{
	hwPowerOn(MT6325_POWER_LDO_VGP1,VOL_2800,"LVDS");
	//hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_3300,"LVDS");
}
void lvds_kernel_power_deinit(void)
{
	if(!g_first_suspend)
	{
		mt6325_upmu_set_rg_vgp1_en(0);//
		upmu_set_rg_vgp1_vosel(0);//7=3.3v
		g_first_suspend = 1;
	}
	else
	{
		hwPowerDown(MT6325_POWER_LDO_VGP1,"LVDS");
		//hwPowerDown(MT65XX_POWER_LDO_VGP6,"LVDS");
	}
}
#endif
#endif
//
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcd_power_en(unsigned char enabled)
{
    if (enabled)
    {

	    mt_set_gpio_mode(GPIO_LDO_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LDO_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LDO_EN, GPIO_OUT_ONE);
		MDELAY(50);
        mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
    }
    else
    {
        mt_set_gpio_mode(GPIO_LCD_PWR_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_PWR_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
		MDELAY(10);
        mt_set_gpio_mode(GPIO_LDO_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LDO_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LDO_EN, GPIO_OUT_ZERO); 
    }
}

static void lcd_lcm_en(unsigned char enabled)
{
	if(enabled)
	{
		 mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	        mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	        mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
	}
	else
	{
		 mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
	        mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
	        mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	}
}


static void lcd_reset(unsigned char enabled)
{
    if (enabled)
    {
        mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
		MDELAY(1);
		mt_set_gpio_mode(LVDS_LCM_STBY, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_STBY, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_STBY, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);  
		MDELAY(1);
		mt_set_gpio_mode(LVDS_LCM_STBY, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_STBY, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_STBY, GPIO_OUT_ZERO);
    }
}

static void lcd_stbyb(unsigned char enabled)
{
    if (enabled)
    {
        mt_set_gpio_mode(LVDS_LCM_STBY, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_STBY, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_STBY, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_mode(LVDS_LCM_STBY, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_STBY, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_STBY, GPIO_OUT_ZERO);    	
    }
}

static void ti_enable(unsigned char enabled)
{
    if (enabled)
    {
        mt_set_gpio_mode(LVDS_LCM_MIPI_EN, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_MIPI_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_MIPI_EN, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_mode(LVDS_LCM_MIPI_EN, GPIO_MODE_00);
        mt_set_gpio_dir(LVDS_LCM_MIPI_EN, GPIO_DIR_OUT);
        mt_set_gpio_out(LVDS_LCM_MIPI_EN, GPIO_OUT_ZERO);    	
    }
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{	
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->dsi.mode   = SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;// BURST_VDO_MODE;
	
		// DSI
		/* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	    params->dsi.word_count=600*3;	

		params->dsi.vertical_sync_active= 6;//10;
		params->dsi.vertical_backporch= 23;//12;
		params->dsi.vertical_frontporch= 6;//9;
		params->dsi.vertical_active_line= FRAME_HEIGHT;//hight

		params->dsi.horizontal_sync_active				=20;//100;  //
		params->dsi.horizontal_backporch				= 150;//80; //
		params->dsi.horizontal_frontporch				= 150;//100; //
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;//=wight
		
		params->dsi.ssc_disable = 1;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL
		params->dsi.PLL_CLOCK = 204;//this value must be in MTK suggested table
        params->dsi.cont_clock = 1;//if not config this para, must config other 7 or 3 paras to gen. PLL
}


static unsigned int lcm_compare_id(void)
{
#if defined(BUILD_LK)
			printf("TM070DDH06_MIPI2LVDS  lcm_compare_id \n");
#endif

		return 1;
}


static void lcm_init(void)
{
	pmic_set_register_value(PMIC_RG_VGP1_VOSEL,5); //2.8V
	pmic_set_register_value(PMIC_RG_VGP1_EN,1);
	lcd_lcm_en(1);
	MDELAY(20);
	lcd_reset(0);
    MDELAY(50);
	lcd_reset(1);
    MDELAY(50);

	lcd_power_en(1);
	MDELAY(50);
#ifdef BUILD_LK	
	ti_enable(1);
	MDELAY(20);

   	init_sn65dsi8x();
	MDELAY(500);
#else
	
	   printk("tM070ddh06--kernel--lcm_init \n");


#endif    
}


static void lcm_suspend(void)
{
	
	lcd_reset(0);
	lcd_power_en(0);
	MDELAY(50);
	pmic_set_register_value(PMIC_RG_VGP1_EN,0);
	
	MDELAY(30);
	ti_enable(0);
	MDELAY(50);
	lcd_lcm_en(0);
}


static void lcm_resume(void)
{    
	lcd_lcm_en(1);
    lcd_reset(0);
    MDELAY(50);
	lcd_reset(1);
	pmic_set_register_value(PMIC_RG_VGP1_EN,1);

    MDELAY(20);
	lcd_power_en(1);
	MDELAY(50);

	ti_enable(1);
	MDELAY(20);

   	init_sn65dsi8x();
	MDELAY(200);
}



LCM_DRIVER hx8282_a01_lvds_dpi_vdo_lcm_drv = 
{
    .name		= "hx8282_a01_lvds_dpi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};

