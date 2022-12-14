/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 sp2508mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "sp2508mipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "SP2508_camera_sensor"

#define LOG_1 LOG_INF("SP2508,MIPI 1LANE\n")
#define LOG_2 LOG_INF("preview 1600*1200@30fps,864Mbps/lane; video 1600*1200@30fps,864Mbps/lane; capture 2M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1

static imgsensor_info_struct imgsensor_info = { 
    .sensor_id = SP2508MIPI_SENSOR_ID,

    //.checksum_value = 0xbaa74750,
	.checksum_value = 0x820e0cf,

    .pre = {
        .pclk = 30000000,              //record different mode's pclk
        .linelength = 1160,             //record different mode's linelength
        .framelength = 1241,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 1600,//2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1200,//1552,		//record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 85,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,   
    },
    .cap = {   // 30  fps  capture
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .cap1 = {    // 24 fps  capture
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,   
    },
    .normal_video = {
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,//4192,
		.grabwindow_height = 1200,//3104,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .hs_video = {     // 60 fps
        .pclk = 30000000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .slim_video = {
        .pclk = 30000000,//231270000,
        .linelength = 1160,
        .framelength = 1241,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1600,
        .grabwindow_height = 1200,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 300,
    },
    .margin = 0,
    .min_shutter = 7,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,    //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,   //support sensor mode num
    
    .cap_delay_frame = 1, 
    .pre_delay_frame = 3, 
    .video_delay_frame = 5,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,
    
    .isp_driving_current = ISP_DRIVING_8MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_1_LANE,
    .i2c_addr_table = {0x78, 0x7a, 0xff},
    .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,             //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                   //current shutter   // Danbo ??
    .gain = 0x100,                      //current gain     // Danbo ??
    .dummy_pixel = 0,                   //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,      //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x78,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =    
{{ 1600, 1200, 0000, 0000, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0000, 0000, 1600, 1200}, // Preview 2112*1558
 { 1600, 1200, 0000, 0000, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0000, 0000, 1600, 1200}, // capture 4206*3128
 { 1600, 1200, 0000, 0000, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0000, 0000, 1600, 1200}, // video 
 { 1600, 1200, 0000, 0000, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0000, 0000, 1600, 1200}, //hight speed video 
 { 1600, 1200, 0000, 0000, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0000, 0000, 1600, 1200},// slim video
 };// slim video  


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    kal_uint16 get_byte=0;

    char pu_send_cmd[1] = {(char)(addr & 0xFF)};
    iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
        
    char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}


static void set_dummy()
{

 #if 0
       LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
        write_cmos_sensor(0xfd, 0x01); 
	      write_cmos_sensor(0x05, (imgsensor.dummy_line>> 8) & 0xFF);
        write_cmos_sensor(0x06, imgsensor.dummy_line & 0xFF);     
	      write_cmos_sensor(0x09, (imgsensor.dummy_pixel>> 8) & 0xFF);
        write_cmos_sensor(0x0a, imgsensor.dummy_pixel & 0xFF);
        write_cmos_sensor(0x01, 0x01); 
        //mdelay(50);
#endif
  
}   /*  set_dummy  */

static kal_uint32 return_sensor_id()
{  
    write_cmos_sensor(0xfd, 0x00);
	  return ((read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	#if 1 
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

   
    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
	  imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
   #endif
}   /*  set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
   #if 1
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
       
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
    
    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
	
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)       
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
	  shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	  shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
  	
    // Update Shutter
    
    write_cmos_sensor(0xfd, 0x01); 
    write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
    write_cmos_sensor(0x04, shutter & 0xFF); 
    write_cmos_sensor(0x01, 0x01); 
    LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
    #endif
}   /*  write_shutter  */



/*************************************************************************
* FUNCTION
*   set_shutter
*
* DESCRIPTION
*   This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	#if 1
    unsigned long flags;
    if (shutter < 7)
          shutter = 7; 
	else if(shutter > 0xffff)
	   shutter = 0xffff;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    
	  write_shutter(shutter);
 #endif
}   /*  set_shutter */




static kal_uint16 gain2reg(const kal_uint16 gain)
{
//#if 0
   kal_uint16 reg_gain = 0x0000;
   
   reg_gain =  (( gain / BASEGAIN) << 4) + (( gain % BASEGAIN)* 16 / BASEGAIN);
   
   reg_gain = reg_gain & 0xFFFF;

   return (kal_uint16)reg_gain;
	
}




/*************************************************************************
* FUNCTION
*   set_gain
*
* DESCRIPTION
*   This function is to set global gain to sensor.
*
* PARAMETERS
*   iGain : sensor global gain(base: 0x40)
*
* RETURNS
*   the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{

	 #if 1
	  kal_uint8  iReg;

	    if(gain >= BASEGAIN && gain <= 15*BASEGAIN)
   	 {   
   	    	 iReg = 0x10 * gain/BASEGAIN;        //change mtk gain base to aptina gain base
			 //iReg += (gain%BASEGAIN)/(0x10/BASEGAIN);
						   	
   	    	 if(iReg<=0x10)
   	    	 {
   	    	    	 write_cmos_sensor(0xfd, 0x01);
   	    	    	 write_cmos_sensor(0x24, 0x10);//0x23
   	    	    	 write_cmos_sensor(0x01, 0x01);
                 LOG_INF("SP2508MIPI_SetGain = 16");
   	    	 }
   	    	 else if(iReg>= 0xa0)//gpw
   	    	 {
   	    	    	 write_cmos_sensor(0xfd, 0x01);
   	    	    	 write_cmos_sensor(0x24,0xa0);
   	    	    	 write_cmos_sensor(0x01, 0x01); 
                 LOG_INF("SP2508MIPI_SetGain = 160"); 
	        }        	
   	    	 else
   	    	 {
   	    	    	write_cmos_sensor(0xfd, 0x01);
   	    	    	write_cmos_sensor(0x24, (kal_uint8)iReg);
   	    	    	write_cmos_sensor(0x01, 0x01);
			          LOG_INF("SP2508MIPI_SetGain = %d",iReg);		 
	       }	
   	 }	
   	 else
   	    	 LOG_INF("error gain setting");	 
	 return gain;
        
        #endif
        
}   /*  set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    kal_uint8 iRation;
    kal_uint8 iReg;

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);

    /********************************************************
       *
       *   0x0101 Sensor mirror flip 
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/
    kal_uint8  iTemp; 
    write_cmos_sensor(0xfd, 0x01);
    //iTemp = read_cmos_sensor(0x3f) & 0x03;
    //iTemp&= ~0x03; //Clear the mirror and flip bits.
    
    switch (image_mirror) {
        case IMAGE_NORMAL:
			      write_cmos_sensor(0x3f, 0x00);    //Set normal
            break;
        case IMAGE_H_MIRROR:
            //write_cmos_sensor(0x3f, iTemp | 0x01); //Set mirror
            write_cmos_sensor(0x3f, 0x01);
            break;
        case IMAGE_V_MIRROR:
            //write_cmos_sensor(0x3f, iTemp | 0x02); //Set flip
            write_cmos_sensor(0x3f, 0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x3f, 0x03); //Set mirror and flip
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }

}


/*************************************************************************
* FUNCTION
*   night_mode
*
* DESCRIPTION
*   This function night mode of sensor.
*
* PARAMETERS
*   bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}   /*  night_mode  */

static void sensor_init(void)
{
 //from K2

 write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x3f,0x00);//0x03

	write_cmos_sensor(0xfd,0x00);
	write_cmos_sensor(0x1c,0x03);
	write_cmos_sensor(0x35,0x20); //pll bias
	write_cmos_sensor(0x2f,0x08); //pll clk 60M            
	write_cmos_sensor(0xfd,0x01);
	write_cmos_sensor(0x03,0x03); //exp time, 3 base
	write_cmos_sensor(0x04,0x09);
	write_cmos_sensor(0x06,0x10); //vblank
	write_cmos_sensor(0x24,0xa0); //pga gain 10x
	write_cmos_sensor(0x01,0x01); //enable reg write
	write_cmos_sensor(0x2b,0xc4); //readout vref
	write_cmos_sensor(0x2e,0x20); //dclk delay
	write_cmos_sensor(0x79,0x42); //p39 p40
	write_cmos_sensor(0x85,0x0f); //p51
	write_cmos_sensor(0x09,0x01); //hblank
	write_cmos_sensor(0x0a,0x40);
	write_cmos_sensor(0x21,0xef); //pcp tx 4.05v
	write_cmos_sensor(0x25,0xf0); //reg dac 2.7v, enable bl_en,vbl 1.28v ????0xf2 ????0xf0 ????????????????????
	write_cmos_sensor(0x26,0x00); //vref2 1v, disable ramp driver
	write_cmos_sensor(0x2a,0xea); //bypass dac res, adc range 0.745, vreg counter 0.9
	write_cmos_sensor(0x2c,0xf0); //high 8bit, pldo 2.7v
	write_cmos_sensor(0x8a,0x44); //pixel bias 1.58uA ????0x55 ????0x44 ????pixel bias
	write_cmos_sensor(0x8b,0x44); //????0x55 ????0x44 ????pixel bias
	write_cmos_sensor(0x19,0xf3); //icom1 1.7u, icom2 0.6u 
	write_cmos_sensor(0x11,0x30); //rst num
	write_cmos_sensor(0xd0,0x01); //disable boost
	write_cmos_sensor(0xd1,0x01);
	write_cmos_sensor(0xd2,0xd0);
	write_cmos_sensor(0x55,0x10);
	write_cmos_sensor(0x58,0x30);
	write_cmos_sensor(0x5d,0x15);
	write_cmos_sensor(0x5e,0x05);
	write_cmos_sensor(0x64,0x40);
	write_cmos_sensor(0x65,0x00);
	write_cmos_sensor(0x66,0x66);
	write_cmos_sensor(0x67,0x00);
	write_cmos_sensor(0x68,0x68);
	write_cmos_sensor(0x72,0x70);
	write_cmos_sensor(0xfb,0x25);
	write_cmos_sensor(0xf0,0x00);//offset
	write_cmos_sensor(0xf1,0x00);
	write_cmos_sensor(0xf2,0x00);
	write_cmos_sensor(0xf3,0x00);
	write_cmos_sensor(0xfd,0x02);//raw data digital gain
	write_cmos_sensor(0x00,0xb0);
	write_cmos_sensor(0x01,0xb0);
	write_cmos_sensor(0x03,0xb0);
	write_cmos_sensor(0x04,0xb0);
	write_cmos_sensor(0xfd,0x01);//mipi
	write_cmos_sensor(0xb3,0x00);
	write_cmos_sensor(0x93,0x01);
	write_cmos_sensor(0x9d,0x17);
	write_cmos_sensor(0xc5,0x01);
	write_cmos_sensor(0xc6,0x00);
	write_cmos_sensor(0xb1,0x01);
	write_cmos_sensor(0x8e,0x06);
	write_cmos_sensor(0x8f,0x50);
	write_cmos_sensor(0x90,0x04);
	write_cmos_sensor(0x91,0xc0);
	write_cmos_sensor(0x92,0x01);
	write_cmos_sensor(0xa1,0x05);
	write_cmos_sensor(0xaa,0x01);
	write_cmos_sensor(0xac,0x01);
 	LOG_INF("Exit sensor_init.\n");   

}   /*  sensor_init  */



static void preview_setting(kal_uint16 currefps)   //PreviewSetting
{

}   /*  preview_setting  */

static void capture_setting(kal_uint16 currefps)  // IMX135MIPI_set_13M
{

}

static void normal_video_setting(kal_uint16 currefps)    // VideoFullSizeSetting
{

}
static void hs_video_setting()  // VideoHDSetting_60fps
{

}

static void slim_video_setting()  // VideoHDSetting
{

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{

}


/*************************************************************************
* FUNCTION
*   get_imgsensor_id
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //kal_uint32 p1,p2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
			      //*sensor_id = 0x2508;
			      //p1 = read_cmos_sensor(0x02) << 8;
			      //p2 = read_cmos_sensor(0x03);
			      //LOG_INF("preview setting 0x0100 = %d,0x03 = %d\n",p1,p2);
			      
            if (*sensor_id == imgsensor_info.sensor_id) {               
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);      
                return ERROR_NONE;
            }   
            LOG_INF("Read sensor id fail:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   open
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0; 
    LOG_1;  
    LOG_2;
   
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            //sensor_id = 0x2508;
            if (sensor_id == imgsensor_info.sensor_id) {                
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);   
                break;
            }   
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }        
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
    
    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
	  imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*   close
*
* DESCRIPTION
*   
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/ 
    
    return ERROR_NONE;
}   /*  close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("Enter preview\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength; 
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting(imgsensor.current_fps);

    return ERROR_NONE;
}   /*  preview   */

/*************************************************************************
* FUNCTION
*   capture
*
* DESCRIPTION
*   This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("Enter capture\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == 300) {
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;  
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else  {  //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;  
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } 
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
    //sset_dummy();
    capture_setting(imgsensor.current_fps); 
    return ERROR_NONE;
}   /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("Enter normal_video\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps); 
    return ERROR_NONE;
}   /*  normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("Enter hs_video\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting(); 
    return ERROR_NONE;
}   /*  hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("Enter slim_video\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();  
    return ERROR_NONE;
}   /*  slim_video   */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
    
    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;       

    
    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight    = imgsensor_info.hs_video.grabwindow_height;
    
    sensor_resolution->SensorSlimVideoWidth  = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    
    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 1; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;  
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
    
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;   
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 5; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */
    
    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x 
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;      
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
                  
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

            break;   
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            
            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
       
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

            break;    
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:         
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
                  
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
                  
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

            break;
        default:            
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;      
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }
    
    return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;  
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);  // VideoFullSizeSetting
            break;    
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);  // VideoHDSetting_120fps
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data); // VideoHDSetting
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}   /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker     
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
    kal_uint32 frame_length;
  
    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);  
            //set_dummy();         
            break;          
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0; 
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);    
            //set_dummy();       
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:			
            if (imgsensor.current_fps == 300){
                    frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                    spin_lock(&imgsensor_drv_lock);
                    imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                    imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                    imgsensor.min_frame_length = imgsensor.frame_length;
                    spin_unlock(&imgsensor_drv_lock);
                }
            else{
                    frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                    spin_lock(&imgsensor_drv_lock);
                    imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
                    imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
                    spin_unlock(&imgsensor_drv_lock);
                }
                //set_dummy();
            break;  
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock); 
            //set_dummy();          
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock); 
            //set_dummy();      
            break; 
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
			      imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			      imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			      imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);  
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    } 
    return ERROR_NONE;
	
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;      
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO: 
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;
    
    SENSOR_WINSIZE_INFO_STRUCT *wininfo;    
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;         
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:       
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break; 
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32); 
            break; 
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            //get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data_32, (MUINT32 *)(*(feature_data_32+1)));
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing             
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;                             
            break;              
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);       
            break;
		   case SENSOR_FEATURE_SET_HDR:
			      LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			      spin_lock(&imgsensor_drv_lock);
			      imgsensor.ihdr_en = (BOOL)*feature_data;
			      spin_unlock(&imgsensor_drv_lock);		
			      break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            //wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(*(feature_data_32+1));
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;    
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
			}
		    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			      LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2)); 
			      ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));	
			      break;
		   default:
			break;
	}
  
    return ERROR_NONE;
}   /*  feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 SP2508_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}   /*  SP2508_MIPI_RAW_SensorInit  */
