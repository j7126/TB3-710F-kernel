/*
 * iqs263.c - Android SAR Sensor Driver for IQS263
 *
 * Copyright (C) 2013 Azoteq (Pty) Ltd
 * Author: Alwino van der Merwe <alwino.vandermerwe@azoteq.com>
 *
 * Based on mcs5000_ts.c, azoteqiqs440_ts.c, iqs263.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	Azoteq (Pty) Ltd does not take responsibility for the use of this driver
 *
 *	This driver is an example driver. It will need to be ported to
 *	the specific platform and for the specific case in which it is used.
 */
#include "iqs263.h"
#include "IQS263_Init.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#include <mach/upmu_common.h>
#include <cust_eint.h>
#include <mach/eint.h>
#include <linux/kthread.h>


/*	Get time for the IQS263 Timeout	*/
#include <linux/time.h>
#define IQS_I2C_NUM 2
static int num = 0;
static u8 cap_sensor_flag=0;
static u8 enter_sleep_flag = 0;
static struct i2c_board_info __initdata i2c_devs0  = {
	I2C_BOARD_INFO(DEVICE_NAME,0x44)
};

static struct i2c_client *iqs263_client = NULL;
/****************************
 *	Pins used in this setup	*
 ****************************/

/*	RDY Pin used on this specific setup	- needs to be changed
 *	for each case (hardware dependent)
 */
/*	GPIO0_27 - Bank 0	*/
#define RDY_LINE_PIN			(GPIO6 |0x80000000)

/*	VDDHI used on this specific setup	- needs to be changed
 *	for each case (hardware dependent).
 *	The SAR sensor can also be powered from the voltage
 *	rail of the system, but this allows more freedom.
 */
/*	GPIO0_7 - Bank 0 Header P9-42 on BBB	*/
#define PWR_LINE_PIN			7

/*	Numbet of Bytes to read from IQS263 continuously	*/
#define IQS263_BLOCK_SIZE		3
#define IQS263_SETUP_REG_SIZE	26

/* Boolean value used for the initial setup */
bool doInitialSetup;
bool initialATI;
/*	Indicate the a Reseed of the IQS263 will follow	*/
bool reseed;

u8 events;

/*	Boolean to keep track of Chip Reset Events	*/
bool showReset;
/*	Boolean to keep track of Prox   Events	*/
bool prox;
/*	Boolean to keep track of Touch Events	*/
bool touch;

/*	Counter to keep track of setup state	*/
u8 setupCounter;

/*	Global variable to keep the currentState in	*/
u8 currentState;	/*	Always start at state 0	*/

/*	Boolean to indicate event mode active or not	*/
bool eventMode;		/*	not activated by default	*/

/*	Each client has this additional data	*/
struct iqs263_sar_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct iqs263_sar_platform_data *platform_data;
};

/*	Struct to keep the timer SAR Timer information in	*/
static struct timer_list stuck_timer;

/*	Struct to keep a stuck timer in	*/
static struct timer_list stuck;

/********************************************************
 *		The IQS263 specific setup is done here			*
 *		This setup will change for each application		*
 ********************************************************/

/*	Command for reseeding the Azoteq IQS263	*/
static void iqs263_reseed(struct i2c_client *client)
{
//	i2c_smbus_write_byte_data(client, PROX_SETTINGS0, RESEED);
}

/*	Command for re-doing ATI on the Azoteq IQS263	*/
static void iqs263_reati(struct i2c_client *client)
{
	/*	Redo ATI	*/
//	i2c_smbus_write_byte_data(client, PROX_SETTINGS0, REDO_ATI);
}

const unsigned char mydata[8] = {0x04,0x6d,0x6d,0x00,0x3d,0x00,0x2d,0xff};

/************************************************************************
 *							IQS263 Initial Setup						*
 ************************************************************************/
/* Setup the Active Channels for Azoteq IQS263 */
static void iqs263_init_setup(struct i2c_client *client,u8 numOfRegs)
{
	u8 dataBuffer[8];
	/**
	 *	Setup IQS263 completely - start at ATI register and
	 *	run through all regsiters
	 */
	switch(num)
	{
		case 0:
		dataBuffer[0] = ACTIVE_CHS;
		i2c_smbus_write_i2c_block_data(client, ACTIVE_CHANNEL_263, 1,
			dataBuffer);
		num++;
			break;
		case 1:
		dataBuffer[0] = PROX_THRESHOLD;
        	dataBuffer[1] = TOUCH_THRESHOLD_CH1;
        	dataBuffer[2] = TOUCH_THRESHOLD_CH2;
        	dataBuffer[3] = TOUCH_THRESHOLD_CH3;
        	dataBuffer[4] = MOVEMENT_THRESHOLD;
        	dataBuffer[5] = RESEED_BLOCK;
        	dataBuffer[6] = HALT_TIME;
        	//dataBuffer[7] = I2C_TIMEOUT;
		i2c_smbus_write_i2c_block_data(client, THRESHOLD_263, 7,
			dataBuffer);
		num++;
			break;
		case 2:
		dataBuffer[0] = LOW_POWER;
        	dataBuffer[1] = ATI_TARGET_TOUCH;
        	dataBuffer[2] = ATI_TARGET_PROX;
		i2c_smbus_write_i2c_block_data(client, TIMING_TARGET_263, 3,
			dataBuffer);
		num++;
			break;
		case 3:
		dataBuffer[0] = MULTIPLIERS_CH0;
        	dataBuffer[1] = MULTIPLIERS_CH1;
        	dataBuffer[2] = MULTIPLIERS_CH2;
        	dataBuffer[3] = MULTIPLIERS_CH3;
        	dataBuffer[4] = BASE_VAL;
		i2c_smbus_write_i2c_block_data(client, MULTIPLIERS_263, 5,
			dataBuffer);
		num++;
			break;
		case 4:
		dataBuffer[0] = PROXSETTINGS0_VAL;
        	dataBuffer[1] = PROXSETTINGS1_VAL_SLEEP;
        	dataBuffer[2] = PROXSETTINGS2_VAL;
        	dataBuffer[3] = PROXSETTINGS3_VAL;
        	dataBuffer[4] = EVENT_MASK_VAL;
		i2c_smbus_write_i2c_block_data(client, PROXSETTING_263, 5,
			dataBuffer);
		num++;
			break;
		default:
 			break;
	}
}

void stuck_time(unsigned long data);
void iqs263_event_mode_handshake(void);
/************************************************************************
 *						State Machine Helper Functions					*
 ************************************************************************/
/**
 *	Check for Events on the IQS263. ATI, Prox and Touch event. Also check
 *	the Show Reset Flag
 *	The function returns no value - instead Global flags are set to
 *	indicate the events that ocurred
 */
static void readEvents(struct i2c_client *client)
{
	mt_eint_mask(CUST_EINT_CAP_PANEL_NUM);
	u8 i=0, data_buffer[6]={0};
	for(i=0;i<2;i++)
	{
		data_buffer[0] = TOUCH_STAT_263;
		mdelay(5);
		iqs263_event_mode_handshake();	
		i2c_master_send(client,data_buffer,1);
		mdelay(5);
		iqs263_event_mode_handshake();	
		i2c_master_recv(client, data_buffer,2);
		
		if (data_buffer[0]&CH_1_EVENT)
		{
			prox = true;
			printk("[zoro]prox = true\n");
			cap_sensor_flag = 1;
		}
		else
		{
			prox = false;
			printk("[zoro]prox = false\n");
			cap_sensor_flag = 0;
		}
		mdelay(600);
	}
	mt_eint_unmask(CUST_EINT_CAP_PANEL_NUM);
}

/*************************************************************************/

/*	Platform data for the Azoteq IQS263 SAR driver	*/
struct iqs263_sar_platform_data {
	void (*cfg_pin)(void);
};

/**
 *	Because the IQS263 operates in event mode, implement a handshake
 *	function that will initiate comms with the IQS263 if we want to talk
 *	to it.
 */
void iqs263_event_mode_handshake(void)
{
	/********************************************************
	 *			Try and do an Event Mode Handshake			*
	 *******************************************************/
	/*
	 *	There might be another way to build in the delay.
	 *	Event mode handshake is done by manually pulling
	 *	the IQS263 RDY line low for ~10ms and then releasing
	 *	it. This will tell the IQS263 that the Master wants
	 *	to talk and it will then give a Communications window
	 *	which will call the interrupt request function.
	 *	From there the IQS263 can be read again
	 */
	/*	Pull RDY low	*/
	mt_set_gpio_mode(RDY_LINE_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(RDY_LINE_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(RDY_LINE_PIN,0);
	/*	Hold the RDY low for ~10ms	*/
	mdelay(HANDSHAKE_DELAY_HOLD);
	/*	Release RDY line	*/
	mt_set_gpio_dir(RDY_LINE_PIN, GPIO_DIR_IN);
	/*	Delay before talking to IQS263	*/
	mt_set_gpio_mode(RDY_LINE_PIN, GPIO_MODE_06);
	udelay(HANDSHAKE_DELAY_SET);
}

void iqs263_enter_sleep(struct i2c_client *client);
/**
 *	Function that gets called when an interrupt does not occur after a while
 */
void interrupt_timer(unsigned long data)
{
	if (currentState > 0) {
		/*	Now from here Reseed and go back to State 0	*/
		currentState = 0;
		/*	Chip will re-ATI if reseed was out of spec	*/
		reseed = true;	/*	Reseed	*/
	}

	/********************************************************
	 *			Try and do an Event Mode Handshake			*
	 *******************************************************/

	iqs263_event_mode_handshake();
}

/** Timer interrupt function	*/
void stuck_time(unsigned long data)
{
	u8 data_buffer[6]={0};
	data_buffer[0] = TOUCH_STAT_263;
	iqs263_event_mode_handshake();	
	i2c_master_send(iqs263_client,data_buffer,1);
	iqs263_event_mode_handshake();	
	i2c_master_recv(iqs263_client, data_buffer,2);

	if (data_buffer[0]&CH_1_EVENT)
	{
		prox = true;
		printk("[zoro]prox = true\n");
		cap_sensor_flag = 1;
	}
	else
	{
		prox = false;
		printk("[zoro]prox = false\n");
		cap_sensor_flag = 0;
	}
}

/**
 *	Interrupt event fires on the Falling edge of
 *	the RDY signal from the Azoteq IQS263
 */

static struct workqueue_struct * iqs263_workqueue = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int iqs_flag = 0;
static struct mutex iqs_i2c_lock;

//static irqreturn_t iqs263_sar_interrupt(int irq, void *dev_id)
static int iqs263_sar_interrupt(void *data)
{
	//struct iqs263_sar_data *data = dev_id;
	
	do
	{
		//set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, iqs_flag != 0);
		iqs_flag = 0;
		printk("[zoro] here!\n");
		//set_current_state(TASK_RUNNING); 

		/*	Do the initial setup for the IQS263
		 *	Because we have to wait for the RDY from the IQS263 and
		 *	we cannot change the way Android handles I2C comms
		 *	the setup will take a few comms cycles
		 */

		/*
		 *	Check for a reset - if reset was seen,
		 *	Then setup has to been done all over again
		 *
		 *	If we need to setup the IQS263, traverse through the
		 *	setup
		 */
		//if(mt_get_gpio_in(RDY_LINE_PIN))
		//	continue;
		if (doInitialSetup) {
			switch (setupCounter) {
			/*	Setup ProxSettings	*/
			case 0:
				printk(KERN_ALERT "Initializing..");
				/*	Do event mode handshake	*/
				iqs263_init_setup(iqs263_client,IQS263_SETUP_REG_SIZE);
				if(num >= 5)
				{
					num = 0;
					setupCounter++;
				}
				break;
			default:
				setupCounter = 0;
				doInitialSetup = false;
				break;
			}
		}
		
		/**********************	State Machine	***************************/
		else
		{
			printk("readEvents\n");
			readEvents(iqs263_client);	/*	Check Events	*/
		}
	}while ( !kthread_should_stop() ); 
	printk("shou not here!\n");
	return 0;
}

static void iqs_event_handler(void)
{
	iqs_flag=1; 
	wake_up_interruptible(&waiter);
}

static struct task_struct *thread = NULL;

static ssize_t show_CapSensor_Data(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(cap_sensor_flag==0)
	{
		printk("cap sensor away");
		return sprintf(buf,"removed\n");
	}
	else
	{   
		printk("near");
		return sprintf(buf,"near\n");
	} 
}

static DEVICE_ATTR(CapSensor_Data,  0666, show_CapSensor_Data, NULL);



static struct attribute *sx9310_attributes[] = {
	&dev_attr_CapSensor_Data.attr,
	NULL,
};
static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};

static unsigned int iqs263_sysfs_init(void)
{
	int ret;
	struct kobject *iqs263_debug_kobj;
	iqs263_debug_kobj = kobject_create_and_add("CapSensor", NULL) ;
	if (iqs263_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_file(iqs263_debug_kobj, &dev_attr_CapSensor_Data.attr);
	if (ret)
	{
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
}


/**
 *	The probe function is called when Android is looking
 *	for the IQS263 on the I2C bus (I2C-1)
 */
static int iqs263_sar_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	printk("[zoro] iqs263_sar_probe~\n");
	struct iqs263_sar_data *data;
	int ret;
	iqs263_sysfs_init();
	iqs263_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL);
	memset(iqs263_client, 0, sizeof(struct i2c_client));
	iqs263_client = client;
	/*	Allocate memory	*/
	data = kzalloc(sizeof(struct iqs263_sar_data), GFP_KERNEL);
	data->client = client;
	data->platform_data = client->dev.platform_data;

		
	mt_eint_mask(CUST_EINT_CAP_PANEL_NUM);
	thread = kthread_run(iqs263_sar_interrupt, 0, DEVICE_NAME);
	if (IS_ERR(thread)) {
		printk("kthread_run error!\n");
	}
	mt_eint_registration(CUST_EINT_CAP_PANEL_NUM, EINTF_TRIGGER_FALLING, iqs_event_handler, 1);
	mt_eint_unmask(CUST_EINT_CAP_PANEL_NUM);

	printk("[zoro] set up interrupt!\n");
		
	/*	Now, assign default values to global variables	*/
	/*	Periodically check for chip reset	*/
	currentState = 0;	/*	Always start at state 0	*/
	setupCounter = 0;	/*	Start initial setup	*/
	doInitialSetup = true;
	initialATI = false;
	eventMode = false;
	touch = false; /*	Assume no touch at first	*/
	reseed = false;
	showReset = false;

	return 0;

err_free_mem:
	//input_free_device(input_dev);
	kfree(data);
	return ret;
}

void iqs263_enter_sleep(struct i2c_client *client)
{
	printk("[zoro] iqs263_enter_sleep!\n");
	u8 dataBuffer[8];
	dataBuffer[0] = PROXSETTINGS0_VAL;
	dataBuffer[1] = PROXSETTINGS1_VAL_SLEEP;
	dataBuffer[2] = PROXSETTINGS2_VAL;
	dataBuffer[3] = PROXSETTINGS3_VAL;
	dataBuffer[4] = EVENT_MASK_VAL_SLEEP;
	i2c_smbus_write_i2c_block_data(client, PROXSETTING_263, 5,
			dataBuffer);

}

void iqs263_out_sleep(struct i2c_client *client)
{
	printk("[zoro] iqs263_out_sleep!\n");
	u8 dataBuffer[8];
	dataBuffer[0] = PROXSETTINGS0_VAL;
	dataBuffer[1] = PROXSETTINGS1_VAL_SLEEP;
	dataBuffer[2] = PROXSETTINGS2_VAL;
	dataBuffer[3] = PROXSETTINGS3_VAL;
	dataBuffer[4] = EVENT_MASK_VAL;
	i2c_smbus_write_i2c_block_data(client, PROXSETTING_263, 5,
			dataBuffer);

}

static int iqs263_sar_remove(struct i2c_client *client)
{
	struct iqs263_sar_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);

	input_unregister_device(data->input_dev);
	kfree(data);

	i2c_set_clientdata(client, NULL);

	return 0;
}

int iqs263_suspend(struct i2c_client * client, pm_message_t mesg)
{
		mt_eint_mask(CUST_EINT_CAP_PANEL_NUM);
		mdelay(5);
		printk("iqs263_suspend\n");
		iqs263_event_mode_handshake();
		iqs263_enter_sleep(client);
		printk("iqs263_suspend ok\n");
		cap_sensor_flag = 0;
		return 0;
}

int iqs263_resume(struct i2c_client * client)
{
		printk("iqs263_resume\n");
		iqs263_event_mode_handshake();
		iqs263_out_sleep(client);
		printk("iqs263_resume ok\n");
		mdelay(5);
		mt_eint_unmask(CUST_EINT_CAP_PANEL_NUM);
		cap_sensor_flag = 0;
		return 0;
}

/*	Standard stucture with the device id for identification	*/
static const struct i2c_device_id iqs263_sar_id[] = {
		{ DEVICE_NAME, 0 },
		{ }
};

MODULE_DEVICE_TABLE(i2c, iqs263_sar_id);

/*
 *	Standard stucture containing the driver
 *	information and procedures
 */
static struct i2c_driver iqs263_sar_driver = {
	.probe = iqs263_sar_probe,
	.remove = iqs263_sar_remove,
	.driver = {.name = DEVICE_NAME,},
	.id_table = iqs263_sar_id,
	.suspend = iqs263_suspend,
	.resume = iqs263_resume,
};

/**
 *	Gets called from 'board-omap3beagle.c'
 *	when the device I2C bus init is called
 */
static int __init iqs263_sar_init(void)
{
	/*	Add i2c driver to kernel	*/
	printk(KERN_ALERT "Installing IQS263 SAR Sensor Driver");
	i2c_register_board_info(IQS_I2C_NUM, &i2c_devs0, 1);
	return i2c_add_driver(&iqs263_sar_driver);
}

/*	Remove the driver */
static void __exit iqs263_sar_exit(void)
{
	/*	Boolean value used for the initial setup	*/
	setupCounter = 0;	/*	Start initial setup	*/
	doInitialSetup = true;
	initialATI = false;

	/*	Reset the IC - or set the proxsettings again	*/
	//gpio_set_value(PWR_LINE_PIN, 0);

	/*	Delete driver	*/
	i2c_del_driver(&iqs263_sar_driver);
	printk(KERN_ALERT "Delete: - %s\n",
			"IQS263 SAR Sensor driver Deleted! ");
}

module_init(iqs263_sar_init);
module_exit(iqs263_sar_exit);

/* Module information */
MODULE_AUTHOR("Alwino van der Merwe <alwino.vandermerwe@azoteq.com>");
MODULE_DESCRIPTION("SAR Sensor driver for Azoteq IQS263");
MODULE_LICENSE("GPL");
