/*! \file sx9310.c
 * \brief  SX9310 Driver
 *
 * Driver for the SX9310 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG
#define DRIVER_NAME "sx9310"

#define MAX_WRITE_ARRAY_SIZE 32
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include "sx9310.h" /* main struct, interrupt,init,pointers */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/time.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>
#include <mach/upmu_common.h>
#include <cust_eint.h>
#include <mach/eint.h>

#define IDLE 0
#define ACTIVE 1
#define sx9310_I2C_NUM 2

//#define GPIO_SX9310_IRQ GPIO19
#define GPIO_SX9310_NIRQ GPIO19
#define SX9310_NIRQ 3
//#define GPIO_SX9310_IRQ_M_EINT  GPIO_MODE_04
static u8 cap_sensor_flag=0;

static DEFINE_MUTEX(sx9310_i2c_access);
static struct i2c_client *new_client = NULL;
static psx93XX_t sx9310 = NULL;
static struct i2c_driver sx9310_driver;

static int initialize(psx93XX_t this);
static void touchProcess(psx93XX_t this);

static int sx9310_get_nirq_state(void)
{
	return !mt_get_gpio_in(GPIO_SX9310_NIRQ);
}

static inline void __init sx9310_gpio_init(void)
{
	/*if ((gpio_request(GPIO_SX9310_NIRQ, "SX9310_NIRQ") == 0) &&
		(gpio_direction_input(GPIO_SX9310_NIRQ) == 0)) {
		gpio_export(GPIO_SX9310_NIRQ, 0);
		s3c_gpio_setpull(GPIO_SX9310_NIRQ,S3C_GPIO_PULL_UP);
		printk(KERN_ERR "obtained gpio for SX9310_NIRQ\n");
		} else {
		printk(KERN_ERR "could not obtain gpio for SX9310_NIRQ\n");
		return;
	}*/
	/*mt_set_gpio_mode(GPIO_SX9310_IRQ, GPIO_SX9310_IRQ_M_EINT);
	mt_set_gpio_dir(GPIO_SX9310_IRQ, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SX9310_IRQ, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SX9310_IRQ, GPIO_PULL_UP);*/
}

static struct _totalButtonInformation smtcButtonInformation = {
	.buttons = psmtcButtons,
	.buttonSize = ARRAY_SIZE(psmtcButtons),
};

/*! \struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9310
{
  pbuttonInformation_t pbuttonInformation;
	psx9310_platform_data_t hw; /* specific platform data settings */
} sx9310_t, *psx9310_t;

static sx9310_platform_data_t sx9310_config = {
  /* Function pointer to get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
  .get_is_nirq_low = sx9310_get_nirq_state,
  /*  pointer to an initializer function. Here in case needed in the future */
  //.init_platform_hw = sx9310_init_ts,
  .init_platform_hw = NULL,
  /*  pointer to an exit function. Here in case needed in the future */
  //.exit_platform_hw = sx9310_exit_ts,
  .exit_platform_hw = NULL,
	
  .pi2c_reg = sx9310_i2c_reg_setup,
  .i2c_reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup),

  .pbuttonInformation = &smtcButtonInformation,
};

static struct i2c_board_info __initdata i2c_devs0  = {
	I2C_BOARD_INFO("sx9310",0x28),
	.flags = I2C_CLIENT_WAKE,
	.irq = SX9310_NIRQ,
	.platform_data = &sx9310_config,
};

static void ForcetoTouched(psx93XX_t this)
{
  psx9310_t pDevice = NULL;
  //struct input_dev *input = NULL;
  struct _buttonInfo *pCurrentButton  = NULL;

  if (this && (pDevice = this->pDevice))
  {
      dev_dbg(this->pdev, "ForcetoTouched()\n");
    
      pCurrentButton = pDevice->pbuttonInformation->buttons;
      //input = pDevice->pbuttonInformation->input;

      //input_report_key(input, pCurrentButton->keycode, 1);
	  cap_sensor_flag = 1;
      pCurrentButton->state = ACTIVE;

      //input_sync(input);

	  dev_dbg(this->pdev, "Leaving ForcetoTouched()\n");
  }
}

int sx9310_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&sx9310_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&sx9310_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&sx9310_i2c_access);    
    return 1;
}


/*! \fn static int write_register(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;
	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c,buffer,2);
	  	dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
	    address,value,returnValue);
	}
	if(returnValue < 0){
		ForcetoTouched(this);
		dev_info( this->pdev, "Write_register-ForcetoTouched()\n");
	}
	return returnValue;
}

/*! \fn static int read_register(psx93XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		if (returnValue >= 0) {
		  *value = returnValue;
		  return 0;
		} else {
		  return returnValue;
		}
	}
	ForcetoTouched(this);
	dev_info( this->pdev, "read_register-ForcetoTouched()\n");
	return -ENOMEM;
}
/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct 
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
static int write_registerEx(psx93XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	u8 tx[MAX_WRITE_ARRAY_SIZE];
	int ret = 0;

	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_dbg(this->pdev, "inside write_registerEx()\n");
		tx[0] = reg;
		dev_dbg(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
		        (void *)i2c,tx[0]);
		for (ret = 0; ret < size; ret++)
		{
		  tx[ret+1] = data[ret];
		  dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
		}
		dev_dbg(this->pdev, "\n");

		ret = i2c_master_send(i2c, tx, size+1 );
		  if (ret < 0)
		  	dev_err(this->pdev, "I2C write error\n");
	}
	dev_dbg(this->pdev, "leaving write_registerEx()\n");


	return ret;
}
/*! \brief Reads a group of registers from the device
* \param this Pointer to main parent struct 
* \param reg 8-Bit address to read from (base address)
* \param data Pointer to 8-bit value array to save registers to 
* \param size size of array
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_registerEx(psx93XX_t this, unsigned char reg,
				unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	int ret = 0;
	u8 tx[] = {
		reg
	};
	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_dbg(this->pdev, "inside read_registerEx()\n");
		dev_dbg(this->pdev,
	    "going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
	                                                           (void *)i2c,(void *)tx,tx[0]);
		ret = i2c_master_send(i2c,tx,1);
		if (ret >= 0) {
	  	dev_dbg(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
	                                                          (void *)i2c,(void *)data,size);
			ret = i2c_master_recv(i2c, data, size);
		}
	}
	if (unlikely(ret < 0))
		dev_err(this->pdev, "I2C read error\n");
	dev_dbg(this->pdev, "leaving read_registerEx()\n");
	return ret;
}
/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;
	returnValue = write_register(this,SX9310_IRQSTAT_REG,0xFF);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);

	dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this,SX9310_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 0, &val))
		return -EINVAL;
	if (val) {
		dev_info( this->pdev, "Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}

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


static DEVICE_ATTR(calibrate, 0666, manual_offset_calibration_show,
                                manual_offset_calibration_store);

static struct attribute *sx9310_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_CapSensor_Data.attr,
	NULL,
};
static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};

static unsigned int sx9310_sysfs_init(void)
{
	int ret;
	struct kobject *sx9310_debug_kobj;
	sx9310_debug_kobj = kobject_create_and_add("CapSensor", NULL) ;
	if (sx9310_debug_kobj == NULL)
	{
		printk("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}
	ret = sysfs_create_file(sx9310_debug_kobj, &dev_attr_CapSensor_Data.attr);
	if (ret)
	{
		printk("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
}


/*********************************************************************/





/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u8 data = 0;
	if (this) {
		if (read_register(this,SX9310_IRQSTAT_REG,&data) == 0)
		  	return (data & 0x00FF);
	}
	return 0;
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx93XX_t this)
{
	u8 data;
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
	{
		while ( i < pdata->i2c_reg_num) {
		  /* Write all registers/values contained in i2c_reg */
		  dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
		            pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
		//      msleep(3);        
		  write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
		  i++;
		}
	} 
	else 
	{
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
		//Force to touched if error
		ForcetoTouched(this);
		dev_info( this->pdev, "Hardware_init-ForcetoTouched()\n");
	}
	read_register(this,SX9310_CPS_CTRL5_REG,&data);
}
/*********************************************************************/

/*! \fn static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0;
	sx9310_sysfs_init();
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pplatData = 0;
	//struct input_dev *input = NULL;
	dev_info(&client->dev, "sx9310_probe()\n");
	
	pplatData = client->dev.platform_data;
	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;
  	sx9310 = kzalloc(sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
	dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%p\n",sx9310);
	if (sx9310)
	{
	/* In case we need to reinitialize data 
	 * (e.q. if suspend reset device) */
	sx9310->init = initialize;
	/* shortcut to read status of interrupt */
	sx9310->refreshStatus = read_regStat;
	/* pointer to function from platform data to get pendown 
	 * (1->NIRQ=0, 0->NIRQ=1) */
	sx9310->get_nirq_low = pplatData->get_is_nirq_low;
	/* save irq in case we need to reference it */
	sx9310->irq = client->irq;
	/* do we need to create an irq timer after interrupt ? */
	sx9310->useIrqTimer = 0;

	/* Setup function to call on corresponding reg irq source bit */
	if (MAX_NUM_STATUS_BITS>= 8)
	{
	  sx9310->statusFunc[0] = 0; /* TXEN_STAT */
	  sx9310->statusFunc[1] = 0; /* UNUSED */
	  sx9310->statusFunc[2] = 0; /* UNUSED */
	  sx9310->statusFunc[3] = 0; /* CONV_STAT */
	  sx9310->statusFunc[4] = 0; /* COMP_STAT */
	  sx9310->statusFunc[5] = touchProcess; /* RELEASE_STAT */
	  sx9310->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
	  sx9310->statusFunc[7] = 0; /* RESET_STAT */
	}

	/* setup i2c communication */
	  sx9310->bus = client;
		new_client = client;
	  i2c_set_clientdata(client, sx9310);

	/* record device struct */
	sx9310->pdev = &client->dev;
	 
	/* create memory for device specific struct */
	sx9310->pDevice = pDevice = kzalloc(sizeof(sx9310_t), GFP_KERNEL);
	  dev_dbg(&client->dev, "\t Initialized Device Specific Memory: 0x%p\n",pDevice);

	if (pDevice)
	{
	  /* for accessing items in user data (e.g. calibrate) */
	  //sysfs_create_group(&client->dev.kobj, &sx9310_attr_group);


	  /* Check if we hava a platform initialization function to call*/
	  if (pplatData->init_platform_hw)
	    pplatData->init_platform_hw();

	  /* Add Pointer to main platform data struct */
	  pDevice->hw = pplatData;
	  
	  /* Initialize the button information initialized with keycodes */
	  pDevice->pbuttonInformation = pplatData->pbuttonInformation;
#if 0
	  /* Create the input device */
	  input = input_allocate_device();
	  if (!input) {
	    return -ENOMEM;
	  }

	  /* Set all the keycodes */
	  __set_bit(EV_KEY, input->evbit);
	  for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
	    __set_bit(pDevice->pbuttonInformation->buttons[i].keycode, 
	                                                    input->keybit);
	    pDevice->pbuttonInformation->buttons[i].state = IDLE;
	  }
	  /* save the input pointer and finish initialization */
	  pDevice->pbuttonInformation->input = input;
	  input->name = "SX9310 Cap Touch";
	  input->id.bustype = BUS_I2C;
	  if(input_register_device(input))
	    return -ENOMEM;
#endif
	}
    sx93XX_init(sx9310);
	u8 a, b;
	read_register(sx9310, SX9310_IRQ_ENABLE_REG, &a);
	sx9310_read_byte(SX9310_IRQ_ENABLE_REG, &b);
	cap_sensor_flag = 0;
    return  0;
  }
  return -1;
}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
	u8 value;
  if (this) {
    /* prepare reset by disabling any irq handling */
    this->irq_disabled = 1;
    disable_irq(this->irq);
	mt_eint_mask(3);
	/* perform a reset */
    write_register(this,SX9310_SOFTRESET_REG,SX9310_SOFTRESET);
    /* wait until the reset has finished by monitoring NIRQ */
    dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
    /* just sleep for awhile instead of using a loop with reading irq status */
	msleep(300);
	//while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
	dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
    hw_init(this);
    msleep(100); /* make sure everything is running */
    manual_offset_calibration(this);
    /* re-enable interrupt handling */
     enable_irq(this->irq);
    mt_eint_unmask(3);
    this->irq_disabled = 0;
   	
    /* make sure no interrupts are pending since enabling irq will only
     * work on next falling edge */
    read_regStat(this);
    dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
    return 0;
  }
  return -ENOMEM;
}

/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
 */
static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u8 i = 0;
	int numberOfButtons = 0;
	psx9310_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	//struct input_dev *input = NULL;

	struct _buttonInfo *pCurrentButton  = NULL;

	if (this && (pDevice = this->pDevice))
	{
		dev_dbg(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX9310_STAT0_REG, &i);

		buttons = pDevice->pbuttonInformation->buttons;
		//input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		/*if (unlikely( (buttons==NULL) || (input==NULL) )) {
			  dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
			  return;
		}*/

		for (counter = 0; counter < numberOfButtons; counter++) {
		  	pCurrentButton = &buttons[counter];
		  	if (pCurrentButton==NULL) {
		    	dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n",
		                                                                  counter);
		    	return; // ERRORR!!!!
		  	}
	  		switch (pCurrentButton->state) {
	    		case IDLE: /* Button is not being touched! */
	      		if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
			        /* User pressed button */
			        printk("cap-sensor button %d touched\n", counter);
			        //input_report_key(input, pCurrentButton->keycode, 1);
					cap_sensor_flag = 1;
			        pCurrentButton->state = ACTIVE;
			      } else {
			        printk("cap-sensor Button %d already released.\n",counter);
			      }
			      break;
		    	case ACTIVE: /* Button is being touched! */ 
			      if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
			        /* User released button */
			        printk("cap-sensor button %d released\n",counter);
			        //input_report_key(input, pCurrentButton->keycode, 0);
					cap_sensor_flag = 0;
			        pCurrentButton->state = IDLE;
			      	} else {
			        	printk("cap-sensor Button %d still touched.\n",counter);
			      	}
		      		break;
		    	default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
		      		break;
	  		};
		}
	//input_sync(input);

	dev_dbg(this->pdev, "Leaving touchProcess()\n");
	}
}

/*! \fn static int sx9310_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
static int sx9310_remove(struct i2c_client *client)
{
	psx9310_platform_data_t pplatData =0;
	psx9310_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);
	if (this && (pDevice = this->pDevice))
	{
		//input_unregister_device(pDevice->pbuttonInformation->input);

		//sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
	  		pplatData->exit_platform_hw();
		kfree(this->pDevice);
	}
	return sx93XX_remove(this);
}

/*====================================================*/
/***** Kernel Suspend *****/
static int sx9310_suspend(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	sx93XX_suspend(this);
	return 0;
}
/***** Kernel Resume *****/
static int sx9310_resume(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	sx93XX_resume(this);
	return 0;
}
/*====================================================*/
static struct i2c_device_id sx9310_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9310_idtable);
static struct i2c_driver sx9310_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME
	},
	.id_table = sx9310_idtable,
	.probe	  = sx9310_probe,
	.remove	  = sx9310_remove,
  	.suspend  = sx9310_suspend,
  	.resume   = sx9310_resume,
};

#ifdef USE_THREADED_IRQ
static void sx93XX_process_interrupt(psx93XX_t this,u8 nirqlow)
{
	int status = 0;
	int counter = 0;
	if (!this) {
		printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
		return;
	}
	/* since we are not in an interrupt don't need to disable irq. */
	status = this->refreshStatus(this);
	counter = -1;
	dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);

	while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
	  	dev_dbg(this->pdev, "Looping Counter %d\n",counter);
		if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
	    	dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
	  		this->statusFunc[counter](this);
		}
	}
	if (unlikely(this->useIrqTimer && nirqlow)) {
	/* In case we need to send a timer for example on a touchscreen
	 * checking penup, perform this here
	 */
		cancel_delayed_work(&this->dworker);
		schedule_delayed_work(&this->dworker,msecs_to_jiffies(this->irqTimeout));
		dev_info(this->pdev,"Schedule Irq timer");
	} 
}


static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	if (work) {
	this = container_of(work,sx93XX_t,dworker.work);
	if (!this) {
	  	printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
	  	return;
	}
	if ((!this->get_nirq_low) || (!this->get_nirq_low())) {
	  /* only run if nirq is high */
	  	sx93XX_process_interrupt(this,0);
		}
	} else {
		printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
	}
}
static irqreturn_t sx93XX_interrupt_thread(int irq, void *data)
{
	psx93XX_t this = 0;
	this = data;
	mutex_lock(&this->mutex);
	dev_dbg(this->pdev, "sx93XX_irq\n");
	if ((!this->get_nirq_low) || this->get_nirq_low()) {
		sx93XX_process_interrupt(this,1);
	}
	else
	  	dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
	mutex_unlock(&this->mutex);
	return IRQ_HANDLED;
}
#else
static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
	unsigned long flags;
	if (this) {
	 	dev_dbg(this->pdev, "sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		//after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
		schedule_delayed_work(&this->dworker,delay);
		spin_unlock_irqrestore(&this->lock,flags);
	}
	else
		printk(KERN_ERR "sx93XX_schedule_work, NULL psx93XX_t\n");
} 

static void sx93XX_irq(void)
{
	psx93XX_t this = 0;
	if (sx9310) {
		this = sx9310;
	  	dev_dbg(this->pdev, "sx93XX_irq\n");
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
	    	dev_dbg(this->pdev, "sx93XX_irq - Schedule Work\n");
	  	sx93XX_schedule_work(this,0);
		}
		else
	    	dev_err(this->pdev, "sx93XX_irq - nirq read high\n");
	}
	else
		printk(KERN_ERR "sx93XX_irq, NULL pvoid\n");
	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
	if (work) {
		this = container_of(work,sx93XX_t,dworker.work);

		if (!this) {
	  		printk(KERN_ERR "sx93XX_worker_func, NULL sx93XX_t\n");
	  		return;
		}
		if (unlikely(this->useIrqTimer)) {
	  		if ((!this->get_nirq_low) || this->get_nirq_low()) {
	    		nirqLow = 1;
	  		}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
		dev_dbg(this->pdev, "Worker - Refresh Status %d\n",status);
		while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
		    dev_dbg(this->pdev, "Looping Counter %d\n",counter);
		  	if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
		      	dev_dbg(this->pdev, "Function Pointer Found. Calling\n");
		    	this->statusFunc[counter](this);
		  	}
		}
		if (unlikely(this->useIrqTimer && nirqLow))
		{ 
			/* Early models and if RATE=0 for newer models require a penup timer */
		  	/* Queue up the function again for checking on penup */
		  	sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		printk(KERN_ERR "sx93XX_worker_func, NULL work_struct\n");
	}
}
#endif




void sx93XX_suspend(psx93XX_t this)
{
  	if (this)
    	disable_irq(this->irq);
    mt_eint_mask(3);
}
void sx93XX_resume(psx93XX_t this)
{
	if (this) {
#ifdef USE_THREADED_IRQ
	mutex_lock(&this->mutex);
	/* Just in case need to reset any uncaught interrupts */
	sx93XX_process_interrupt(this,0);
	mutex_unlock(&this->mutex);
#else
	sx93XX_schedule_work(this,0);
#endif
	if (this->init)
	  this->init(this);
	enable_irq(this->irq);
	mt_eint_unmask(3);
	u8 tmp;
	tmp=read_regStat(this);
	}
}

#if 0//def CONFIG_HAS_WAKELOCK
/*TODO: Should actually call the device specific suspend/resume
 * As long as the kernel suspend/resume is setup, the device
 * specific ones will be called anyways
 */
extern suspend_state_t get_suspend_state(void);
void sx93XX_early_suspend(struct early_suspend *h)
{
	psx93XX_t this = 0;
  dev_dbg(this->pdev, "inside sx93XX_early_suspend()\n");
	this = container_of(h, sx93XX_t, early_suspend);
  sx93XX_suspend(this);
  dev_dbg(this->pdev, "exit sx93XX_early_suspend()\n");
}

void sx93XX_late_resume(struct early_suspend *h)
{
	psx93XX_t this = 0;
  dev_dbg(this->pdev, "inside sx93XX_late_resume()\n");
	this = container_of(h, sx93XX_t, early_suspend);
  sx93XX_resume(this);
  dev_dbg(this->pdev, "exit sx93XX_late_resume()\n");
}
#endif

int sx93XX_init(psx93XX_t this)
{
	int err = 0;
	if (this && this->pDevice)
	{
#ifdef USE_THREADED_IRQ

		/* initialize worker function */
	  	INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);


		/* initialize mutex */
		mutex_init(&this->mutex);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
	  	err = request_threaded_irq(this->irq, NULL, sx93XX_interrupt_thread,
	                          IRQF_TRIGGER_FALLING, this->pdev->driver->name,
	                          this);
	  	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, sx93XX_interrupt_thread, 1);
#else
		/* initialize spin lock */
		spin_lock_init(&this->lock);

		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);

		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		//err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,this->pdev->driver->name, this);

#endif
		if (err) {
		  	dev_err(this->pdev, "irq %d busy?\n", this->irq);
		  	return err;
		}
#ifdef USE_THREADED_IRQ
		dev_info(this->pdev, "registered with threaded irq (%d)\n", this->irq);
#else
		dev_info(this->pdev, "registered with irq (%d)\n", this->irq);
#endif
#if 0//def CONFIG_HAS_WAKELOCK	
		this->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		this->early_suspend.suspend = sx93XX_early_suspend;
		this->early_suspend.resume = sx93XX_late_resume;
		register_early_suspend(&this->early_suspend);
		if (has_wake_lock(WAKE_LOCK_SUSPEND) == 0 && 
	                        get_suspend_state() == PM_SUSPEND_ON)
	 	sx93XX_early_suspend(&this->early_suspend);
#endif //CONFIG_HAS_WAKELOCK
		/* call init function pointer (this should initialize all registers */

		if (this->init)
			this->init(this);
		mt_eint_mask(CUST_EINT_CAP_PANEL_NUM);
		mt_eint_registration(CUST_EINT_CAP_PANEL_NUM, EINTF_TRIGGER_FALLING, sx93XX_irq, 1);
		mt_eint_unmask(CUST_EINT_CAP_PANEL_NUM);
		return 1;
		dev_err(this->pdev,"No init function!!!!\n");
	}
	return -ENOMEM;
}

int sx93XX_remove(psx93XX_t this)
{
  if (this) {
    cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
    /*destroy_workqueue(this->workq); */
#if 0//def CONFIG_HAS_WAKELOCK
    unregister_early_suspend(&this->early_suspend);
#endif
    free_irq(this->irq, this);
    kfree(this);
    return 0;
  }
  return -ENOMEM;
}

static int __init sx9310_init(void)
{
	sx9310_gpio_init();
	i2c_register_board_info(sx9310_I2C_NUM, &i2c_devs0, 1);
	return i2c_add_driver(&sx9310_driver);
}
static void __exit sx9310_exit(void)
{
	i2c_del_driver(&sx9310_driver);
}

module_init(sx9310_init);
module_exit(sx9310_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9310 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

