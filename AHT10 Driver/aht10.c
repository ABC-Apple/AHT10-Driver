/***************************************************************
Copyright (C) 1998-2023 VVX - CN. All rights reserved.
document name		: aht10.c
author	  	        : VVX
version	   	        : V1.0
description	   	    : AHT10 driver
rests	   	        : Imx6ull Linux I2C driver for AHT10 sensor
forum 	   	        : NULL
log	   	            : Incipient Editio V1.0 2023/11/11 vvx create
***************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

/* AHT10 */
#define AHT10_CNT 1
#define AHT10_NAME "aht10"

/* AHT10 command list */
uint8_t Init_Order[]={0xe1,0x08,0x00};
uint8_t Soft_Reset[]={0xba};
uint8_t Measure_Order[]={0xac,0x33,0x00};

struct aht10_dev{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int major;
    void *private_data;
    unsigned int temp;	// temperature reg data - need to be converted
	unsigned int humd;	// humidity reg data - need to be converted
};

static struct aht10_dev aht10dev;

static int aht10_read_regs(struct aht10_dev *dev, u8 reg, void *val, int len)
{
    int ret;
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client*)dev->private_data;

    msg.addr=client->addr;
    msg.flags=I2C_M_RD;			/* i2c recv data */
    msg.buf=val;
    msg.len=len;

    ret = i2c_transfer(client->adapter,&msg,1);
    if (ret==1)
    {
        ret =0;
    }else
    {
        printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
        ret=-EREMOTEIO;
    }
    return ret;
    
    
}

static s32 aht10_write_regs(struct aht10_dev *dev, u8 reg, u8 *buf, u8 len)
{
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client *)dev->private_data;

    memcpy(b,buf,len);

    msg.addr = client->addr;
    msg.flags = 0;              /* i2c send data */
    msg.buf =b;
    msg.len =len;
    return i2c_transfer(client->adapter,&msg,1);
}

static void aht10_write_reg(struct aht10_dev *dev,u8 reg,u8 data)
{
    u8 buf=0;
    buf=data;
    aht10_write_regs(dev,reg,&buf,1);
}

void aht10_readdata(struct aht10_dev *dev)
{
    unsigned char buf[6];

    aht10_write_regs(dev,0x70,Measure_Order,3);
    mdelay(80);

    aht10_read_regs(dev,0x71,buf,6);

    if(!(buf[0]&0x80)){        //0-->normall 1-->busy
        dev->humd = ((buf[1] << 16) | (buf[2] << 8) | buf[3]) >> 4;
		dev->temp = ((buf[3] & 0x0F) << 16) | (buf[4] << 8) | buf[5];
    }

}

static int aht10_open(struct inode *inode,struct file *filp)
{
    mdelay(40);
    filp->private_data = &aht10dev;
    aht10_write_regs(&aht10dev,0x70,Init_Order,3);
    mdelay(40);
    aht10_write_reg(&aht10dev,0x70,0xba);
    mdelay(20);
    return 0;
}

static ssize_t aht10_read(struct file *filp,char __user *buf,size_t cnt,loff_t *off)
{
    unsigned int data[2];
    long err = 0;
    struct aht10_dev *dev = (struct aht10_dev *)filp->private_data;
    aht10_readdata(dev);

    data[0]=dev->temp;
    data[1]=dev->humd;

    err = copy_to_user(buf,data,sizeof(data));
    return 0;
}

static int aht10_release(struct inode *inode,struct file *filp)
{
    return 0;
}

static const struct file_operations aht10_ops = {
    .owner = THIS_MODULE,
    .open = aht10_open,
    .read = aht10_read,
    .release = aht10_release,
};

static int aht10_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    if(aht10dev.major){
        aht10dev.devid = MKDEV(aht10dev.major,0);
        register_chrdev_region(aht10dev.devid,AHT10_CNT,AHT10_NAME);
    }else{
        alloc_chrdev_region(&aht10dev.devid,0,AHT10_CNT,AHT10_NAME);
        aht10dev.major = MAJOR(aht10dev.devid);
    }

    cdev_init(&aht10dev.cdev, &aht10_ops);
	cdev_add(&aht10dev.cdev, aht10dev.devid, AHT10_CNT);

	aht10dev.class = class_create(THIS_MODULE, AHT10_NAME);
	if (IS_ERR(aht10dev.class)) {
		return PTR_ERR(aht10dev.class);
	}

	aht10dev.device = device_create(aht10dev.class, NULL, aht10dev.devid, NULL, AHT10_NAME);
	if (IS_ERR(aht10dev.device)) {
		return PTR_ERR(aht10dev.device);
	}

	aht10dev.private_data = client;
	printk("aht10 major=%d,Device match,driver install completed!\r\n",aht10dev.major);
	return 0;
}

static int aht10_remove(struct i2c_client *client)
{
    cdev_del(&aht10dev.cdev);
    unregister_chrdev_region(aht10dev.devid,AHT10_CNT);
    device_destroy(aht10dev.class,aht10dev.devid);
    class_destroy(aht10dev.class);
    return 0;
}

static const struct of_device_id aht10_of_match[] = {
    {.compatible = "vvx,aht10"},
    {}
};

static const struct i2c_device_id aht10_id[] = {
    {"vvx,aht10",0},
    {}
};

static struct i2c_driver aht10_driver = {
    .probe = aht10_probe,
    .remove = aht10_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "aht10",
        .of_match_table = aht10_of_match,
    },
    .id_table = aht10_id,
};

static int __init aht10_init(void){
    int ret = 0;
    ret = i2c_add_driver(&aht10_driver);
    return ret;
}

static void __exit aht10_exit(void){
    i2c_del_driver(&aht10_driver);
}

module_init(aht10_init);
module_exit(aht10_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("vvx");
