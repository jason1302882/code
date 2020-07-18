/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/delay.h>


static struct input_dev *sinput_dev;
static struct delayed_work gpio_work;


static int key_v1;
static int key_v2;
static int key_v3;
static int key_v4;
static int key_v5;

static int w1_l1_flag = -1;
static int w1_l2_flag = -1;
static int w3_l1_flag = -1;
static int w3_l2_flag = -1;



int read_l_gpio(void);
void set_w1_gpio_low(void)
{
	int ret;
	gpio_direction_output(key_v3, 0);
	gpio_direction_output(key_v4, 1);
	gpio_direction_output(key_v5, 1);
	ret = read_l_gpio();
	if(ret == 0)
	{
		//printk("no key press\n");
		w1_l1_flag = 0;
		w1_l2_flag = 0;
	}
	else if(ret == 1)
	{
		//printk("key w1_l2 press\n");
		if(w1_l2_flag == 0)
		{
			input_event(sinput_dev, EV_KEY, KEY_F8, 1);
			input_sync(sinput_dev);
			input_event(sinput_dev, EV_KEY, KEY_F8, 0);
			input_sync(sinput_dev);
			w1_l2_flag = 1;
		}

	}
	else if(ret == 10)
	{
		//printk("key w1_l1 press\n");
		if(w1_l1_flag == 0)
		{
			input_event(sinput_dev, EV_KEY, KEY_BACK, 1);
			input_sync(sinput_dev);
			msleep(50);
			input_event(sinput_dev, EV_KEY, KEY_BACK, 0);
			input_sync(sinput_dev);	
			w1_l1_flag = 1;
		}
	}
	else if(ret == 11)
	{
		//printk("key w1_l1 and w1_l2 press\n");
	#if 0
		input_event(sinput_dev, EV_KEY, KEY_F8, 1);
		input_sync(sinput_dev);
		input_event(sinput_dev, EV_KEY, KEY_F8, 0);
		input_sync(sinput_dev);	

		input_event(sinput_dev, EV_KEY, KEY_BACK, 1);
		input_sync(sinput_dev);
		input_event(sinput_dev, EV_KEY, KEY_BACK, 0);
		input_sync(sinput_dev);	
	#endif
	}
	else
	{
		//printk("error key \n");
	}
}
void set_w2_gpio_low(void)
{
	int ret;
	gpio_direction_output(key_v3, 1);
	gpio_direction_output(key_v4, 0);
	gpio_direction_output(key_v5, 1);
	ret = read_l_gpio();
	if(ret == 0)
	{
		//printk("no key press\n");
	}
	else if(ret == 1)
	{
		//printk("key w2_l2 press\n");
	}
	else if(ret == 10)
	{
		//printk("key w2_l1 press\n");
	}
	else if(ret == 11)
	{
		//printk("key w2_l1 and w2_l2 press\n");
	}
	else
	{
		//printk("error key \n");
	}

}
void set_w3_gpio_low(void)
{
	int ret;
	gpio_direction_output(key_v3, 1);
	gpio_direction_output(key_v4, 1);
	gpio_direction_output(key_v5, 0);
	ret = read_l_gpio();
	if(ret == 0)
	{
		//printk("no key press\n");
		w3_l1_flag = 0;
		w3_l2_flag = 0;
	}
	else if(ret == 1)
	{
		printk("key w3_l2 press\n");
		if(w3_l2_flag == 0)
		{
			input_event(sinput_dev, EV_KEY, KEY_HOMEPAGE, 1);
			input_sync(sinput_dev);
			input_event(sinput_dev, EV_KEY, KEY_HOMEPAGE, 0);
			input_sync(sinput_dev);
			w3_l2_flag = 1;
		}
	}
	else if(ret == 10)
	{
		//printk("key w3_l1 press\n");
		if(w3_l1_flag == 0)
		{
			input_event(sinput_dev, EV_KEY, KEY_F7, 1);
			input_sync(sinput_dev);
			input_event(sinput_dev, EV_KEY, KEY_F7, 0);
			input_sync(sinput_dev);
			w3_l1_flag = 1;
		}
	}
	else if(ret == 11)
	{
		//printk("key w3_l1 and w3_l2 press\n");
	#if 0
		input_event(sinput_dev, EV_KEY, KEY_F7, 1);
		input_sync(sinput_dev);
		input_event(sinput_dev, EV_KEY, KEY_F7, 0);
		input_sync(sinput_dev);	

		input_event(sinput_dev, EV_KEY, KEY_HOMEPAGE, 1);
		input_sync(sinput_dev);
		input_event(sinput_dev, EV_KEY, KEY_HOMEPAGE, 0);
		input_sync(sinput_dev);	
	#endif
	}
	else
	{
		//printk("error key \n");
	}


}

int read_l_gpio(void)
{
	gpio_direction_input(key_v1);
	gpio_direction_input(key_v2);

	if ((gpio_get_value(key_v1) == 1 ) && (gpio_get_value(key_v2) == 1))
	{
		return 0;
	}
	else if ((gpio_get_value(key_v1) == 0) && (gpio_get_value(key_v2) == 1))
	{
		//printk("l1 have key press\n");
		msleep(8);
		if((gpio_get_value(key_v1) == 0) && (gpio_get_value(key_v2) == 1))
			return 10;
		else
			return 0;
	}
	else if ((gpio_get_value(key_v1) == 1) && (gpio_get_value(key_v2) == 0))
	{
		//printk("l2 have key press\n");
		msleep(8);
		if((gpio_get_value(key_v1) == 1) && (gpio_get_value(key_v2) == 0))
			return 1;
		else
			return 0;
	}
	else if ((gpio_get_value(key_v1) == 0) && (gpio_get_value(key_v2) == 0))
	{
		//printk("l1 and l2 have key press\n");
		msleep(8);
		if((gpio_get_value(key_v1) == 0) && (gpio_get_value(key_v2) == 0))
			return 11;
		else
			return 0;
	}

	return -1;
}

void read_juzhen_gpio_function(void)
{
	set_w1_gpio_low();
	//set_w2_gpio_low();
	set_w3_gpio_low();
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	read_juzhen_gpio_function();
	schedule_delayed_work(&gpio_work, 8);
}

#ifdef CONFIG_OF
static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "juzhen-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);
#endif
static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct input_dev *input;
	int ret, error;

	printk("cjc func: %s\n", __func__); 
	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}


	input->name = "incar-keypad";
	input->phys = "gpio-keys/input1";
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_remove_group;
	}

	sinput_dev = input;		
	input_set_capability(input, EV_KEY, 158); //KEY_BACK		
	input_set_capability(input, EV_KEY, 172); //KEY_HOMEPAGE
	input_set_capability(input, EV_KEY, 64);  //KEY_F6
	input_set_capability(input, EV_KEY, 65);  //KEY_F7	
	input_set_capability(input, EV_KEY, 66);  //KEY_F8


	key_v1 = of_get_named_gpio(node, "sprd,key-v1-gpio", 0);
	if (key_v1 < 0) {
		printk("invalid key_v1 in dt: %d", key_v1);
		return -EINVAL;
	}
	
	ret = devm_gpio_request(&pdev->dev, key_v1, "key_v1");
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request GPIO%d for key_v1\n",
			key_v1);
		return -EINVAL;
	}
	printk("set gpio direction_input cjc v1###");
	gpio_direction_input(key_v1);

	key_v2 = of_get_named_gpio(node, "sprd,key-v2-gpio", 0);
	if (key_v2 < 0) {
		printk("invalid key_v2 in dt: %d", key_v2);
		return -EINVAL;
	}
	
	ret = devm_gpio_request(&pdev->dev, key_v2, "key_v2");
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request GPIO%d for key_v2\n",
			key_v2);
		return -EINVAL;
	}
	printk("set gpio direction_input cjc key_v2###");
	gpio_direction_input(key_v2);

	key_v3 = of_get_named_gpio(node, "sprd,key-v3-gpio", 0);
	if (key_v3 < 0) {
		printk("invalid key_v3 in dt: %d", key_v3);
		return -EINVAL;
	}
	
	ret = devm_gpio_request(&pdev->dev, key_v3, "key_v3");
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request GPIO%d for key_v3\n",
			key_v3);
		return -EINVAL;
	}
	printk("set gpio direction_input cjc key_v3###");
	gpio_direction_input(key_v3);


	key_v4 = of_get_named_gpio(node, "sprd,key-v4-gpio", 0);
	if (key_v4 < 0) {
		printk("invalid key_v4 in dt: %d", key_v4);
		return -EINVAL;
	}
	
	ret = devm_gpio_request(&pdev->dev, key_v4, "key_v4");
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request GPIO%d for key_v4\n",
			key_v4);
		return -EINVAL;
	}
	printk("set gpio direction_input cjc key_v4###");
	gpio_direction_input(key_v4);

	key_v5 = of_get_named_gpio(node, "sprd,key-v5-gpio", 0);
	if (key_v5 < 0) {
		printk("invalid key_v5 in dt: %d", key_v5);
		return -EINVAL;
	}
	
	ret = devm_gpio_request(&pdev->dev, key_v5, "key_v5");
	if (ret) {
		dev_err(&pdev->dev,
			"failed to request GPIO%d for key_v5\n",
			key_v5);
		return -EINVAL;
	}
	printk("set gpio direction_input cjc key_v5###");
	gpio_direction_input(key_v5);



	INIT_DELAYED_WORK(&gpio_work, gpio_keys_gpio_work_func);
	schedule_delayed_work(&gpio_work, 10);

	
	return 0;

err_remove_group:
	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "juzhen-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("cjc <chengjc@incartech.cn>");
MODULE_DESCRIPTION("juzhen Keyboard driver for sprd");
MODULE_ALIAS("platform:sprd");
