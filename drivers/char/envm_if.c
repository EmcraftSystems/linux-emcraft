/*
 * Device driver for the SmartFusion eNVM
 *
 * (C) Copyright 2011
 * Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include "mss_envm.h"

/*
 * Driver verbosity level: 0->silent; >=1(1..3)->verbose
 */
static int envm_debug = 0;

/*
 * User can change verbosity of the driver
 */
module_param(envm_debug, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(envm_debug, "eNVM driver verbosity level");

static int __init envm_debug_setup(char * str)
{
	get_option(&str, &envm_debug);
	return 1;
}
__setup("envm_debug=", envm_debug_setup);

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)				\
	if (envm_debug >= level) printk(KERN_INFO "%s: " fmt,	\
					__func__, ## args)

/*
 * This is the base of the eNVM region the user can access using
 * this driver. By default, we don't allow accessing 
 * the low part of the eNVM, where U-boot resides.
 */
static ulong envm_base = (1024 * 128);

/*
 * User can change base of the accessible region
 */
module_param(envm_base, ulong, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(envm_base, "Base of the accessible eNVM region");

static int __init envm_base_setup(char * str)
{
	get_option(&str, &envm_base);
	return 1;
}
__setup("envm_base=", envm_base_setup);

/*
 * Device access lock. Only one process can access eNVM at a time
 */
static int envm_lock = 0;

/*
 * Device name
 */
#define ENVM_NAME "envm"

/*
 * Device open
 */
static int envm_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
 	 * One process at a time
 	 */
	if (envm_lock ++ > 0) {
		ret = -EBUSY;
		goto Done;
	}

	/*
 	 * Increment the module use counter
 	 */
	try_module_get(THIS_MODULE);

Done:
	d_printk(2, "lock=%d\n", envm_lock);
	return ret;
}

/*
 * Device close
 */
static int envm_release(struct inode *inode, struct file *file)
{
	/*
 	 * Release device. Other processes can use it now.
 	 */
	envm_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	d_printk(2, "lock=%d\n", envm_lock);
	return 0;
}

/*
 * An auxilary service that performs a read or write access
 */
static ssize_t envm_read_or_write(
	int do_read, struct file *filp, char *buffer,
	size_t length, loff_t * offset) 
{
	unsigned int size;
	unsigned int len = 0;
	unsigned int addr = 0;
	int ret = 0;

	/*
 	 * Check that the user has supplied a valid buffer
 	 */
	if (! access_ok(0, buffer, length)) {
		/*
 		 * TO-DO: is this the right errno?
 		 */
		ret = -EINVAL;
		goto Done;
	}

	addr = envm_base + *offset;
	size = mss_envm_size();

	/*
	 * Check for an EOF condition.
	 */
	if (addr >= size) {
		ret = 0;
		goto Done;
	}

	/*
 	 * Access the required or remaining number of bytes.
 	 */
	len = addr + length < size ? length : size - addr;
	ret = do_read ? mss_envm_read(addr, buffer, len) :
			mss_envm_write(addr, buffer, len);
	*offset += ret;

Done:
	d_printk(3, "do_r=%d,off=%x,addr=%x,len=%d,l=%d,ret=%d\n",
		 do_read, (uint) *offset, addr, length, len, ret);
	return ret;
}

/* 
 * eNVM driver read
 */
static ssize_t envm_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	return envm_read_or_write(1, filp, buffer, length, offset);
}

/* 
 * eNVM driver write
 */
static ssize_t envm_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	return envm_read_or_write(0, filp, (char *) buffer, length, offset);
}

/*
 * Device operations
 */
static struct file_operations envm_fops = {
	.owner = THIS_MODULE,
	.read = envm_read,
	.write = envm_write,
	.open = envm_open,
	.release = envm_release
};

/* 
 * A misc device descriptor
 */
static struct miscdevice envm_dev = {
	A2F_ENVM_MINOR,
	ENVM_NAME,
	&envm_fops
};

/*
 * Initialize the kernel module
 */
static int __init envm_init_module(void)
{
	int ret = 0;

	/*
 	 * Check that envm_base is within the eNVM region
 	 */
	if (! (0 <= envm_base && envm_base < mss_envm_size())) {
		printk(KERN_ALERT "%s: envm_base out of range: %lx\n",
		       __func__, envm_base);
		ret = -EINVAL;
		goto Done;
	}

	/*
	 * Prepare the eNVM
	 */
	mss_envm_init();

	/*
 	 * Register device
 	 */
	ret = misc_register(&envm_dev);
	if (ret) {
		printk(KERN_ALERT "%s: registering device %s with minor %d "
				  "failed with %d\n",
		       __func__, ENVM_NAME, A2F_ENVM_MINOR, ret);
		goto Done;
	}
	
Done:
	d_printk(1, "size=%lx,base=%lx,name=%s,minor=%d\n", 
                 mss_envm_size(), envm_base, ENVM_NAME, A2F_ENVM_MINOR);

	return ret;
}

/* 
 * Kernel module closure
 */
static void __exit envm_cleanup_module(void)
{
	/*
	 * Unregister device
	 */
	misc_deregister(&envm_dev);

	d_printk(1, "%s\n", "clean-up successful");
}

/*
 * Kernel module set-up
 */
module_init(envm_init_module);
module_exit(envm_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Khusainov, vlad@emcraft.com");
MODULE_DESCRIPTION("Driver for SmartFusion eNVM");
