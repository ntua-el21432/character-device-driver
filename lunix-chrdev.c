/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Vasileios Oikonomou and Vasileios Kotronis
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	WARN_ON (!(sensor = state->sensor));
	if(sensor->msr_data[state->type]->last_update != state->buf_timestamp){
		printk(KERN_DEBUG"refresh state\n");
		return 1;
	}
	return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	int ret=0;
	uint32_t raw_value;
	uint32_t timestamp;
	long int cooked_value;
	sensor=state->sensor;
	WARN_ON(!sensor);
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	if(!lunix_chrdev_state_needs_refresh(state)) return -EAGAIN;
	spin_lock_irq(&sensor->lock);
	raw_value=sensor->msr_data[state->type]->values[0];
	timestamp=sensor->msr_data[state->type]->last_update;
	spin_unlock_irq(&sensor->lock);
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	state->buf_timestamp=timestamp;
	if(state->type==BATT)
                {cooked_value=lookup_voltage[raw_value];}
        else if(state->type==LIGHT)
                {cooked_value=lookup_light[raw_value];}
        else if(state->type==TEMP)
                {cooked_value=lookup_temperature[raw_value];}
        else
        {
                debug("type must be BATT, TEMP or LIGHT");
                ret=-EMEDIUMTYPE;
                goto out;
        }
        state->buf_lim=snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ,\
                                "%ld.%03ld\n",cooked_value/1000,cooked_value%1000);
	out:
		debug("leaving\n");
		return ret;
}
/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	unsigned int minor, sensor, type;
	struct lunix_chrdev_state_struct *state;
	int ret=0;

	debug("entering open\n");
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */

	/* Allocate a new Lunix character device private state structure */
	minor=iminor(inode);
	type=minor%8; //type of measurement;
	sensor=minor/8;
	state=kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(!state)
	{
		ret=-ENOMEM;
		debug("error allocating memory\n");
		goto out;
	}
	//init state
	state->type=type;
	state->sensor=&lunix_sensors[sensor];
	state->buf_timestamp=0;
	state->buf_lim=0;
	//state->raw_data=0; //cooked by default
	sema_init(&state->lock,1); //init semaphore with 1-available
	filp->private_data=state;
out:
	debug("leaving with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	filp->private_data=NULL;
	debug("released memory succesfully\n");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret=0;
	ssize_t rem_bytes;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	debug("entering read\n");
	/* Lock? */
	if(down_interruptible(&state->lock))
		return -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			up(&state->lock);
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			if(filp->f_flags & O_NONBLOCK)// if file was opened with O_NONBLOCK return -EAGAIN
				return -EAGAIN;
			if(wait_event_interruptible(sensor->wq , lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS;
			if(down_interruptible(&state->lock))
				return -ERESTARTSYS;
		}
	}
	rem_bytes=state->buf_lim - *f_pos; //bytes to copy
	if(rem_bytes<0) rem_bytes=0;
	if(cnt>rem_bytes) cnt=rem_bytes;
	if(cnt==0) goto out; //end of file
	if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt))
	{
		ret=-EFAULT;
		goto out;
	}
	/* Auto-rewind on EOF mode? */
	*f_pos+=cnt;
	ret=cnt;
	if(*f_pos>=state->buf_lim) *f_pos=0;

out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = {
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* register_chrdev_region? */
	ret=register_chrdev_region(dev_no,lunix_minor_cnt, "Lunix;TNG");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* cdev_add? */
	ret= cdev_add(&lunix_chrdev_cdev,dev_no,lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
