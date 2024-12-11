/*** 
 * @Author       : stoneBeast
 * @Date         : 2024-12-10 17:20:15
 * @Encoding     : UTF-8
 * @LastEditors  : stoneBeast
 * @LastEditTime : 2024-12-11 10:12:51
 * @Description  : 《linux设备驱动开发详解》中second驱动
 */

#include "linux/cdev.h"
#include "linux/device.h"
#include "linux/device/class.h"
#include "linux/export.h"
#include "linux/fs.h"
#include "linux/gfp.h"
#include "linux/init.h"
#include "linux/jiffies.h"
#include "linux/kdev_t.h"
#include "linux/kern_levels.h"
#include "linux/module.h"
#include "linux/moduleparam.h"
#include "linux/printk.h"
#include "linux/slab.h"
#include "linux/stat.h"
#include "linux/stddef.h"
#include "linux/timer.h"
#include "linux/types.h"

#define SECOND_MAJOR    248
static int second_major = SECOND_MAJOR;
module_param(second_major, int, S_IRUGO);

struct second_dev {
    struct cdev cdev;
    atomic_t counter;
    struct timer_list s_timer;
};

static struct second_dev *second_devp;
static struct class *second_class;
static struct device *second_dev;

static void second_timer_handler(struct timer_list *arg)
{
    mod_timer(&second_devp->s_timer, jiffies+HZ);
    atomic_inc(&second_devp->counter);

    printk( KERN_INFO "surrent jiffies si %ld\n", jiffies);
}

static int second_open(struct inode *inode, struct file *filp)
{
    timer_setup(&second_devp->s_timer, &second_timer_handler, 0);
    second_devp->s_timer.expires = jiffies + HZ;

    add_timer(&second_devp->s_timer);
    atomic_set(&second_devp->counter, 0);

    return 0;
}

static ssize_t second_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    int counter;

    counter = atomic_read(&second_devp->counter);
    if (put_user(counter, (int *)buf))
    {
        return -EFAULT;
    }
    else
    {
        return sizeof(unsigned int);
    }
}

static int second_release(struct inode *inode, struct file *filp)
{
    del_timer(&second_devp->s_timer);

    return 0;
}

static const struct file_operations second_fops = {
    .owner = THIS_MODULE,
    .open = second_open,
    .read = second_read,
    .release = second_release,
};

static void second_setup_cdev(struct second_dev *dev, int index)
{
    int err, devno = MKDEV(second_major, index);

    cdev_init(&dev->cdev, &second_fops);
    dev->cdev.owner = THIS_MODULE;
    err = cdev_add(&dev->cdev, devno, 1);

    if (err)
    {
        printk(KERN_ERR "Failed to add seond device\n");
    }
}

static int __init second_init(void)
{
    int ret;
    dev_t devno = MKDEV(second_major, 0);

    if (second_major)
    {
        ret = register_chrdev_region(devno, 1, "second");
    }
    else
    {
        ret = alloc_chrdev_region(&devno, 0, 1, "second");
        second_major = MAJOR(devno);
    }

    if (ret < 0)
    {
        return ret;
    }

    second_devp = kzalloc(sizeof(struct second_dev), GFP_KERNEL);
    if (!second_devp)
    {
        ret = -ENOMEM;
        goto fail_malloc;
    }

    second_setup_cdev(second_devp, 0);

    second_class = class_create(THIS_MODULE, "second_class");
    second_dev = device_create(second_class, NULL, devno, NULL, "second0");

    return 0;

fail_malloc:
    unregister_chrdev_region(devno, 1);
    return ret;
}

module_init(second_init);

static void __exit second_exit(void)
{
    device_destroy(second_class, MKDEV(second_major, 0));
    class_destroy(second_class);

    cdev_del(&second_devp->cdev);
    kfree(second_devp);
    unregister_chrdev_region(MKDEV(second_major, 0), 1);
}
module_exit(second_exit);

MODULE_LICENSE("GPL v2");
