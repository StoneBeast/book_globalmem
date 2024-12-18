/*** 
 * @Author       : stoneBeast
 * @Date         : 2024-12-06 15:02:51
 * @Encoding     : UTF-8
 * @LastEditors  : stoneBeast
 * @LastEditTime : 2024-12-12 09:41:23
 * @Description  : 《linux设备驱动开发详解》中的globalmem驱动程序
 */

#include "linux/container_of.h"
#include "linux/device.h"
#include "linux/device/class.h"
#include "linux/errno.h"
#include "linux/miscdevice.h"
#include "linux/mutex.h"
#include "linux/platform_device.h"
#include "linux/poll.h"
#include "linux/sched.h"
#include "linux/sched/signal.h"
#include "linux/stddef.h"
#include "linux/wait.h"
#include <linux/cdev.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/kdev_t.h>
#include <linux/kern_levels.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#define GLOBALMEM_MAJOR     230
#define GLOBALMEM_SIZE      0x1000
#define MEM_CLEAR           0x01
#define DEVICE_NUM          1

static int globalmem_major = GLOBALMEM_MAJOR;
module_param(globalmem_major, int, S_IRUGO);

struct globalmem_dev {
    struct cdev cdev;
    unsigned char mem[GLOBALMEM_SIZE];
    unsigned int current_len;
    struct mutex mutex;
    wait_queue_head_t r_wait;
    wait_queue_head_t w_wait;
    struct fasync_struct *async_queue;
    struct miscdevice miscdev;
};

static struct globalmem_dev *globalmem_devp;

static int globalmem_open(struct inode *inode, struct file *filp)
{
    /* linux在调用 misc_open()时,会将miscdev赋值给filp->private_data */
    //  struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);

    // filp->private_data = dev;
    return 0;
}

static long globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);

    switch (cmd) {
    case MEM_CLEAR:
        
        mutex_lock(&dev->mutex);
        
        memset(dev->mem, 0, GLOBALMEM_SIZE);
        
        mutex_unlock(&dev->mutex);
        printk(KERN_INFO "globalmem si set to zero\n");
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

static ssize_t globalmem_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    unsigned int count = size;
    int ret = 0;
    struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);
    DECLARE_WAITQUEUE(wait, current);

    mutex_lock(&dev->mutex);
    add_wait_queue(&dev->r_wait, &wait);

    printk("miscname: %s\n", dev->miscdev.name);

    while (dev->current_len == 0)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }
        __set_current_state(TASK_INTERRUPTIBLE);
        mutex_unlock(&dev->mutex);

        schedule();
        if(signal_pending(current))
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&dev->mutex);
    }

    if (count > dev->current_len)
    {
        count = dev->current_len;
    }

    if (copy_to_user(buf, dev->mem, count)) {
        ret = -EFAULT;
        goto out;
    } else {
        memcpy(dev->mem, dev->mem+count, dev->current_len-count);
        dev->current_len -= count;
        ret = count;

        printk(KERN_INFO "read %u bytes, current_len: %u\n", count, dev->current_len);
        wake_up_interruptible(&dev->w_wait);
    }

out:
    mutex_unlock(&dev->mutex);
out2:
    remove_wait_queue(&dev->r_wait, &wait);
    set_current_state(TASK_RUNNING);
    return ret;
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    unsigned int count = size;
    int ret = 0;
    struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);
    DECLARE_WAITQUEUE(wait, current);
    
    mutex_lock(&dev->mutex);
    add_wait_queue(&dev->w_wait, &wait);

    while (dev->current_len == GLOBALMEM_SIZE)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }
        __set_current_state(TASK_INTERRUPTIBLE);
        mutex_unlock(&dev->mutex);

        schedule();
        if(signal_pending(current)) /* 因为被信号唤醒，而不是可写 */
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&dev->mutex);
    }

    if (count > GLOBALMEM_SIZE - dev->current_len)
    {
        count = GLOBALMEM_SIZE - dev->current_len;
    }

    if (copy_from_user(dev->mem+dev->current_len, buf, count))
    {
        ret = -EFAULT;
        goto out;
    }
    else 
    {
        dev->current_len += count;
        printk( KERN_INFO "written %d bytes, current_len: %d\n", count, dev->current_len);
        wake_up_interruptible(&dev->r_wait);

        if (dev->async_queue)
        {
            /*产生异步信号: 有数据io时，发送SIGIO信号,有数据写入设置POLL_IN(可读) */
            kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
            printk(KERN_DEBUG "%s kill SIGIO\n", __func__);
        }

        ret = count;
    }
    
out:
    mutex_unlock(&dev->mutex);
out2:
    remove_wait_queue(&dev->w_wait, &wait);
    set_current_state(TASK_RUNNING);
    return ret;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset, int orig)
{
    loff_t ret = 0;

    switch (orig) {
        case 0:
            if (offset < 0) {
                ret = -EINVAL;
                break;
            }
            if ((unsigned int) offset > GLOBALMEM_SIZE) {
                ret = -EINVAL;
                break;
            }
            filp->f_pos = (unsigned int)offset;
            ret = filp->f_pos;
            break;

        case 1:
            if ((filp->f_pos + offset) > GLOBALMEM_SIZE) {
                ret = -EINVAL;
                break;
            }
            if ((filp->f_pos + offset) > 0) {
                ret = -EINVAL;
                break;
            }
            filp->f_pos += offset;
            ret = filp->f_pos;
            
            break;
        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static __poll_t globalmem_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int mask = 0;
    struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);

    mutex_lock(&dev->mutex);
    poll_wait(filp, &dev->r_wait, wait);
    poll_wait(filp, &dev->w_wait, wait);

    if (dev->current_len != 0)
    {
        mask |= POLLIN | POLLRDNORM;
    }

    if (dev->current_len != GLOBALMEM_SIZE)
    {
        mask |= POLLOUT | POLLWRNORM;
    }

    mutex_unlock(&dev->mutex);
    return mask;
}

int globalmem_fasync(int fd, struct file *filp, int mode)
{
    struct globalmem_dev *dev = container_of(filp->private_data, struct globalmem_dev, miscdev);

    /* 处理FASYNC标志变更的函数(初始化struct fasync_struct结构体) */
    return fasync_helper(fd, filp, mode, &dev->async_queue);
}

static int globalmem_release(struct inode *inode, struct file *filp)
{
    globalmem_fasync(-1, filp, 0);
    return 0;
}

static const struct file_operations globalmem_fops = {
    .owner = THIS_MODULE,
    .llseek = globalmem_llseek,
    .read = globalmem_read,
    .write = globalmem_write,
    .unlocked_ioctl = globalmem_ioctl,
    .open = globalmem_open,
    .release = globalmem_release,
    .poll = globalmem_poll,
    .fasync = globalmem_fasync,
};

static int globalmem_probe(struct platform_device *pdev)
{
    int ret;

    /* 申请globalmem_dev空间 */
    globalmem_devp = kzalloc(sizeof(struct globalmem_dev) * DEVICE_NUM, GFP_KERNEL);
    if (!globalmem_devp) {
        ret = -ENOMEM;
        return ret;
    }

    globalmem_devp->miscdev.minor = MISC_DYNAMIC_MINOR;
    globalmem_devp->miscdev.name = "globalmem";
    globalmem_devp->miscdev.fops = &globalmem_fops;

    mutex_init(&globalmem_devp->mutex);
    /* 初始化等待队列 */
    init_waitqueue_head(&globalmem_devp->r_wait);
    init_waitqueue_head(&globalmem_devp->w_wait);

    ret = misc_register(&globalmem_devp->miscdev);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

static int globalmem_remove(struct platform_device *pdev)
{
    struct globalmem_dev *gl = platform_get_drvdata(pdev);
    misc_deregister(&gl->miscdev);
    
    return 0;
}

static struct platform_driver globalmem_dirver = {
    .driver = {
        .name = "globalmem",
        .owner = THIS_MODULE,
    },
    .probe = globalmem_probe,
    .remove = globalmem_remove,
};

module_platform_driver(globalmem_dirver);

MODULE_LICENSE("GPL v2");
