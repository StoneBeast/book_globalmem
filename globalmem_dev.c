/*** 
 * @Author       : stoneBeast
 * @Date         : 2024-12-11 18:10:48
 * @Encoding     : UTF-8
 * @LastEditors  : stoneBeast
 * @LastEditTime : 2024-12-11 18:15:15
 * @Description  : globalmem设备改造后与之匹配的platform_device
 */

#include "linux/init.h"
#include "linux/module.h"
#include "linux/platform_device.h"

static struct platform_device *globalmem_pdev;

static int __init globalmem_dev_init(void)
{
    int ret;

    globalmem_pdev = platform_device_alloc("globalmem", -1);
    if (!globalmem_pdev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(globalmem_pdev);
    if (ret)
    {
        platform_device_put(globalmem_pdev);
        return ret;
    }

    return 0;
}

module_init(globalmem_dev_init);

static void __exit globalmem_dev_exit(void)
{
    platform_device_unregister(globalmem_pdev);
}

module_exit(globalmem_dev_exit);

MODULE_LICENSE("GPL v2");