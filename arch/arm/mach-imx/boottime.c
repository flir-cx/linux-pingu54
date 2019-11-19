/*
 *  linux/arch/arm/mach-imx/boottime.c
 *
 */

#include <linux/time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/kobject.h> 
#include <linux/sysfs.h>

#define TSMRB_LOW 0x410A3C08
#define TSMRB_HIGH 0x410A3C0C

static void* timer_low = 0;
static void* timer_high = 0;

static int __init boottime_init (void);
static struct kobject *boottime_kobject;

u64 board_get_time(void)
{
    u64 res;
#ifdef CONFIG_IMX7ULP_BOOTTIME
    u32 low;
    u32 high;
    

    if(timer_low == 0)
        timer_low = ioremap(TSMRB_LOW, 4);
    if(timer_high == 0)
        timer_high = ioremap(TSMRB_HIGH, 4);

    low = readl(timer_low);
    high = readl(timer_high);

    res = (u64) high << 32 | low;
    
#endif
    return res;
}

static ssize_t boottime_show(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
    return sprintf(buf, "%llu\n", board_get_time());
}

static ssize_t boottime_store(struct kobject *kobj, struct kobj_attribute *attr,
                      const char *buf, size_t count)
{
    char a[count+1];
    char *b;

    strcpy(a, buf);
    b = strchr(a, '\n');
    if(b)
        *b = '\0';

    printk("%s boottime %llu\n", a, board_get_time());
    return count;
}

static struct kobj_attribute boottime_attribute =__ATTR(stamp, 0660, boottime_show, boottime_store);

static int __init boottime_init (void)
{
    int error = 0;

    pr_info("kernel_init boottime %llu\n", board_get_time());

    pr_debug("Module initialized successfully \n");

    boottime_kobject = kobject_create_and_add("boottime",
                                                kernel_kobj);
    if(!boottime_kobject)
            return -ENOMEM;

    error = sysfs_create_file(boottime_kobject, &boottime_attribute.attr);
    if (error) {
            pr_debug("failed to create the stamp file in /sys/kernel/boottime \n");
    }

    return error;
}

static void __exit boottime_exit (void)
{
        pr_debug ("Module un initialized successfully \n");
        sysfs_remove_file(boottime_kobject, &boottime_attribute.attr);
        kobject_put(boottime_kobject);
}

module_init(boottime_init);
module_exit(boottime_exit);
