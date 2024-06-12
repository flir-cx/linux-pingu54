/* 
 * File:   fusb30x_driver.c
 * Company: Fairchild Semiconductor
 *
 * Created on September 2, 2015, 10:22 AM
 */

/* Standard Linux includes */
#include <linux/init.h>                                                         // __init, __initdata, etc
#include <linux/module.h>                                                       // Needed to be a module
#include <linux/kernel.h>                                                       // Needed to be a kernel module
#include <linux/i2c.h>                                                          // I2C functionality
#include <linux/slab.h>                                                         // devm_kzalloc
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/errno.h>                                                        // EINVAL, ERANGE, etc
#include <linux/of_device.h>                                                    // Device tree functionality

/* Driver-specific includes */
#include "fusb30x_global.h"                                                     // Driver-specific structures/types
#include "platform_helpers.h"                                                   // I2C R/W, GPIO, misc, etc

#ifdef FSC_DEBUG
#include "../core/core.h"                                                       // GetDeviceTypeCStatus
#endif // FSC_DEBUG

#include "fusb30x_driver.h"

/******************************************************************************
* Driver functions
******************************************************************************/
static int __init fusb30x_init(void)
{
    pr_debug("FUSB  %s - Start driver initialization...\n", __func__);

	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
    pr_debug("FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb302_i2c_resume(struct device* dev)
{
     fusb_platform_resume();
     return 0;
}

static int fusb302_i2c_suspend(struct device* dev)
{
    fusb_platform_suspend();
    return 0;
}


static int fusb30x_probe (struct i2c_client* client,
                          const struct i2c_device_id* id)
{
    int ret = 0;
    struct fusb30x_chip* chip; 
    struct i2c_adapter* adapter;
    struct device *dev = &client->dev;

    if (!client)
    {
        pr_err("Error: Client structure is NULL!\n");
        return -EINVAL;
    }

    /* Make sure probe was called on a compatible device */
    if (!of_match_device(fusb30x_dt_match, &client->dev))
    {
        dev_err(&client->dev, "Error: Device tree mismatch!\n");
        return -EINVAL;
    }

    /* Allocate space for our chip structure (devm_* is managed by the device) */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
       return -ENOMEM;

    chip->client = client;                                                      // Assign our client handle to our chip
    fusb30x_SetChip(chip);                                                      // Set our global chip's address to the newly allocated memory

    /* Initialize the chip lock */
    mutex_init(&chip->lock);

    /* Initialize the chip's data members */
    fusb_InitChipData();
    dev_dbg(dev, "Chip struct data initialized!\n");

    /* Verify that the system has our required I2C/SMBUS functionality (see <linux/i2c.h> for definitions) */
    adapter = to_i2c_adapter(client->dev.parent);
    if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
    {
        chip->use_i2c_blocks = true;
    }
    else
    {
        // If the platform doesn't support block reads, try with block writes and single reads (works with eg. RPi)
        // NOTE: It is likely that this may result in non-standard behavior, but will often be 'close enough' to work for most things
        dev_warn(dev, "Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n");
        if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
        {
            dev_err(&client->dev, "Error: Required I2C/SMBus functionality not supported!\n");
            dev_err(&client->dev, "I2C Supported Functionality Mask: 0x%x\n", i2c_get_functionality(adapter));
            return -EIO;
        }
    }
    dev_dbg(dev, "I2C Functionality check passed! Block reads: %s\n", chip->use_i2c_blocks ? "YES" : "NO");

    /* Assign our struct as the client's driverdata */
    i2c_set_clientdata(client, chip);

    /* Verify that our device exists and that it's what we expect */
    if (!fusb_IsDeviceValid())
    {
        dev_err(dev, "Error: Unable to communicate with device!\n");
        return -EIO;
    }
    dev_dbg(dev, "Device check passed!\n");

    /* reset fusb302*/
    fusb_reset();

    /* Initialize semaphore*/
    sema_init(&chip->suspend_lock, 1);

    /* Initialize the platform's GPIO pins and IRQ */
    ret = fusb_InitializeGPIO();
    if (ret)
    {
        dev_err(&client->dev, "Error: Unable to initialize GPIO!\n");
        return ret;
    }
    dev_dbg(dev, "GPIO initialized!\n");

    ret = fusb_InitializeUSBMux();
    if (ret) {
        dev_err(&client->dev, "Unable to initialize USBMUX!\n");
    } else {
        dev_dbg(&client->dev, "USBMUX GPIO initialized!\n");
    }

//#ifdef FSC_DEBUG
    /* Initialize debug sysfs file accessors */
    fusb_Sysfs_Init();
    dev_dbg(dev, "Sysfs device file created!\n");
//#endif // FSC_DEBUG

#ifdef FSC_INTERRUPT_TRIGGERED
    /* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        dev_err(&client->dev, "Error: Unable to enable interrupts! Error code: %d\n", ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    dev_dbg(dev, "Core is initialized!\n");
#else
    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now) */
    fusb_InitializeCore();
    dev_dbg(dev, "Core is initialized!\n");

    /* Init our workers, but don't start them yet */
    fusb_InitializeWorkers();
    /* Start worker threads after successful initialization */
    fusb_ScheduleWork();
    dev_dbg(dev, "Workers initialized and scheduled!\n");
#endif  // ifdef FSC_POLLING elif FSC_INTERRUPT_TRIGGERED

    dev_info(&client->dev, "FUSB30X Driver loaded successfully!\n");
	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
    pr_debug("FUSB  %s - Removing fusb30x device!\n", __func__);

#ifndef FSC_INTERRUPT_TRIGGERED // Polling mode by default
    fusb_StopThreads();
#endif  // !FSC_INTERRUPT_TRIGGERED

    fusb_GPIO_Cleanup();
    pr_debug("FUSB  %s - FUSB30x device removed from driver...\n", __func__);
    return 0;
}

static void fusb30x_shutdown(struct i2c_client *client)
{
    fusb_reset();
        pr_debug("FUSB	%s - fusb302 shutdown\n", __func__);
}


/*******************************************************************************
 * Driver macros
 ******************************************************************************/
late_initcall(fusb30x_init);                                                    // Defines the module's entrance function
module_exit(fusb30x_exit);                                                      // Defines the module's exit function

MODULE_LICENSE("GPL");                                                          // Exposed on call to modinfo
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");                                 // Exposed on call to modinfo
MODULE_AUTHOR("Fairchild");                        								// Exposed on call to modinfo
