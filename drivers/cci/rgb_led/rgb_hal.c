
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "rgb_hal.h"

#include "rgb_hw.h"
#include "myLog.h"
#include "rgb_interface.h"

// To disable illumination LED in 58 project.
#include <mach/cci_hw_id.h>

#define CCI_CONTROL_PIN_STRING "cci,control_gpio"

static int __init rgb_init(void);
static void __exit rgb_exit(void);

module_init(rgb_init);
module_exit(rgb_exit);

// module init entry.
static struct platform_driver mDriver;
static int rgb_probe(struct platform_device *);

static int __init rgb_init(void) {
	int ret = 0;
	Fin();
	if (get_cci_project_id() == CCI_PROJECTID_VY58_59 ) {
		Fout("Ignore 58/59 project, would not initialize illumination LED.");
		return -ENODEV;
	}
	ret = platform_driver_probe(&mDriver, rgb_probe);
	Fout("ret: %d", ret);
	return ret;
}

static void __exit rgb_exit(void) {
    Ftr();
    hw_sendData(0, 0x07); 
	// Clean and reset the reset/enable reg#0
    if_release();
}


static int __init rgb_probe(struct platform_device *pdev) {

    u32 control_pin;
    struct device_node *node = NULL;
	int res = 0;
    Fin();
    if (pdev==NULL) {
        Fout("No platform device?");
        return -ENODEV;
    }
    if ( &(pdev->dev)==NULL) {
        Fout("Device not found!");
        return -ENODEV;
    }
	node = pdev->dev.of_node;
	of_property_read_u32(node, CCI_CONTROL_PIN_STRING, &control_pin);
    if (1) {
        rgb_chip_layout layout;
        layout.gpio_CTRL = control_pin; // default: 0
        res = hw_requestGPIO(&layout);
    }
	if (res<0) {
		Fout("Can not config gpio, res: %d", res);
		return res;
	}
    hw_wakeup();
	hw_sendData(0, 0x0f); // Reset the chip to "CTRL toggling" mode.
	if_create();
	Fout();
	return 0;
}

// Define the platform_driver data  member
static struct of_device_id rgb_dt_match[] = {
    { .compatible = "kinet,ktd2024", },
    {}
};

static int rgb_dev_pm_suspend(struct device *dev);
static int rgb_dev_pm_resume(struct device *dev);
static struct dev_pm_ops rgb_dev_pm_ops = {
    .suspend = &rgb_dev_pm_suspend,
    .resume = &rgb_dev_pm_resume,
};

static struct platform_driver mDriver = {
    .driver = {
        .name = "rgb_led", // DRIVER_NAME
        .owner = THIS_MODULE,
        .of_match_table = rgb_dt_match,
#ifdef CONFIG_PM
        .pm = &rgb_dev_pm_ops,
#endif
    },
};

static int rgb_dev_pm_suspend(struct device *dev) {
    if ( hw_trySuspend() < 0) {
        Ftr("Can not suspend");
        return 0;
    }
//  if_release(); // TODO: must keep sync with attr_r/w and interface exist.
    Fout("Sleep...");
    return 0;
}

// No need to implement state machine, because current state can be check by gpio#0
static int rgb_dev_pm_resume(struct device *dev) {
    Ftr();
    if (hw_resume()<0 ) {
		return 0;
	}
//  if_create();
    return 0;
}


MODULE_AUTHOR("Mike_Jang@comaplcomm.com");
MODULE_LICENSE("GPL");
