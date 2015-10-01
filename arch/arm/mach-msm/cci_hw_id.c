#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

#include <mach/msm_iomap.h>
#include <mach/cci_hw_id.h>

#ifdef HWID_USE_SHARED_MEMORY
#include <mach/msm_smsm.h>
#endif

static char hwid_devtree_property_table[][HWID_DEVTREE_RPOPERTY_NAME_SIZE] = {
	HWID_DEVTREE_PROPERTY_PROJECT_ID,
	HWID_DEVTREE_PROPERTY_HW_ID,
	HWID_DEVTREE_PROPERTY_SIM_ID,
	HWID_DEVTREE_PROPERTY_BAND_ID,
	HWID_DEVTREE_PROPERTY_CUSTOMER_ID
};

static char hwid_devtree_gpio_label_table[][HWID_DEVTREE_GPIO_LABEL_NAME_SIZE] = {
	HWID_DEVTREE_GPIO_LABEL_PROJECT_ID,
	HWID_DEVTREE_GPIO_LABEL_HW_ID,
	HWID_DEVTREE_GPIO_LABEL_SIM_ID,
	HWID_DEVTREE_GPIO_LABEL_BAND_ID,
	HWID_DEVTREE_GPIO_LABEL_CUSTOMER_ID
};

struct hwid_info {
	unsigned int project_id;
	unsigned int hw_id;
	unsigned int sim_id;
	unsigned int band_id;
	unsigned int customer_id;
};
static struct hwid_info cci_hwid_info;

//#define CONFIG_CCI_HWID_READ_GPIO 1

static const char cci_sim_type_str[][CCI_HWID_TYPE_STRLEN] = 
{
	"Single sim",
	"Dual sim",
	"Invalid"
};

static const char cci_board_type_str[][CCI_HWID_TYPE_STRLEN] = 
{
	"EVT board",
	"DVT1 board",
	"DVT1-1 board",
	"DVT2 board",
	"DVT3 board",
	"PVT board",
	"MP board",
	"Invalid"
};

static const char cci_band_type_str[][CCI_HWID_TYPE_STRLEN] = 
{
	"EU band",
	"US band",
	"Invalid"
};

static const char cci_proj_name_str[][CCI_HWID_TYPE_STRLEN] = 
{
	"VY52_53",
	"VY55_56",
	"VY58_59",
	"Undefined"
};

static const char cci_customer_type_str[][CCI_HWID_TYPE_STRLEN] = 
{
	"Customer",
	"CCI",
	"Invalid"
};

int get_cci_project_id(void)
{
	return cci_hwid_info.project_id;
}
EXPORT_SYMBOL(get_cci_project_id);

int get_cci_hw_id(void)
{
	return cci_hwid_info.hw_id;
}
EXPORT_SYMBOL(get_cci_hw_id);

int get_cci_sim_id(void)
{
	return cci_hwid_info.sim_id;
}
EXPORT_SYMBOL(get_cci_sim_id);

int get_cci_band_id(void)
{
	return cci_hwid_info.band_id;
}
EXPORT_SYMBOL(get_cci_band_id);

int get_cci_customer_id(void)
{
	return cci_hwid_info.customer_id;
}
EXPORT_SYMBOL(get_cci_customer_id);

int cci_hw_proj_name_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int len;
	len = sprintf(page, "%s\n", cci_proj_name_str[ cci_hwid_info.project_id ] );
	return len;
}

int cci_hw_board_type_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int  len;
	len = sprintf(page, "%s\n", cci_board_type_str[ cci_hwid_info.hw_id ] );
	return len;
}

int cci_hw_sim_type_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int len;
	len = sprintf(page, "%s\n", cci_sim_type_str[ cci_hwid_info.sim_id] );
	return len;
}

int cci_hw_band_type_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int len;
	len = sprintf(page, "%s\n", cci_band_type_str[ cci_hwid_info.band_id ] );
	return len;
}

int cci_hw_customer_type_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int len;
	len = sprintf(page, "%s\n", cci_customer_type_str[ cci_hwid_info.customer_id ] );
	return len;
}

int cci_hwid_info_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
	int len;
	len = sprintf(page, "%s=%d %s=%d %s=%d %s=%d %s=%d\n", 
					"projectid", cci_hwid_info.project_id, 
					"hwid", cci_hwid_info.hw_id, 
					"multisim", cci_hwid_info.sim_id, 
					"bandid", cci_hwid_info.band_id, 
					"customer", cci_hwid_info.customer_id);
	return len;
}

static const struct of_device_id cci_hw_id_of_match[] = {
	{ .compatible = "gpio-hwid", },
	{ },
};
MODULE_DEVICE_TABLE(of, cci_hw_id_of_match);

/**
 **	HWID device tree node is defined in msm8226-720p-mtp.dtsi
 **/

static int __devinit cci_hw_id_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	int i, j, gpio_cnt=0, gpio_weight, gpio, ret, is_waterproof = 0;
	const char *name = NULL;
	unsigned int hwid[5] = {0};

	node = dev->of_node;
	
	/* Initialize cci_hwid_info */
	cci_hwid_info.project_id = 0;
	cci_hwid_info.hw_id = 0;
	cci_hwid_info.sim_id = 0;
	cci_hwid_info.band_id = 0;
	cci_hwid_info.customer_id = 0;

	printk(KERN_INFO "%s: Read HWID from DT\n", "cci_hw_id_probe");
	for ( i=0; i < HWID_DEVTREE_RPOPERTY_NAME_NUM; i++) {
		gpio_cnt = of_gpio_named_count(node, hwid_devtree_property_table[i]);
		if ( gpio_cnt ) {
			for (j = 0, gpio_weight=1; j < gpio_cnt; j++, gpio_weight*=2) {
				gpio = of_get_named_gpio(node, hwid_devtree_property_table[i], j);
				if (!gpio_is_valid(gpio))
					goto gpio_is_invalid;				
				of_property_read_string_index(pdev->dev.of_node, hwid_devtree_gpio_label_table[i], j, &name);
				if (!name)
					continue;
				//printk(KERN_INFO "In %s, gpio name = %s\n", __FUNCTION__, name);
				ret = gpio_request(gpio, name);
				if (ret)
					goto gpio_request_fail;
				if (gpio_get_value(gpio) == 1)
					hwid[i] += gpio_weight;	
				gpio_free(gpio);
			}
		}
	}

	cci_hwid_info.project_id = hwid[0];
	cci_hwid_info.hw_id = hwid[1];
	cci_hwid_info.sim_id = hwid[2];
	cci_hwid_info.band_id = hwid[3];
	cci_hwid_info.customer_id = hwid[4];

	if (cci_hwid_info.project_id == CCI_PROJECTID_VY55_56) {
		printk(KERN_INFO "Check if waterproof\n");
		cci_hwid_info.sim_id = CCI_SIMID_SS;
		ret = gpio_request(GPIO_CCI_WATERPROOF_ID, "gpio_waterproof_id");
		if (ret)
			goto gpio_request_fail;
		is_waterproof = gpio_get_value(GPIO_CCI_WATERPROOF_ID);
		if (is_waterproof)
			cci_hwid_info.project_id = CCI_PROJECTID_VY58_59;
		gpio_free(GPIO_CCI_WATERPROOF_ID);
	}
	
	printk(KERN_INFO "cci_project_id=%d, cci_hw_id=%d, cci_sim_id=%d, cci_band_id=%d, cci_customer_id=%d",
				cci_hwid_info.project_id,
				cci_hwid_info.hw_id,
				cci_hwid_info.sim_id,
				cci_hwid_info.band_id,
				cci_hwid_info.customer_id);

	create_proc_read_entry("cci_hw_sim_type", 0, NULL, cci_hw_sim_type_read, NULL);
	create_proc_read_entry("cci_hw_board_type", 0, NULL, cci_hw_board_type_read, NULL);
	create_proc_read_entry("cci_hw_band_type", 0, NULL, cci_hw_band_type_read, NULL);
	create_proc_read_entry("cci_hw_proj_type", 0, NULL, cci_hw_proj_name_read, NULL);
	create_proc_read_entry("cci_hw_customer_type", 0, NULL, cci_hw_customer_type_read, NULL);
	create_proc_read_entry("cci_hwid_info", 0, NULL, cci_hwid_info_read, NULL);

	return 0;

gpio_is_invalid:
	printk(KERN_INFO "gpio_%d is invalid !!!!\n", gpio); 	
	return gpio;
gpio_request_fail:
	printk(KERN_INFO "gpio_%d request failed!!!!\n", gpio); 	
	return gpio;	
}

static int cci_hw_id_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "CCI HWID remove\n");

	return 0;
}

static struct platform_driver cci_hw_id_driver = {
	.probe		= cci_hw_id_probe,
	.remove		= cci_hw_id_remove,
	.driver = {
		.name = "gpio-hwid",
		.owner = THIS_MODULE,
		.of_match_table = cci_hw_id_of_match,
	},
};

static int __init cci_hw_id_init(void)
{
	printk(KERN_INFO "cci_hw_id_init\n");
	return platform_driver_register(&cci_hw_id_driver);
}
static void __exit cci_hw_id_exit(void)
{
	printk(KERN_INFO "cci_hw_id_exit\n");
	platform_driver_unregister(&cci_hw_id_driver);
}

module_init(cci_hw_id_init);
module_exit(cci_hw_id_exit);

MODULE_DESCRIPTION("cci hardware ID driver");
MODULE_AUTHOR("Chewei Liang <chewei_liang@compal.com>");
MODULE_LICENSE("GPL");
