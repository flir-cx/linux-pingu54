/*
 * linux/arch/arm/common/nic_301.c
 *
 * NIC-301 AXI switch settings
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

static int nic_301_probe(struct platform_device *dev)
{
	struct resource *mem;
	void __iomem *base;
	int i;
	int prio = 0;
	char name[20];
	int err;

	mem = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	base = devm_ioremap_resource(&dev->dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for (i=0; i<7; i++) {
		sprintf(name, "m%d_prio", i);

		err = of_property_read_u32(dev->dev.of_node, name, &prio);

		if (!err) {
			writel(prio, (void*)((uint32_t)base + 0x42000 + 0x1000 * i + 0x100));   // Read prio
			writel(prio, (void*)((uint32_t)base + 0x42000 + 0x1000 * i + 0x104));   // Write prio
			dev_info(&dev->dev, "m%d has prio %d\n", i, prio);
		}
	}
	devm_iounmap(&dev->dev, base);

	return 0;
}

static int nic_301_remove(struct platform_device *dev)
{
	return 0;
}

static const struct of_device_id nic_301_of_match[] = {
	{ .compatible = "arm,nic-301", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nic_301_of_match);

static struct platform_driver nic_301_device_driver = {
	.probe		= nic_301_probe,
	.remove		= nic_301_remove,
	.driver		= {
		.name	= "nic_301",
		.of_match_table = of_match_ptr(nic_301_of_match),
	},
};

module_platform_driver(nic_301_device_driver);

MODULE_DESCRIPTION("NIC-301 priority configuration");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Fitger <peter.fitger@flir.se>");
