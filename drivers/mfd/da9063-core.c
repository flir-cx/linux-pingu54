// SPDX-License-Identifier: GPL-2.0+
/*
 * Device access for Dialog DA9063 modules
 *
 * Copyright 2012 Dialog Semiconductors Ltd.
 * Copyright 2013 Philipp Zabel, Pengutronix
 *
 * Author: Krystian Garbaciak, Dialog Semiconductor
 * Author: Michal Hajduk, Dialog Semiconductor
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>


static struct resource da9063_regulators_resources[] = {
	{
		.name	= "LDO_LIM",
		.start	= DA9063_IRQ_LDO_LIM,
		.end	= DA9063_IRQ_LDO_LIM,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_rtc_resources[] = {
	{
		.name	= "ALARM",
		.start	= DA9063_IRQ_ALARM,
		.end	= DA9063_IRQ_ALARM,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "TICK",
		.start	= DA9063_IRQ_TICK,
		.end	= DA9063_IRQ_TICK,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource da9063_onkey_resources[] = {
	{
		.name	= "ONKEY",
		.start	= DA9063_IRQ_ONKEY,
		.end	= DA9063_IRQ_ONKEY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_hwmon_resources[] = {
	{
		.start	= DA9063_IRQ_ADC_RDY,
		.end	= DA9063_IRQ_ADC_RDY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_wake_resources[] = {
	{
		 .name	= "WAKE",
		.start	= DA9063_IRQ_WAKE,
		.end	= DA9063_IRQ_WAKE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_comp1v2_resources[] = {
	{
		 .name	= "COMP_1V2",
		.start	= DA9063_IRQ_COMP_1V2,
		.end	= DA9063_IRQ_COMP_1V2,
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell da9063_common_devs[] = {
	{
		.name		= DA9063_DRVNAME_REGULATORS,
		.num_resources	= ARRAY_SIZE(da9063_regulators_resources),
		.resources	= da9063_regulators_resources,
	},
	{
		.name		= DA9063_DRVNAME_LEDS,
	},
	{
		.name		= DA9063_DRVNAME_WATCHDOG,
		.of_compatible	= "dlg,da9063-watchdog",
	},
	{
		.name		= DA9063_DRVNAME_HWMON,
		.num_resources	= ARRAY_SIZE(da9063_hwmon_resources),
		.resources	= da9063_hwmon_resources,
	},
	{
		.name		= DA9063_DRVNAME_ONKEY,
		.num_resources	= ARRAY_SIZE(da9063_onkey_resources),
		.resources	= da9063_onkey_resources,
		.of_compatible = "dlg,da9063-onkey",
	},
	{
		.name		= DA9063_DRVNAME_VIBRATION,
	},
};

/* Only present on DA9063 , not on DA9063L */
static const struct mfd_cell da9063_devs[] = {
	{
		.name		= DA9063_DRVNAME_RTC,
		.num_resources	= ARRAY_SIZE(da9063_rtc_resources),
		.resources	= da9063_rtc_resources,
		.of_compatible	= "dlg,da9063-rtc",
	},
	{
		.name		= DA9063_DRVNAME_POWEROFF,
		.of_compatible	= "dlg,da9063-poweroff",
	},
	{
		.name		= DA9063_DRVNAME_RESET,
		.of_compatible	= "dlg,da9063-reset",
	},
	{
		.name		= DA9063_DRVNAME_WAKE,
		.num_resources	= ARRAY_SIZE(da9063_wake_resources),
		.resources	= da9063_wake_resources,
		.of_compatible	= "dlg,da9063-wake",
	},
	{
		.name		= DA9063_DRVNAME_COMP1V2,
		.num_resources	= ARRAY_SIZE(da9063_comp1v2_resources),
		.resources	= da9063_comp1v2_resources,
		.of_compatible	= "dlg,da9063-comp1v2",
	},
};

/*
 * Variant specific regmap configs
 */

static const struct regmap_range da9063_ad_readable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_AD_REG_SECOND_D),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_T_OFFSET, DA9063_AD_REG_GP_ID_19),
	regmap_reg_range(DA9063_REG_DEVICE_ID, DA9063_REG_VARIANT_ID),
};

static const struct regmap_range da9063_ad_writeable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_PAGE_CON),
	regmap_reg_range(DA9063_REG_FAULT_LOG, DA9063_REG_VSYS_MON),
	regmap_reg_range(DA9063_REG_COUNT_S, DA9063_AD_REG_ALARM_Y),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_CONFIG_I, DA9063_AD_REG_MON_REG_4),
	regmap_reg_range(DA9063_AD_REG_GP_ID_0, DA9063_AD_REG_GP_ID_19),
};

static const struct regmap_range da9063_ad_volatile_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_EVENT_D),
	regmap_reg_range(DA9063_REG_CONTROL_A, DA9063_REG_CONTROL_B),
	regmap_reg_range(DA9063_REG_CONTROL_E, DA9063_REG_CONTROL_F),
	regmap_reg_range(DA9063_REG_BCORE2_CONT, DA9063_REG_LDO11_CONT),
	regmap_reg_range(DA9063_REG_DVC_1, DA9063_REG_ADC_MAN),
	regmap_reg_range(DA9063_REG_ADC_RES_L, DA9063_AD_REG_SECOND_D),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_SEQ),
	regmap_reg_range(DA9063_REG_EN_32K, DA9063_REG_EN_32K),
	regmap_reg_range(DA9063_AD_REG_MON_REG_5, DA9063_AD_REG_MON_REG_6),
};

static const struct regmap_access_table da9063_ad_readable_table = {
	.yes_ranges = da9063_ad_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_ad_readable_ranges),
};

static const struct regmap_access_table da9063_ad_writeable_table = {
	.yes_ranges = da9063_ad_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_ad_writeable_ranges),
};

static const struct regmap_access_table da9063_ad_volatile_table = {
	.yes_ranges = da9063_ad_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_ad_volatile_ranges),
};

static const struct regmap_range da9063_bb_readable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_BB_REG_SECOND_D),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_T_OFFSET, DA9063_BB_REG_GP_ID_19),
	regmap_reg_range(DA9063_REG_DEVICE_ID, DA9063_REG_VARIANT_ID),
};

static const struct regmap_range da9063_bb_writeable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_PAGE_CON),
	regmap_reg_range(DA9063_REG_FAULT_LOG, DA9063_REG_VSYS_MON),
	regmap_reg_range(DA9063_REG_COUNT_S, DA9063_BB_REG_ALARM_Y),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_CONFIG_I, DA9063_BB_REG_MON_REG_4),
	regmap_reg_range(DA9063_BB_REG_GP_ID_0, DA9063_BB_REG_GP_ID_19),
};

static const struct regmap_range da9063_bb_da_volatile_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_EVENT_D),
	regmap_reg_range(DA9063_REG_CONTROL_A, DA9063_REG_CONTROL_B),
	regmap_reg_range(DA9063_REG_CONTROL_E, DA9063_REG_CONTROL_F),
	regmap_reg_range(DA9063_REG_BCORE2_CONT, DA9063_REG_LDO11_CONT),
	regmap_reg_range(DA9063_REG_DVC_1, DA9063_REG_ADC_MAN),
	regmap_reg_range(DA9063_REG_ADC_RES_L, DA9063_BB_REG_SECOND_D),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_SEQ),
	regmap_reg_range(DA9063_REG_EN_32K, DA9063_REG_EN_32K),
	regmap_reg_range(DA9063_BB_REG_MON_REG_5, DA9063_BB_REG_MON_REG_6),
};

static const struct regmap_access_table da9063_bb_readable_table = {
	.yes_ranges = da9063_bb_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_bb_readable_ranges),
};

static const struct regmap_access_table da9063_bb_writeable_table = {
	.yes_ranges = da9063_bb_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_bb_writeable_ranges),
};

static const struct regmap_access_table da9063_bb_da_volatile_table = {
	.yes_ranges = da9063_bb_da_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_bb_da_volatile_ranges),
};

static const struct regmap_range da9063l_bb_readable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_MON_A10_RES),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_T_OFFSET, DA9063_BB_REG_GP_ID_19),
	regmap_reg_range(DA9063_REG_DEVICE_ID, DA9063_REG_VARIANT_ID),
};

static const struct regmap_range da9063l_bb_writeable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_PAGE_CON),
	regmap_reg_range(DA9063_REG_FAULT_LOG, DA9063_REG_VSYS_MON),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_CONFIG_I, DA9063_BB_REG_MON_REG_4),
	regmap_reg_range(DA9063_BB_REG_GP_ID_0, DA9063_BB_REG_GP_ID_19),
};

static const struct regmap_range da9063l_bb_da_volatile_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_EVENT_D),
	regmap_reg_range(DA9063_REG_CONTROL_A, DA9063_REG_CONTROL_B),
	regmap_reg_range(DA9063_REG_CONTROL_E, DA9063_REG_CONTROL_F),
	regmap_reg_range(DA9063_REG_BCORE2_CONT, DA9063_REG_LDO11_CONT),
	regmap_reg_range(DA9063_REG_DVC_1, DA9063_REG_ADC_MAN),
	regmap_reg_range(DA9063_REG_ADC_RES_L, DA9063_REG_MON_A10_RES),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_SEQ),
	regmap_reg_range(DA9063_REG_EN_32K, DA9063_REG_EN_32K),
	regmap_reg_range(DA9063_BB_REG_MON_REG_5, DA9063_BB_REG_MON_REG_6),
};

static const struct regmap_access_table da9063l_bb_readable_table = {
	.yes_ranges = da9063l_bb_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063l_bb_readable_ranges),
};

static const struct regmap_access_table da9063l_bb_writeable_table = {
	.yes_ranges = da9063l_bb_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063l_bb_writeable_ranges),
};

static const struct regmap_access_table da9063l_bb_da_volatile_table = {
	.yes_ranges = da9063l_bb_da_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063l_bb_da_volatile_ranges),
};

static const struct regmap_range da9063_da_readable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_BB_REG_SECOND_D),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_T_OFFSET, DA9063_BB_REG_GP_ID_11),
	regmap_reg_range(DA9063_REG_DEVICE_ID, DA9063_REG_VARIANT_ID),
};

static const struct regmap_range da9063_da_writeable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_PAGE_CON),
	regmap_reg_range(DA9063_REG_FAULT_LOG, DA9063_REG_VSYS_MON),
	regmap_reg_range(DA9063_REG_COUNT_S, DA9063_BB_REG_ALARM_Y),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_CONFIG_I, DA9063_BB_REG_MON_REG_4),
	regmap_reg_range(DA9063_BB_REG_GP_ID_0, DA9063_BB_REG_GP_ID_11),
};

static const struct regmap_access_table da9063_da_readable_table = {
	.yes_ranges = da9063_da_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_da_readable_ranges),
};

static const struct regmap_access_table da9063_da_writeable_table = {
	.yes_ranges = da9063_da_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063_da_writeable_ranges),
};

static const struct regmap_range da9063l_da_readable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_MON_A10_RES),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_T_OFFSET, DA9063_BB_REG_GP_ID_11),
	regmap_reg_range(DA9063_REG_DEVICE_ID, DA9063_REG_VARIANT_ID),
};

static const struct regmap_range da9063l_da_writeable_ranges[] = {
	regmap_reg_range(DA9063_REG_PAGE_CON, DA9063_REG_PAGE_CON),
	regmap_reg_range(DA9063_REG_FAULT_LOG, DA9063_REG_VSYS_MON),
	regmap_reg_range(DA9063_REG_SEQ, DA9063_REG_ID_32_31),
	regmap_reg_range(DA9063_REG_SEQ_A, DA9063_REG_AUTO3_LOW),
	regmap_reg_range(DA9063_REG_CONFIG_I, DA9063_BB_REG_MON_REG_4),
	regmap_reg_range(DA9063_BB_REG_GP_ID_0, DA9063_BB_REG_GP_ID_11),
};

static const struct regmap_access_table da9063l_da_readable_table = {
	.yes_ranges = da9063l_da_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063l_da_readable_ranges),
};

static const struct regmap_access_table da9063l_da_writeable_table = {
	.yes_ranges = da9063l_da_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(da9063l_da_writeable_ranges),
};

struct regmap_range_cfg da9063_range_cfg[] = {
	{
		.range_min = DA9063_REG_PAGE_CON,
		.range_max = DA9063_REG_CONFIG_ID,
		.selector_reg = DA9063_REG_PAGE_CON,
		.window_start = 0,
	}
};

struct regmap_config da9063_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.ranges = da9063_range_cfg,
	.num_ranges = ARRAY_SIZE(da9063_range_cfg),
	.max_register = DA9063_REG_CONFIG_ID,
	.cache_type = REGCACHE_RBTREE,
};


int da9063_register_regmap(struct da9063 *da9063)
{
	switch (da9063->type) {
	case PMIC_TYPE_DA9063:
		switch (da9063->variant_code) {
		case PMIC_DA9063_AD:
			da9063_regmap_config.rd_table =
				&da9063_ad_readable_table;
			da9063_regmap_config.wr_table =
				&da9063_ad_writeable_table;
			da9063_regmap_config.volatile_table =
				&da9063_ad_volatile_table;
			break;
		case PMIC_DA9063_BB:
		case PMIC_DA9063_CA:
			da9063_regmap_config.rd_table =
				&da9063_bb_readable_table;
			da9063_regmap_config.wr_table =
				&da9063_bb_writeable_table;
			da9063_regmap_config.volatile_table =
				&da9063_bb_da_volatile_table;
			break;
		case PMIC_DA9063_DA:
			da9063_regmap_config.rd_table =
				&da9063_da_readable_table;
			da9063_regmap_config.wr_table =
				&da9063_da_writeable_table;
			da9063_regmap_config.volatile_table =
				&da9063_bb_da_volatile_table;
			break;
		default:
			dev_err(da9063->dev,
				"Chip variant not supported for DA9063\n");
			return -ENODEV;
		}
		break;
	case PMIC_TYPE_DA9063L:
		switch (da9063->variant_code) {
		case PMIC_DA9063_BB:
		case PMIC_DA9063_CA:
			da9063_regmap_config.rd_table =
				&da9063l_bb_readable_table;
			da9063_regmap_config.wr_table =
				&da9063l_bb_writeable_table;
			da9063_regmap_config.volatile_table =
				&da9063l_bb_da_volatile_table;
			break;
		case PMIC_DA9063_DA:
			da9063_regmap_config.rd_table =
				&da9063l_da_readable_table;
			da9063_regmap_config.wr_table =
				&da9063l_da_writeable_table;
			da9063_regmap_config.volatile_table =
				&da9063l_bb_da_volatile_table;
			break;
		default:
			dev_err(da9063->dev,
				"Chip variant not supported for DA9063L\n");
			return -ENODEV;
		}
		break;
	default:
		dev_err(da9063->dev, "Chip type not supported\n");
		return -ENODEV;
	}

	return 0;
}



static int da9063_clear_fault_log(struct da9063 *da9063)
{
	int ret = 0;
	int fault_log = 0;

	ret = regmap_read(da9063->regmap, DA9063_REG_FAULT_LOG, &fault_log);
	if (ret < 0) {
		dev_err(da9063->dev, "Cannot read FAULT_LOG.\n");
		return -EIO;
	}

	if (fault_log) {
		if (fault_log & DA9063_TWD_ERROR)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_TWD_ERROR\n");
		if (fault_log & DA9063_POR)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_POR\n");
		if (fault_log & DA9063_VDD_FAULT)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_VDD_FAULT\n");
		if (fault_log & DA9063_VDD_START)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_VDD_START\n");
		if (fault_log & DA9063_TEMP_CRIT)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_TEMP_CRIT\n");
		if (fault_log & DA9063_KEY_RESET)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_KEY_RESET\n");
		if (fault_log & DA9063_NSHUTDOWN)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_NSHUTDOWN\n");
		if (fault_log & DA9063_WAIT_SHUT)
			dev_dbg(da9063->dev,
				"Fault log entry detected: DA9063_WAIT_SHUT\n");
	}

	ret = regmap_write(da9063->regmap,
			   DA9063_REG_FAULT_LOG,
			   fault_log);
	if (ret < 0)
		dev_err(da9063->dev,
			"Cannot reset FAULT_LOG values %d\n", ret);

	return ret;
}

int da9063_device_init(struct da9063 *da9063, unsigned int irq)
{
	int t_offset = 0;
	int ret;

	ret = da9063_clear_fault_log(da9063);
	if (ret < 0)
		dev_err(da9063->dev, "Cannot clear fault log\n");

        dev_info(da9063->dev,
                 "Device detected (type: 0x%02X, variant: 0x%02X)\n",
                 da9063->type, da9063->variant_code);

	da9063->flags = 0;
	da9063->irq_base = -1;
	da9063->chip_irq = irq;

	ret = da9063_irq_init(da9063);
	if (ret) {
		dev_err(da9063->dev, "Cannot initialize interrupts.\n");
		return ret;
	}

	da9063->irq_base = regmap_irq_chip_get_base(da9063->regmap_irq);

	ret = devm_mfd_add_devices(da9063->dev, PLATFORM_DEVID_NONE,
				   da9063_common_devs,
				   ARRAY_SIZE(da9063_common_devs),
				   NULL, da9063->irq_base, NULL);
	if (ret) {
		dev_err(da9063->dev, "Failed to add child devices\n");
		return ret;
	}

	if (da9063->type == PMIC_TYPE_DA9063) {
		ret = devm_mfd_add_devices(da9063->dev, PLATFORM_DEVID_NONE,
					   da9063_devs, ARRAY_SIZE(da9063_devs),
					   NULL, da9063->irq_base, NULL);
		if (ret) {
			dev_err(da9063->dev, "Failed to add child devices\n");
			return ret;
		}
	}

	ret = regmap_read(da9063->regmap, DA9063_REG_T_OFFSET, &t_offset);
	if (ret < 0)
		dev_warn(da9063->dev,
			 "Temperature trimming value cannot be read (defaulting to 0)\n");

	/* pass this on to the hwmon driver */
	da9063->t_offset = t_offset;

	return ret;
}

MODULE_DESCRIPTION("PMIC driver for Dialog DA9063");
MODULE_AUTHOR("Krystian Garbaciak");
MODULE_AUTHOR("Michal Hajduk");
MODULE_LICENSE("GPL");
