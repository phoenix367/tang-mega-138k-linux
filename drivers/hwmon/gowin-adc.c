// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 (Tang Mega 138K Pro) on-die temperature / supply-voltage
 * monitor -- hwmon driver for the sensor core in the gateware (sensor/rtl,
 * registers at 0xE8004000).
 *
 * The core scans the FPGA's two hard ADC blocks continuously (ADCLRC in
 * temperature mode, ADCULC in voltage mode over its 8 input buses) and keeps
 * the latest result of each in a register, so a read here is just a register
 * read: no conversion wait, no interrupt.  Temperature is signed quarter
 * degrees; a voltage is the ADC input in 1/2048 V, times the bus's resistive
 * divider for the rail voltage.
 *
 * sysfs: temp1_input (m degC), in0..in7_input (mV) with _label, and _fault =
 * the last conversion of that slot timed out (value is the previous one).
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/hwmon.h>
#include <linux/bitops.h>
#include <linux/bitfield.h>

#define REG_CTRL	0x00
#define  CTRL_RUN	BIT(0)
#define  CTRL_ADCEN	BIT(1)
#define REG_STATUS	0x04
#define REG_SCANS	0x08
#define REG_TIMEOUTS	0x0C
#define REG_TEMP	0x10
#define REG_TCAL	0x14
#define REG_VCAL	0x18
#define REG_ID		0x1C
#define  ID_SENS	0x53454E53
#define REG_VOLT(n)	(0x20 + 4 * (n))
#define  RES_VALUE	GENMASK(13, 0)
#define  RES_VALID	BIT(30)
#define  RES_TIMEOUT	BIT(31)

struct gowin_adc {
	void __iomem *base;
};

/* ADCULC voltage buses (UG299 table 4-2): label and divider of the rail that
 * sensor_adc.v enables on each bus (checked against the board's nominal
 * rails: 0.9 V core, 1.8 V VCCX / MIPI, 1.5 V DDR3 bank).  Bus 3 and 6 have
 * nothing connected and read 0. */
static const struct {
	const char *label;
	unsigned int div;
} gowin_adc_in[8] = {
	{ "VCC core (vtest)",                 2 },
	{ "VCCM (vdd09_0)",                   2 },
	{ "VDDA_Q0 SerDes (vdd09_1)",         2 },
	{ "vdd09_2 (not connected)",          1 },
	{ "VDDX_MIPI (vdd18_0)",              4 },
	{ "VCCX (vdd18_1)",                   4 },
	{ "reserved",                         1 },
	{ "VCCIO7 DDR3 bank (vdd33)",         8 },
};

static int gowin_adc_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct gowin_adc *p = dev_get_drvdata(dev);
	u32 r;

	switch (type) {
	case hwmon_temp:
		r = readl(p->base + REG_TEMP);
		if (!(r & RES_VALID))
			return -ENODATA;
		switch (attr) {
		case hwmon_temp_input:
			*val = sign_extend32(r & RES_VALUE, 13) * 250;	/* 1/4 degC */
			return 0;
		case hwmon_temp_fault:
			*val = !!(r & RES_TIMEOUT);
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_in:
		if (channel >= 8)
			return -EINVAL;
		r = readl(p->base + REG_VOLT(channel));
		if (!(r & RES_VALID))
			return -ENODATA;
		switch (attr) {
		case hwmon_in_input:
			*val = ((r & RES_VALUE) * gowin_adc_in[channel].div * 1000) / 2048;
			return 0;
		case hwmon_in_fault:
			*val = !!(r & RES_TIMEOUT);
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static int gowin_adc_read_string(struct device *dev, enum hwmon_sensor_types type,
				 u32 attr, int channel, const char **str)
{
	if (type == hwmon_temp && attr == hwmon_temp_label) {
		*str = "FPGA die (ADCLRC)";
		return 0;
	}
	if (type == hwmon_in && attr == hwmon_in_label && channel < 8) {
		*str = gowin_adc_in[channel].label;
		return 0;
	}
	return -EOPNOTSUPP;
}

static umode_t gowin_adc_is_visible(const void *data, enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	return 0444;
}

static const struct hwmon_channel_info * const gowin_adc_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_FAULT),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_FAULT),
	NULL
};

static const struct hwmon_ops gowin_adc_ops = {
	.is_visible = gowin_adc_is_visible,
	.read = gowin_adc_read,
	.read_string = gowin_adc_read_string,
};

static const struct hwmon_chip_info gowin_adc_chip = {
	.ops = &gowin_adc_ops,
	.info = gowin_adc_info,
};

static int gowin_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gowin_adc *p;
	struct device *hwmon;
	u32 id;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	id = readl(p->base + REG_ID);
	if (id != ID_SENS) {
		dev_err(dev, "no sensor core (ID %08x)\n", id);
		return -ENODEV;
	}
	writel(CTRL_RUN | CTRL_ADCEN, p->base + REG_CTRL);

	hwmon = devm_hwmon_device_register_with_info(dev, "gowin_adc", p,
						     &gowin_adc_chip, NULL);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);

	dev_info(dev, "on-die temperature + 8 voltage buses (scans %u, timeouts %u)\n",
		 readl(p->base + REG_SCANS), readl(p->base + REG_TIMEOUTS));
	return 0;
}

static const struct of_device_id gowin_adc_of_match[] = {
	{ .compatible = "gowin,ae350-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_adc_of_match);

static struct platform_driver gowin_adc_driver = {
	.probe = gowin_adc_probe,
	.driver = {
		.name = "gowin-adc",
		.of_match_table = gowin_adc_of_match,
	},
};
module_platform_driver(gowin_adc_driver);

MODULE_DESCRIPTION("Gowin AE350 FPGA on-die temperature/voltage monitor");
MODULE_LICENSE("GPL");
