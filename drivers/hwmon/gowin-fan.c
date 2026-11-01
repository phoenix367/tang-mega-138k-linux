// SPDX-License-Identifier: GPL-2.0
/*
 * Gowin AE350 (Tang Mega 138K Pro Dock) fan PWM + tachometer -- hwmon
 * driver for the fan core in the gateware (fan/rtl, registers at
 * 0xE8008000, Dock header M1 "Fan_CPU_4pin": PWM on T17, tacho on T18).
 *
 * The core runs a PWM of PERIOD bus clocks (reset 25 kHz) with an on-time
 * of DUTY clocks, and counts tacho edges in one-second windows.  Dock rev
 * 4071d feeds the pin to the fan's PWM input; rev 4071f switches the fan's
 * 5 V through a P-MOSFET from the same pin (active low): set the DT
 * property "gowin,pwm-inverted" for that revision.
 *
 * sysfs (hwmon): pwm1 (0..255), pwm1_enable (0 = full speed, 1 = manual),
 * fan1_input (RPM, 2 tacho pulses per revolution unless
 * "gowin,pulses-per-rev" says otherwise), fan1_fault (fan stopped while
 * PWM > 0).
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/hwmon.h>
#include <linux/bitops.h>

#define REG_CTRL	0x00
#define  CTRL_EN	BIT(0)
#define  CTRL_INV	BIT(1)
#define REG_PERIOD	0x04
#define REG_DUTY	0x08
#define REG_RATE	0x0C
#define  RATE_MASK	GENMASK(15, 0)
#define REG_CNT		0x10
#define REG_ID		0x14
#define  ID_FAN		0x46414E20
#define REG_STATUS	0x18

struct gowin_fan {
	void __iomem *base;
	u32 bus_hz;
	u32 pulses_per_rev;
	u8 pwm;		/* last value written through pwm1 */
	u8 enable;	/* pwm1_enable */
};

static u32 fan_rd(struct gowin_fan *f, u32 reg) { return readl(f->base + reg); }
static void fan_wr(struct gowin_fan *f, u32 reg, u32 v) { writel(v, f->base + reg); }

static void gowin_fan_apply(struct gowin_fan *f)
{
	u32 period = fan_rd(f, REG_PERIOD);
	u32 duty = f->enable ? DIV_ROUND_CLOSEST(period * f->pwm, 255) : period;

	fan_wr(f, REG_DUTY, duty);
}

static int gowin_fan_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct gowin_fan *f = dev_get_drvdata(dev);
	u32 rate;

	switch (type) {
	case hwmon_fan:
		rate = fan_rd(f, REG_RATE) & RATE_MASK;
		if (attr == hwmon_fan_input) {
			*val = rate * 60 / f->pulses_per_rev;
			return 0;
		}
		if (attr == hwmon_fan_fault) {
			*val = rate == 0 && fan_rd(f, REG_DUTY) != 0;
			return 0;
		}
		return -EOPNOTSUPP;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input) {
			*val = f->pwm;
			return 0;
		}
		if (attr == hwmon_pwm_enable) {
			*val = f->enable;
			return 0;
		}
		return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static int gowin_fan_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long val)
{
	struct gowin_fan *f = dev_get_drvdata(dev);

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;
	switch (attr) {
	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;
		f->pwm = val;
		break;
	case hwmon_pwm_enable:
		if (val < 0 || val > 1)
			return -EINVAL;
		f->enable = val;
		break;
	default:
		return -EOPNOTSUPP;
	}
	gowin_fan_apply(f);
	return 0;
}

static umode_t gowin_fan_is_visible(const void *data, enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	return type == hwmon_pwm ? 0644 : 0444;
}

static const struct hwmon_channel_info * const gowin_fan_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT | HWMON_F_FAULT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL
};

static const struct hwmon_ops gowin_fan_ops = {
	.is_visible = gowin_fan_is_visible,
	.read = gowin_fan_read,
	.write = gowin_fan_write,
};

static const struct hwmon_chip_info gowin_fan_chip = {
	.ops = &gowin_fan_ops,
	.info = gowin_fan_info,
};

static int gowin_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gowin_fan *f;
	struct device *hwmon;
	u32 id, ctrl, freq;

	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(f->base))
		return PTR_ERR(f->base);
	id = fan_rd(f, REG_ID);
	if (id != ID_FAN) {
		dev_err(dev, "no fan core (ID %08x)\n", id);
		return -ENODEV;
	}

	f->bus_hz = 50000000;
	of_property_read_u32(dev->of_node, "clock-frequency", &f->bus_hz);
	f->pulses_per_rev = 2;
	of_property_read_u32(dev->of_node, "gowin,pulses-per-rev", &f->pulses_per_rev);
	if (!f->pulses_per_rev)
		f->pulses_per_rev = 2;
	if (!of_property_read_u32(dev->of_node, "gowin,pwm-frequency", &freq) && freq)
		fan_wr(f, REG_PERIOD, max_t(u32, 1, f->bus_hz / freq));
	ctrl = CTRL_EN;
	if (of_property_read_bool(dev->of_node, "gowin,pwm-inverted"))
		ctrl |= CTRL_INV;
	fan_wr(f, REG_CTRL, ctrl);

	/* start at full speed, manual mode: a fan curve (userspace) lowers it */
	f->pwm = 255;
	f->enable = 1;
	gowin_fan_apply(f);

	hwmon = devm_hwmon_device_register_with_info(dev, "gowin_fan", f,
						     &gowin_fan_chip, NULL);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);
	dev_info(dev, "fan PWM %u Hz%s, tacho %u pulses/rev\n",
		 f->bus_hz / fan_rd(f, REG_PERIOD), ctrl & CTRL_INV ? " (inverted)" : "",
		 f->pulses_per_rev);
	return 0;
}

static const struct of_device_id gowin_fan_of_match[] = {
	{ .compatible = "gowin,ae350-fan" },
	{ }
};
MODULE_DEVICE_TABLE(of, gowin_fan_of_match);

static struct platform_driver gowin_fan_driver = {
	.probe = gowin_fan_probe,
	.driver = {
		.name = "gowin-fan",
		.of_match_table = gowin_fan_of_match,
	},
};
module_platform_driver(gowin_fan_driver);

MODULE_DESCRIPTION("Gowin AE350 gateware fan PWM/tacho hwmon driver");
MODULE_LICENSE("GPL");
