// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS Vivobook S15 (S5507QA, Snapdragon X) embedded controller driver.
 *
 * The EC is an ITE IT5571 on the SoC I2C bus behind two addresses:
 *
 *   0x76  command interface: SoC temperature feed, fan RPM, fan profile,
 *         suspend notification, keyboard RGB
 *   0x5B  byte-addressed EC RAM window (page:offset). Page 0xC4 holds the
 *         command mailbox used for fan mode and manual duty.
 *
 * The EC expects the host to report the SoC temperature periodically. If
 * the reports stop for a couple of minutes it hard powers the machine off,
 * so this driver must stay bound.
 *
 * The Windows DSDT also describes custom fan curves through a block mailbox
 * on page 0xC9; this EC firmware NAKs that page, so they are not supported.
 *
 * Derived from the EC firmware (03.00.07) and the Windows DSDT.
 *
 * Besides the EC itself the driver applies two machine policies that used
 * to need userspace helpers:
 *
 *  - every fan profile also caps the CPU frequency and decides whether the
 *    GPU runs at its top frequency (quiet profiles stay quiet under load);
 *  - the EC pulses the lid switch about five minutes into s2idle even with
 *    the lid open, so a suspend with the lid open drops the lid wakeup for
 *    that sleep (power key and keyboard still wake the machine).
 */

#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/platform_data/asus-vivobook-s15-ec.h>
#include <linux/pm.h>
#include <linux/pm_qos.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#define DRIVER_NAME		"asus_vivobook_ec"

#define EC_MAILBOX_ADDR		0x5b

/* Commands on the 0x76 interface */
#define EC_CMD_SOC_TEMP		0x20
#define EC_CMD_FAN_RPM		0x22
#define EC_CMD_SUSPEND		0x23
#define EC_CMD_FAN_PROFILE	0x24
#define EC_CMD_KBD_RGB		0x51

/* 0x5B RAM window */
#define EC_RAM_SELECT		0x10
#define EC_RAM_DATA		0x11

/* Page 0xC4: command mailbox (ECCW/ECCR in the DSDT) */
#define EC_PAGE_CMD		0xc4
#define EC_CMD_MB_OP		0x30
#define EC_CMD_MB_REG		0x31
#define EC_CMD_MB_VAL		0x32

#define EC_BANK_FAN		0x01
#define EC_REG_FAN_MODE		0x82
#define EC_REG_FAN_DUTY		0x8a	/* manual mode only */
#define EC_REG_FAN_SELECT	0x8c

#define EC_FAN_MODE_AUTO	0x00
#define EC_FAN_MODE_MANUAL	0x02

#define EC_NUM_FANS		2

#define EC_TEMP_PERIOD_MS	2000

enum ec_profile {
	EC_PROFILE_WHISPER,
	EC_PROFILE_STANDARD,
	EC_PROFILE_PERFORMANCE,
	EC_PROFILE_FULL,
	EC_PROFILE_COUNT,
};

/* Same order as the EC and Windows MyASUS */
static const char * const ec_profile_names[EC_PROFILE_COUNT] = {
	[EC_PROFILE_WHISPER]	 = "whisper",
	[EC_PROFILE_STANDARD]	 = "standard",
	[EC_PROFILE_PERFORMANCE] = "performance",
	[EC_PROFILE_FULL]	 = "full",
};

enum ec_rgb_mode {
	EC_RGB_SOLID = 1,
	EC_RGB_BREATHE,
	EC_RGB_RAINBOW,
	EC_RGB_STROBE,
};

static const char * const ec_rgb_mode_names[] = {
	"solid", "breathe", "rainbow", "strobe",
};

/*
 * The EC gets the maximum of these zones, each plus an offset. Missing
 * zones (e.g. cpu2 on X1P) are skipped. The per-core "btm" sensors run
 * 5-10 °C above the "top" ones, so all of them are watched.
 *
 * The battery is watched too, without an offset, so a hot battery while
 * charging can still start the fans; at idle it is cooler than the SoC
 * and does not keep them spinning.
 */
struct ec_zone {
	const char *name;
	int offset_mc;
};

#define EC_CPU_ZONES(c)							\
	{ "cpu" #c "-0-top-thermal" }, { "cpu" #c "-0-btm-thermal" },	\
	{ "cpu" #c "-1-top-thermal" }, { "cpu" #c "-1-btm-thermal" },	\
	{ "cpu" #c "-2-top-thermal" }, { "cpu" #c "-2-btm-thermal" },	\
	{ "cpu" #c "-3-top-thermal" }, { "cpu" #c "-3-btm-thermal" }

static const struct ec_zone ec_default_zones[] = {
	EC_CPU_ZONES(0), EC_CPU_ZONES(1), EC_CPU_ZONES(2),
	{ "cpuss0-top-thermal" }, { "cpuss0-btm-thermal" },
	{ "cpuss1-top-thermal" }, { "cpuss1-btm-thermal" },
	{ "cpuss2-top-thermal" }, { "cpuss2-btm-thermal" },
	{ "gpuss-0-thermal" }, { "gpuss-1-thermal" }, { "gpuss-2-thermal" },
	{ "gpuss-3-thermal" }, { "gpuss-4-thermal" }, { "gpuss-5-thermal" },
	{ "gpuss-6-thermal" }, { "gpuss-7-thermal" },
	{ "nsp0-thermal" }, { "nsp1-thermal" }, { "nsp2-thermal" }, { "nsp3-thermal" },
	{ "mem-thermal" }, { "video-thermal" },
	{ "qcom-battmgr-bat" },
};

#define EC_MAX_PARAM_ZONES	64

static char *thermal_zones[EC_MAX_PARAM_ZONES];
static int nr_thermal_zones;
module_param_array(thermal_zones, charp, &nr_thermal_zones, 0444);
MODULE_PARM_DESC(thermal_zones,
		 "Zones reported to the EC as name[:offset_°C], replaces the built-in list");

static unsigned int fan_profile_default = EC_PROFILE_STANDARD;
module_param(fan_profile_default, uint, 0444);
MODULE_PARM_DESC(fan_profile_default,
		 "Fan profile at boot: 0=whisper 1=standard 2=performance 3=full");

static int fallback_temp_mc = 70000;
module_param(fallback_temp_mc, int, 0644);
MODULE_PARM_DESC(fallback_temp_mc,
		 "Temperature (m°C) reported when no thermal zone is readable");

/* whisper, standard, performance, full */
static unsigned int profile_cpu_max_khz[EC_PROFILE_COUNT] = { 998400, 1440000, 0, 0 };
module_param_array(profile_cpu_max_khz, uint, NULL, 0644);
MODULE_PARM_DESC(profile_cpu_max_khz,
		 "CPU frequency cap (kHz) per fan profile whisper,standard,performance,full; 0 = none");

static bool profile_gpu_max[EC_PROFILE_COUNT] = { false, false, true, true };
module_param_array(profile_gpu_max, bool, NULL, 0644);
MODULE_PARM_DESC(profile_gpu_max,
		 "Hold the GPU at its top frequency, per fan profile whisper,standard,performance,full");

static bool lid_wake_quirk = true;
module_param(lid_wake_quirk, bool, 0644);
MODULE_PARM_DESC(lid_wake_quirk,
		 "Disable lid wakeup while suspended with the lid open (the EC pulses the lid switch)");

#define EC_MAX_POLICIES		8

struct asus_ec {
	struct device *dev;
	struct i2c_client *client;	/* 0x76 */
	struct i2c_client *mailbox;	/* 0x5B */

	/* Serialises EC transactions and the cached state below */
	struct mutex lock;

	const struct ec_zone *zones;
	int nr_zones;
	struct delayed_work temp_work;
	int last_temp_mc;
	const char *last_source;	/* hottest zone, NULL for the fallback */
	bool no_zone_warned;

	enum ec_profile profile;
	bool manual;
	u8 pwm[EC_NUM_FANS];
	bool pwm_valid[EC_NUM_FANS];

	u8 rgb[3];
	enum ec_rgb_mode rgb_mode;
	u8 rgb_speed;
	bool rgb_valid;

	/* Per-profile CPU cap and GPU boost, under limits_lock */
	struct mutex limits_lock;
	struct notifier_block cpufreq_nb;
	struct cpufreq_policy *qos_policy[EC_MAX_POLICIES];
	struct freq_qos_request cpu_qos[EC_MAX_POLICIES];
	struct device *gpu;
	struct dev_pm_qos_request gpu_qos;
	bool gpu_qos_added;

	/* Lid wakeup quirk */
	struct notifier_block pm_nb;
	struct device *lid_wake_dev;	/* wakeup dropped for this sleep */
};

/* Single EC per machine; lets the keyboard driver reach it for Fn+F */
static struct asus_ec *ec_instance;
static DEFINE_MUTEX(ec_instance_lock);

/* ---------------------------------------------------------------------- */
/* Low level I2C                                                          */

static int ec_xfer(struct i2c_client *c, struct i2c_msg *msgs, int num)
{
	int ret = i2c_transfer(c->adapter, msgs, num);

	if (ret < 0)
		return ret;
	return ret == num ? 0 : -EIO;
}

static int ec_cmd_write(struct asus_ec *ec, const u8 *buf, u16 len)
{
	struct i2c_msg msg = {
		.addr = ec->client->addr,
		.len = len,
		.buf = (u8 *)buf,
	};

	return ec_xfer(ec->client, &msg, 1);
}

static int ec_ram_read(struct asus_ec *ec, u8 page, u8 off, u8 *val)
{
	u8 sel[3] = { EC_RAM_SELECT, page, off };
	u8 cmd = EC_RAM_DATA;
	struct i2c_msg msgs[] = {
		{ .addr = ec->mailbox->addr, .len = sizeof(sel), .buf = sel },
		{ .addr = ec->mailbox->addr, .len = 1, .buf = &cmd },
		{ .addr = ec->mailbox->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};

	return ec_xfer(ec->mailbox, msgs, ARRAY_SIZE(msgs));
}

static int ec_ram_write(struct asus_ec *ec, u8 page, u8 off, u8 val)
{
	u8 sel[3] = { EC_RAM_SELECT, page, off };
	u8 data[2] = { EC_RAM_DATA, val };
	struct i2c_msg msgs[] = {
		{ .addr = ec->mailbox->addr, .len = sizeof(sel), .buf = sel },
		{ .addr = ec->mailbox->addr, .len = sizeof(data), .buf = data },
	};

	return ec_xfer(ec->mailbox, msgs, ARRAY_SIZE(msgs));
}

/* ---------------------------------------------------------------------- */
/* Page 0xC4 command mailbox                                              */

static int ec_mb_settle(struct asus_ec *ec)
{
	int ret, tries = 40;
	u8 op;

	do {
		ret = ec_ram_read(ec, EC_PAGE_CMD, EC_CMD_MB_OP, &op);
		if (ret)
			return ret;
		if (!op)
			return 0;
		usleep_range(5000, 6000);
	} while (--tries);

	return -ETIMEDOUT;
}

static int ec_mb_write(struct asus_ec *ec, u8 bank, u8 reg, u8 val)
{
	int ret;

	lockdep_assert_held(&ec->lock);

	ret = ec_mb_settle(ec);
	if (ret)
		return ret;
	ret = ec_ram_write(ec, EC_PAGE_CMD, EC_CMD_MB_REG, reg);
	if (ret)
		return ret;
	ret = ec_ram_write(ec, EC_PAGE_CMD, EC_CMD_MB_VAL, val);
	if (ret)
		return ret;
	ret = ec_ram_write(ec, EC_PAGE_CMD, EC_CMD_MB_OP, bank);
	if (ret)
		return ret;
	return ec_mb_settle(ec);
}

/* ---------------------------------------------------------------------- */
/* EC operations (caller holds ec->lock)                                  */

static int ec_send_soc_temp(struct asus_ec *ec, int temp_mc)
{
	/* deci-°C, little endian */
	int deci = clamp(DIV_ROUND_CLOSEST(temp_mc, 100), 0, 2000);
	u8 buf[] = { EC_CMD_SOC_TEMP, 0x01, 0x02, deci & 0xff, deci >> 8 };

	return ec_cmd_write(ec, buf, sizeof(buf));
}

static int ec_set_suspend(struct asus_ec *ec, bool suspend)
{
	u8 buf[] = { EC_CMD_SUSPEND, suspend };

	return ec_cmd_write(ec, buf, sizeof(buf));
}

static int ec_get_rpm(struct asus_ec *ec, int fan, long *rpm)
{
	u8 req[] = { EC_CMD_FAN_RPM, fan + 1 };
	u8 resp[3];
	struct i2c_msg msgs[] = {
		{ .addr = ec->client->addr, .len = sizeof(req), .buf = req },
		{ .addr = ec->client->addr, .flags = I2C_M_RD,
		  .len = sizeof(resp), .buf = resp },
	};
	int ret;

	ret = ec_xfer(ec->client, msgs, ARRAY_SIZE(msgs));
	if (ret)
		return ret;

	*rpm = resp[1] | resp[2] << 8;
	return 0;
}

static int ec_write_profile(struct asus_ec *ec, enum ec_profile profile)
{
	u8 buf[] = { EC_CMD_FAN_PROFILE, profile };
	int ret;

	ret = ec_cmd_write(ec, buf, sizeof(buf));
	if (!ret)
		ec->profile = profile;
	return ret;
}

static int ec_set_fan_duty(struct asus_ec *ec, int fan, u8 duty)
{
	int ret;

	ret = ec_mb_write(ec, EC_BANK_FAN, EC_REG_FAN_SELECT, fan);
	if (ret)
		return ret;
	return ec_mb_write(ec, EC_BANK_FAN, EC_REG_FAN_DUTY, duty);
}

static int ec_set_manual(struct asus_ec *ec, bool manual)
{
	int ret, i;

	ret = ec_mb_write(ec, EC_BANK_FAN, EC_REG_FAN_MODE,
			  manual ? EC_FAN_MODE_MANUAL : EC_FAN_MODE_AUTO);
	if (ret)
		return ret;
	ec->manual = manual;

	if (!manual)
		return 0;

	/* Duty is only accepted in manual mode, so apply what was cached */
	for (i = 0; i < EC_NUM_FANS; i++) {
		if (!ec->pwm_valid[i])
			continue;
		ret = ec_set_fan_duty(ec, i, ec->pwm[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int ec_send_rgb(struct asus_ec *ec)
{
	u8 buf[22] = {
		EC_CMD_KBD_RGB, 0x07, 0x66, 0x00, 0x10, 0x00, 0xb3,
		ec->rgb_mode, ec->rgb[0], ec->rgb[1], ec->rgb[2],
		0x55,			/* unknown */
		ec->rgb_speed * 2,
	};

	return ec_cmd_write(ec, buf, sizeof(buf));
}

/* ---------------------------------------------------------------------- */
/* CPU / GPU limits per fan profile                                       */

static s32 ec_cpu_cap(struct asus_ec *ec)
{
	unsigned int khz = profile_cpu_max_khz[ec->profile];

	return khz ? min_t(unsigned int, khz, FREQ_QOS_MAX_DEFAULT_VALUE) :
		     FREQ_QOS_MAX_DEFAULT_VALUE;
}

/* Caller holds limits_lock */
static void ec_apply_limits(struct asus_ec *ec)
{
	s32 cap = ec_cpu_cap(ec);
	bool gpu_max = profile_gpu_max[ec->profile];
	int i;

	for (i = 0; i < EC_MAX_POLICIES; i++)
		if (ec->qos_policy[i])
			freq_qos_update_request(&ec->cpu_qos[i], cap);

	if (ec->gpu_qos_added)
		dev_pm_qos_update_request(&ec->gpu_qos, gpu_max ? S32_MAX :
					  PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);

	dev_dbg(ec->dev, "%s: CPU cap %d kHz, GPU %s\n", ec_profile_names[ec->profile],
		cap, gpu_max ? "at max" : "on demand");
}

/* Caller holds limits_lock */
static void ec_track_policy(struct asus_ec *ec, struct cpufreq_policy *policy)
{
	int i, free = -1;

	for (i = 0; i < EC_MAX_POLICIES; i++) {
		if (ec->qos_policy[i] == policy)
			return;
		if (!ec->qos_policy[i] && free < 0)
			free = i;
	}
	if (free < 0) {
		dev_warn(ec->dev, "too many cpufreq policies, CPU%u not capped\n", policy->cpu);
		return;
	}
	if (freq_qos_add_request(&policy->constraints, &ec->cpu_qos[free],
				 FREQ_QOS_MAX, ec_cpu_cap(ec)) < 0)
		return;
	ec->qos_policy[free] = policy;
}

/* Caller holds limits_lock */
static void ec_untrack_policy(struct asus_ec *ec, struct cpufreq_policy *policy)
{
	int i;

	for (i = 0; i < EC_MAX_POLICIES; i++) {
		if (ec->qos_policy[i] != policy)
			continue;
		freq_qos_remove_request(&ec->cpu_qos[i]);
		ec->qos_policy[i] = NULL;
	}
}

static int ec_cpufreq_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	struct asus_ec *ec = container_of(nb, struct asus_ec, cpufreq_nb);
	struct cpufreq_policy *policy = data;

	guard(mutex)(&ec->limits_lock);
	if (event == CPUFREQ_CREATE_POLICY)
		ec_track_policy(ec, policy);
	else if (event == CPUFREQ_REMOVE_POLICY)
		ec_untrack_policy(ec, policy);
	return NOTIFY_OK;
}

static void ec_limits_exit(void *data)
{
	struct asus_ec *ec = data;
	int i;

	cpufreq_unregister_notifier(&ec->cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);

	guard(mutex)(&ec->limits_lock);
	for (i = 0; i < EC_MAX_POLICIES; i++) {
		if (!ec->qos_policy[i])
			continue;
		freq_qos_remove_request(&ec->cpu_qos[i]);
		ec->qos_policy[i] = NULL;
	}
	if (ec->gpu_qos_added) {
		dev_pm_qos_remove_request(&ec->gpu_qos);
		ec->gpu_qos_added = false;
	}
	put_device(ec->gpu);
	ec->gpu = NULL;
}

static int ec_limits_init(struct asus_ec *ec)
{
	struct device_node *np;
	unsigned int cpu;
	int ret;

	/* The Adreno GPU: its devfreq honours DEV_PM_QOS_MIN_FREQUENCY */
	np = of_find_compatible_node(NULL, NULL, "qcom,adreno");
	if (np) {
		struct platform_device *pdev = of_find_device_by_node(np);

		of_node_put(np);
		if (pdev) {
			ec->gpu = &pdev->dev;
			if (dev_pm_qos_add_request(ec->gpu, &ec->gpu_qos,
						   DEV_PM_QOS_MIN_FREQUENCY,
						   PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE) >= 0)
				ec->gpu_qos_added = true;
		}
	}

	ec->cpufreq_nb.notifier_call = ec_cpufreq_notifier;
	ret = cpufreq_register_notifier(&ec->cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		put_device(ec->gpu);
		ec->gpu = NULL;
		return ret;
	}

	/* Policies that existed before the notifier */
	scoped_guard(mutex, &ec->limits_lock) {
		for_each_possible_cpu(cpu) {
			struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

			if (!policy)
				continue;
			if (policy->cpu == cpu)
				ec_track_policy(ec, policy);
			cpufreq_cpu_put(policy);
		}
		ec_apply_limits(ec);
	}

	return devm_add_action_or_reset(ec->dev, ec_limits_exit, ec);
}

/* ---------------------------------------------------------------------- */
/* Lid wakeup quirk                                                       */

static struct input_handle *ec_lid_handle;
static DEFINE_MUTEX(ec_lid_lock);

static int ec_lid_connect(struct input_handler *handler, struct input_dev *dev,
			  const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	guard(mutex)(&ec_lid_lock);
	if (ec_lid_handle)
		return -EBUSY;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = DRIVER_NAME "-lid";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	/* Opened so the switch state stays current */
	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	ec_lid_handle = handle;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void ec_lid_disconnect(struct input_handle *handle)
{
	scoped_guard(mutex, &ec_lid_lock) {
		if (ec_lid_handle == handle)
			ec_lid_handle = NULL;
	}
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id ec_lid_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_LID)] = BIT_MASK(SW_LID) },
	},
	{ }
};

static struct input_handler ec_lid_handler = {
	.connect	= ec_lid_connect,
	.disconnect	= ec_lid_disconnect,
	.name		= DRIVER_NAME "-lid",
	.id_table	= ec_lid_ids,
};

static int ec_pm_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	struct asus_ec *ec = container_of(nb, struct asus_ec, pm_nb);
	struct device *wake_dev;

	switch (action) {
	case PM_SUSPEND_PREPARE: {
		if (!lid_wake_quirk)
			break;
		guard(mutex)(&ec_lid_lock);
		if (!ec_lid_handle || test_bit(SW_LID, ec_lid_handle->dev->sw))
			break;	/* no lid switch, or the lid is closed */
		wake_dev = ec_lid_handle->dev->dev.parent;
		if (!wake_dev || !device_may_wakeup(wake_dev))
			break;
		device_set_wakeup_enable(wake_dev, false);
		ec->lid_wake_dev = get_device(wake_dev);
		dev_dbg(ec->dev, "lid open: lid wakeup off for this sleep\n");
		break;
	}
	case PM_POST_SUSPEND:
		if (!ec->lid_wake_dev)
			break;
		device_set_wakeup_enable(ec->lid_wake_dev, true);
		put_device(ec->lid_wake_dev);
		ec->lid_wake_dev = NULL;
		break;
	}
	return NOTIFY_DONE;
}

static void ec_lid_exit(void *data)
{
	struct asus_ec *ec = data;

	unregister_pm_notifier(&ec->pm_nb);
	if (ec->lid_wake_dev) {
		device_set_wakeup_enable(ec->lid_wake_dev, true);
		put_device(ec->lid_wake_dev);
		ec->lid_wake_dev = NULL;
	}
	input_unregister_handler(&ec_lid_handler);
}

static int ec_lid_init(struct asus_ec *ec)
{
	int ret;

	ret = input_register_handler(&ec_lid_handler);
	if (ret)
		return ret;

	ec->pm_nb.notifier_call = ec_pm_notifier;
	ret = register_pm_notifier(&ec->pm_nb);
	if (ret) {
		input_unregister_handler(&ec_lid_handler);
		return ret;
	}

	return devm_add_action_or_reset(ec->dev, ec_lid_exit, ec);
}

/* ---------------------------------------------------------------------- */
/* Fan profile                                                            */

static int ec_set_profile(struct asus_ec *ec, unsigned int profile)
{
	int ret;

	if (profile >= EC_PROFILE_COUNT)
		return -EINVAL;

	mutex_lock(&ec->lock);
	ret = ec_write_profile(ec, profile);
	mutex_unlock(&ec->lock);
	if (ret)
		return ret;

	dev_dbg(ec->dev, "fan profile: %s\n", ec_profile_names[profile]);
	scoped_guard(mutex, &ec->limits_lock)
		ec_apply_limits(ec);
	sysfs_notify(&ec->dev->kobj, NULL, "fan_profile");
	return 0;
}

int asus_vivobook_ec_cycle_fan_profile(void)
{
	unsigned int next;
	int ret = -ENODEV;

	mutex_lock(&ec_instance_lock);
	if (ec_instance) {
		next = (ec_instance->profile + 1) % EC_PROFILE_COUNT;
		ret = ec_set_profile(ec_instance, next);
		if (!ret)
			ret = next;
	}
	mutex_unlock(&ec_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(asus_vivobook_ec_cycle_fan_profile);

/* ---------------------------------------------------------------------- */
/* SoC temperature feed                                                   */

static int ec_read_soc_temp(struct asus_ec *ec, const char **source)
{
	struct thermal_zone_device *tz;
	int i, t, hottest = INT_MIN;

	*source = NULL;

	/* Looked up every time so late-registered zones are picked up */
	for (i = 0; i < ec->nr_zones; i++) {
		tz = thermal_zone_get_zone_by_name(ec->zones[i].name);
		if (IS_ERR(tz))
			continue;
		if (thermal_zone_get_temp(tz, &t))
			continue;
		t += ec->zones[i].offset_mc;
		if (t > hottest) {
			hottest = t;
			*source = ec->zones[i].name;
		}
	}

	if (hottest != INT_MIN) {
		ec->no_zone_warned = false;
		return hottest;
	}

	if (!ec->no_zone_warned) {
		dev_warn(ec->dev, "no readable thermal zone, reporting %d m°C\n",
			 fallback_temp_mc);
		ec->no_zone_warned = true;
	}
	return fallback_temp_mc;
}

static void ec_temp_work(struct work_struct *work)
{
	struct asus_ec *ec = container_of(to_delayed_work(work),
					  struct asus_ec, temp_work);
	const char *source;
	int temp = ec_read_soc_temp(ec, &source);
	int ret;

	mutex_lock(&ec->lock);
	ret = ec_send_soc_temp(ec, temp);
	if (!ret) {
		ec->last_temp_mc = temp;
		ec->last_source = source;
	}
	mutex_unlock(&ec->lock);

	if (ret)
		dev_warn_ratelimited(ec->dev, "failed to report temperature: %d\n", ret);

	schedule_delayed_work(&ec->temp_work, msecs_to_jiffies(EC_TEMP_PERIOD_MS));
}

/* ---------------------------------------------------------------------- */
/* hwmon                                                                  */

static const char * const ec_fan_labels[EC_NUM_FANS] = { "Left Fan", "Right Fan" };

static umode_t ec_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
	case hwmon_temp:
		return 0444;
	case hwmon_pwm:
		return 0644;
	default:
		return 0;
	}
}

static int ec_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ec->lock);

	switch (type) {
	case hwmon_fan:
		ret = ec_get_rpm(ec, channel, val);
		break;

	case hwmon_pwm:
		if (attr == hwmon_pwm_input)
			*val = ec->pwm[channel];
		else
			*val = ec->manual ? 1 : 2;
		break;

	case hwmon_temp:
		*val = ec->last_temp_mc;
		break;

	default:
		ret = -EOPNOTSUPP;
	}

	mutex_unlock(&ec->lock);
	return ret;
}

static int ec_hwmon_read_string(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_fan:
		*str = ec_fan_labels[channel];
		return 0;
	case hwmon_temp:
		*str = "Reported to EC";
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int ec_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret = 0;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	mutex_lock(&ec->lock);

	if (attr == hwmon_pwm_input) {
		ec->pwm[channel] = clamp_val(val, 0, 255);
		ec->pwm_valid[channel] = true;
		/* The EC ignores duty writes in auto mode; keep it cached */
		if (ec->manual)
			ret = ec_set_fan_duty(ec, channel, ec->pwm[channel]);
		goto out;
	}

	/* pwmN_enable: the fan mode is global, it affects both fans */
	switch (val) {
	case 1:
		ret = ec_set_manual(ec, true);
		break;
	case 2:
		ret = ec_set_manual(ec, false);
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&ec->lock);
	return ret;
}

static const struct hwmon_ops ec_hwmon_ops = {
	.is_visible = ec_hwmon_is_visible,
	.read = ec_hwmon_read,
	.read_string = ec_hwmon_read_string,
	.write = ec_hwmon_write,
};

static const struct hwmon_channel_info * const ec_hwmon_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_chip_info ec_hwmon_chip = {
	.ops = &ec_hwmon_ops,
	.info = ec_hwmon_info,
};

/* ---------------------------------------------------------------------- */
/* Device sysfs: fan_profile, kbd_rgb_*, ec_temp_source                   */

static ssize_t fan_profile_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int i, len = 0;

	for (i = 0; i < EC_PROFILE_COUNT; i++)
		len += sysfs_emit_at(buf, len, i == ec->profile ? "[%s] " : "%s ",
				     ec_profile_names[i]);
	buf[len - 1] = '\n';
	return len;
}

static ssize_t fan_profile_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	unsigned int profile;
	int ret;

	ret = sysfs_match_string(ec_profile_names, buf);
	if (ret >= 0)
		profile = ret;
	else if (kstrtouint(buf, 0, &profile))
		return -EINVAL;

	ret = ec_set_profile(ec, profile);
	return ret ?: count;
}
static DEVICE_ATTR_RW(fan_profile);

static ssize_t kbd_rgb_color_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);

	if (!ec->rgb_valid)
		return -ENODATA;
	return sysfs_emit(buf, "%02x%02x%02x\n", ec->rgb[0], ec->rgb[1], ec->rgb[2]);
}

static ssize_t kbd_rgb_color_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 rgb[3];
	int ret;

	if (*buf == '#')
		buf++;
	if (sscanf(buf, "%2hhx%2hhx%2hhx", &rgb[0], &rgb[1], &rgb[2]) != 3)
		return -EINVAL;

	mutex_lock(&ec->lock);
	memcpy(ec->rgb, rgb, sizeof(rgb));
	ec->rgb_valid = true;
	ret = ec_send_rgb(ec);
	mutex_unlock(&ec->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(kbd_rgb_color);

static ssize_t kbd_rgb_mode_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(ec_rgb_mode_names); i++)
		len += sysfs_emit_at(buf, len,
				     i + EC_RGB_SOLID == ec->rgb_mode ? "[%s] " : "%s ",
				     ec_rgb_mode_names[i]);
	buf[len - 1] = '\n';
	return len;
}

static ssize_t kbd_rgb_mode_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int mode, ret;

	mode = sysfs_match_string(ec_rgb_mode_names, buf);
	if (mode < 0)
		return mode;

	mutex_lock(&ec->lock);
	ec->rgb_mode = mode + EC_RGB_SOLID;
	ret = ec->rgb_valid ? ec_send_rgb(ec) : 0;
	mutex_unlock(&ec->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(kbd_rgb_mode);

static ssize_t kbd_rgb_speed_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ec->rgb_speed);
}

static ssize_t kbd_rgb_speed_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	u8 speed;
	int ret;

	ret = kstrtou8(buf, 10, &speed);
	if (ret)
		return ret;
	if (!speed || speed > 127)
		return -EINVAL;

	mutex_lock(&ec->lock);
	ec->rgb_speed = speed;
	ret = ec->rgb_valid ? ec_send_rgb(ec) : 0;
	mutex_unlock(&ec->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(kbd_rgb_speed);

/* "<zone> <m°C reported to the EC>", the zone that currently drives the fans */
static ssize_t ec_temp_source_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&ec->lock);
	len = sysfs_emit(buf, "%s %d\n", ec->last_source ?: "fallback", ec->last_temp_mc);
	mutex_unlock(&ec->lock);
	return len;
}
static DEVICE_ATTR_RO(ec_temp_source);

static struct attribute *ec_attrs[] = {
	&dev_attr_fan_profile.attr,
	&dev_attr_kbd_rgb_color.attr,
	&dev_attr_kbd_rgb_mode.attr,
	&dev_attr_kbd_rgb_speed.attr,
	&dev_attr_ec_temp_source.attr,
	NULL
};
ATTRIBUTE_GROUPS(ec);

/* ---------------------------------------------------------------------- */

/* "name[:offset_°C]" */
static int ec_parse_zone(struct device *dev, const char *spec, struct ec_zone *z)
{
	char *name, *colon;
	int offset = 0;

	name = devm_kstrdup(dev, spec, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	colon = strchr(name, ':');
	if (colon) {
		*colon = '\0';
		if (kstrtoint(colon + 1, 10, &offset))
			return -EINVAL;
	}

	z->name = name;
	z->offset_mc = offset * 1000;
	return 0;
}

/*
 * Zone list: the thermal_zones module parameter, else the devicetree
 * "thermal-zone-names" property, else the built-in list.
 */
static int ec_parse_zones(struct asus_ec *ec)
{
	struct device_node *np = ec->dev->of_node;
	const char **specs;
	struct ec_zone *zones;
	int i, n, ret;

	if (nr_thermal_zones > 0) {
		specs = (const char **)thermal_zones;
		n = nr_thermal_zones;
	} else {
		n = np ? of_property_count_strings(np, "thermal-zone-names") : -EINVAL;
		if (n <= 0) {
			ec->zones = ec_default_zones;
			ec->nr_zones = ARRAY_SIZE(ec_default_zones);
			return 0;
		}
		specs = devm_kcalloc(ec->dev, n, sizeof(*specs), GFP_KERNEL);
		if (!specs)
			return -ENOMEM;
		n = of_property_read_string_array(np, "thermal-zone-names", specs, n);
		if (n < 0)
			return n;
	}

	zones = devm_kcalloc(ec->dev, n, sizeof(*zones), GFP_KERNEL);
	if (!zones)
		return -ENOMEM;

	for (i = 0; i < n; i++) {
		ret = ec_parse_zone(ec->dev, specs[i], &zones[i]);
		if (ret)
			return ret;
	}

	ec->zones = zones;
	ec->nr_zones = n;
	return 0;
}

static void ec_unregister_mailbox(void *data)
{
	i2c_unregister_device(data);
}

static void ec_clear_instance(void *data)
{
	mutex_lock(&ec_instance_lock);
	ec_instance = NULL;
	mutex_unlock(&ec_instance_lock);
}

static int ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon;
	struct asus_ec *ec;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ec->dev = dev;
	ec->client = client;
	ec->rgb_mode = EC_RGB_SOLID;
	ec->rgb_speed = 3;
	i2c_set_clientdata(client, ec);

	ret = devm_mutex_init(dev, &ec->lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &ec->limits_lock);
	if (ret)
		return ret;

	ret = ec_parse_zones(ec);
	if (ret)
		return dev_err_probe(dev, ret, "invalid thermal-zone-names\n");

	ec->mailbox = i2c_new_ancillary_device(client, "mailbox", EC_MAILBOX_ADDR);
	if (IS_ERR(ec->mailbox))
		return dev_err_probe(dev, PTR_ERR(ec->mailbox),
				     "failed to register mailbox address\n");
	ret = devm_add_action_or_reset(dev, ec_unregister_mailbox, ec->mailbox);
	if (ret)
		return ret;

	/*
	 * Feed the temperature before anything that may fail: the EC shuts
	 * the machine down if it stops getting reports.
	 */
	ret = devm_delayed_work_autocancel(dev, &ec->temp_work, ec_temp_work);
	if (ret)
		return ret;
	schedule_delayed_work(&ec->temp_work, 0);

	mutex_lock(&ec->lock);
	ret = ec_set_manual(ec, false);
	if (!ret)
		ret = ec_write_profile(ec, min_t(unsigned int, fan_profile_default,
						 EC_PROFILE_COUNT - 1));
	mutex_unlock(&ec->lock);
	if (ret)
		dev_warn(dev, "failed to initialise fan control: %d\n", ret);

	ret = ec_limits_init(ec);
	if (ret)
		dev_warn(dev, "CPU/GPU limits per fan profile unavailable: %d\n", ret);

	ret = ec_lid_init(ec);
	if (ret)
		dev_warn(dev, "lid wakeup quirk unavailable: %d\n", ret);

	hwmon = devm_hwmon_device_register_with_info(dev, "asus_vivobook_ec", ec,
						     &ec_hwmon_chip, NULL);
	if (IS_ERR(hwmon))
		return dev_err_probe(dev, PTR_ERR(hwmon), "failed to register hwmon\n");

	mutex_lock(&ec_instance_lock);
	ec_instance = ec;
	mutex_unlock(&ec_instance_lock);

	return devm_add_action_or_reset(dev, ec_clear_instance, NULL);
}

static void ec_remove(struct i2c_client *client)
{
	struct asus_ec *ec = i2c_get_clientdata(client);

	mutex_lock(&ec->lock);
	ec_set_manual(ec, false);
	mutex_unlock(&ec->lock);

	dev_crit(ec->dev,
		 "SoC temperature reports stop now; the EC will power the machine off in a few minutes\n");
}

static int ec_suspend(struct device *dev)
{
	struct asus_ec *ec = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&ec->temp_work);

	mutex_lock(&ec->lock);
	ret = ec_set_suspend(ec, true);
	mutex_unlock(&ec->lock);

	if (ret) {
		dev_err(dev, "failed to put EC into suspend mode: %d\n", ret);
		schedule_delayed_work(&ec->temp_work, 0);
	}
	return ret;
}

static int ec_resume(struct device *dev)
{
	struct asus_ec *ec = dev_get_drvdata(dev);

	mutex_lock(&ec->lock);

	if (ec_set_suspend(ec, false))
		dev_err(dev, "failed to wake EC\n");

	if (ec_write_profile(ec, ec->profile) || ec_set_manual(ec, ec->manual))
		dev_warn(dev, "failed to restore fan state\n");

	if (ec->rgb_valid)
		ec_send_rgb(ec);

	mutex_unlock(&ec->lock);

	schedule_delayed_work(&ec->temp_work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ec_pm_ops, ec_suspend, ec_resume);

static const struct of_device_id ec_of_match[] = {
	{ .compatible = "asus,vivobook-s15-ec" },
	{ }
};
MODULE_DEVICE_TABLE(of, ec_of_match);

static const struct i2c_device_id ec_id[] = {
	{ "vivobook-s15-ec" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ec_id);

static struct i2c_driver ec_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ec_of_match,
		.dev_groups = ec_groups,
		.pm = pm_sleep_ptr(&ec_pm_ops),
	},
	.probe = ec_probe,
	.remove = ec_remove,
	.id_table = ec_id,
};
module_i2c_driver(ec_driver);

MODULE_DESCRIPTION("ASUS Vivobook S15 (Snapdragon X) embedded controller driver");
MODULE_LICENSE("GPL");
