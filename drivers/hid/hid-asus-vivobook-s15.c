// SPDX-License-Identifier: GPL-2.0-only
/*
 * HID driver for the ASUS Vivobook S15 (S5507) I2C keyboard.
 *
 * The keyboard is a plain boot keyboard (report 0x36) plus consumer
 * control (0x37), system control (0x35) and the usual ASUS vendor
 * collection (usage page 0xff31, report 0x5a). Fn hotkeys arrive as
 * 0x5a input reports; keyboard backlight and Fn-lock are set with 0x5a
 * feature reports.
 *
 * Hotkeys (Fn-lock off, or with Fn held):
 *   Esc  0x4e  Fn-lock toggle (handled here)
 *   F4   0xc7  keyboard backlight cycle (handled here)
 *   F5   0x10  display brightness down
 *   F6   0x20  display brightness up
 *   F8   0x7e  emoji picker
 *   F9   0x7c  microphone mute
 *   F10  0xcb  camera access toggle
 *   F12  0x86  MyASUS (KEY_PROG1)
 *   F    0x9d  fan profile cycle (forwarded to the EC driver)
 * F1-F3 are consumer usages, F7 is Win+P, Copilot is Win+Shift+F23 and
 * PrtSc a normal key; hid-input handles those.
 *
 * Based on the original driver by Elliot Huang.
 */

#include <linux/bitops.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/asus-vivobook-s15-ec.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define USB_VENDOR_ID_ASUSTEK_I2C	0x0b05
#define I2C_DEVICE_ID_VIVOBOOK_S15_KBD	0x4543

#define VIVO_REPORT_ID			0x5a
#define VIVO_FEATURE_SIZE		64

#define VIVO_KEY_FNLOCK			0x4e
#define VIVO_KEY_KBD_BACKLIGHT		0xc7
#define VIVO_KEY_FAN			0x9d

#define VIVO_KBD_MAX_BRIGHTNESS		3

static bool fnlock_default = true;
module_param(fnlock_default, bool, 0444);
MODULE_PARM_DESC(fnlock_default, "Fn-Lock at boot (Y: F1-F12 act as plain F-keys)");

static unsigned int backlight_default = 1;
module_param(backlight_default, uint, 0444);
MODULE_PARM_DESC(backlight_default, "Keyboard backlight level at boot (0-3)");

enum vivo_work {
	VIVO_WORK_INIT,
	VIVO_WORK_BACKLIGHT,
	VIVO_WORK_FNLOCK,
	VIVO_WORK_FAN,
};

struct vivo_kbd {
	struct hid_device *hdev;
	struct work_struct work;
	unsigned long pending;

	/* Serialises feature reports and the state below */
	struct mutex lock;
	u8 brightness;
	bool fn_lock;

	struct led_classdev led;
};

/* ---------------------------------------------------------------------- */
/* Feature reports                                                        */

static int vivo_send(struct hid_device *hdev, const u8 *data, size_t len)
{
	u8 *buf;
	int ret;

	buf = kzalloc(VIVO_FEATURE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = VIVO_REPORT_ID;
	memcpy(buf + 1, data, len);

	ret = hid_hw_raw_request(hdev, VIVO_REPORT_ID, buf, VIVO_FEATURE_SIZE,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(buf);
	return ret < 0 ? ret : 0;
}

static int vivo_hw_init(struct hid_device *hdev)
{
	/* Query supported functions, then leave OOBE mode */
	static const u8 get_functions[] = { 0x05, 0x20, 0x31, 0x00, 0x08 };
	static const u8 disable_oobe[] = { 0xd0, 0x8f, 0x01 };
	int ret;

	ret = vivo_send(hdev, get_functions, sizeof(get_functions));
	if (ret)
		return ret;
	return vivo_send(hdev, disable_oobe, sizeof(disable_oobe));
}

static int vivo_hw_set_brightness(struct hid_device *hdev, u8 level)
{
	const u8 buf[] = { 0xba, 0xc5, 0xc4, level };

	return vivo_send(hdev, buf, sizeof(buf));
}

static int vivo_hw_set_fn_lock(struct hid_device *hdev, bool on)
{
	const u8 buf[] = { 0xd0, 0x4e, on };

	return vivo_send(hdev, buf, sizeof(buf));
}

/* ---------------------------------------------------------------------- */
/* Keyboard backlight                                                     */

static int vivo_led_set(struct led_classdev *led, enum led_brightness value)
{
	struct vivo_kbd *kbd = container_of(led, struct vivo_kbd, led);
	int ret;

	mutex_lock(&kbd->lock);
	ret = vivo_hw_set_brightness(kbd->hdev, value);
	if (!ret)
		kbd->brightness = value;
	mutex_unlock(&kbd->lock);
	return ret;
}

static enum led_brightness vivo_led_get(struct led_classdev *led)
{
	struct vivo_kbd *kbd = container_of(led, struct vivo_kbd, led);

	return kbd->brightness;
}

/* ---------------------------------------------------------------------- */
/* Deferred work for hotkeys (raw_event runs in atomic context)           */

static int vivo_set_fn_lock(struct vivo_kbd *kbd, bool on)
{
	int ret;

	mutex_lock(&kbd->lock);
	ret = vivo_hw_set_fn_lock(kbd->hdev, on);
	if (!ret)
		kbd->fn_lock = on;
	mutex_unlock(&kbd->lock);

	if (!ret)
		sysfs_notify(&kbd->hdev->dev.kobj, NULL, "fn_lock");
	return ret;
}

static void vivo_restore(struct vivo_kbd *kbd)
{
	struct hid_device *hdev = kbd->hdev;

	mutex_lock(&kbd->lock);
	if (vivo_hw_init(hdev))
		hid_warn(hdev, "keyboard init failed\n");
	if (vivo_hw_set_brightness(hdev, kbd->brightness))
		hid_warn(hdev, "failed to restore backlight\n");
	if (vivo_hw_set_fn_lock(hdev, kbd->fn_lock))
		hid_warn(hdev, "failed to restore Fn-lock\n");
	mutex_unlock(&kbd->lock);
}

static void vivo_work(struct work_struct *work)
{
	struct vivo_kbd *kbd = container_of(work, struct vivo_kbd, work);
	struct hid_device *hdev = kbd->hdev;
	int ret;

	if (test_and_clear_bit(VIVO_WORK_INIT, &kbd->pending))
		vivo_restore(kbd);

	if (test_and_clear_bit(VIVO_WORK_BACKLIGHT, &kbd->pending)) {
		u8 level = (kbd->brightness + 1) % (VIVO_KBD_MAX_BRIGHTNESS + 1);

		ret = vivo_led_set(&kbd->led, level);
		if (ret)
			hid_warn(hdev, "failed to set backlight: %d\n", ret);
		else
			/* Lets UPower/GNOME show the OSD */
			led_classdev_notify_brightness_hw_changed(&kbd->led, level);
	}

	if (test_and_clear_bit(VIVO_WORK_FNLOCK, &kbd->pending)) {
		ret = vivo_set_fn_lock(kbd, !kbd->fn_lock);
		if (ret)
			hid_warn(hdev, "failed to set Fn-lock: %d\n", ret);
	}

	if (test_and_clear_bit(VIVO_WORK_FAN, &kbd->pending)) {
		ret = asus_vivobook_ec_cycle_fan_profile();
		if (ret < 0)
			hid_warn(hdev, "fan profile switch failed: %d\n", ret);
	}
}

static void vivo_schedule(struct vivo_kbd *kbd, enum vivo_work type)
{
	set_bit(type, &kbd->pending);
	schedule_work(&kbd->work);
}

/* ---------------------------------------------------------------------- */
/* HID callbacks                                                          */

static int vivo_raw_event(struct hid_device *hdev, struct hid_report *report,
			  u8 *data, int size)
{
	struct vivo_kbd *kbd = hid_get_drvdata(hdev);

	if (report->type != HID_INPUT_REPORT || size < 2 ||
	    data[0] != VIVO_REPORT_ID)
		return 0;

	/* Only act on the press; the release is an all-zero report */
	switch (data[1]) {
	case VIVO_KEY_FNLOCK:
		vivo_schedule(kbd, VIVO_WORK_FNLOCK);
		break;
	case VIVO_KEY_KBD_BACKLIGHT:
		vivo_schedule(kbd, VIVO_WORK_BACKLIGHT);
		break;
	case VIVO_KEY_FAN:
		vivo_schedule(kbd, VIVO_WORK_FAN);
		break;
	}

	/* Everything else goes through the input mapping below */
	return 0;
}

#define vivo_map_key(c)	hid_map_usage_clear(hi, usage, bit, max, EV_KEY, (c))

static int vivo_input_mapping(struct hid_device *hdev, struct hid_input *hi,
			      struct hid_field *field, struct hid_usage *usage,
			      unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) != HID_UP_ASUSVENDOR)
		return 0;

	switch (usage->hid & HID_USAGE) {
	case 0x10: vivo_map_key(KEY_BRIGHTNESSDOWN);		break;
	case 0x20: vivo_map_key(KEY_BRIGHTNESSUP);		break;
	case 0x7e: vivo_map_key(KEY_EMOJI_PICKER);		break;
	case 0x7c: vivo_map_key(KEY_MICMUTE);			break;
	case 0xcb: vivo_map_key(KEY_CAMERA_ACCESS_TOGGLE);	break;
	case 0x86: vivo_map_key(KEY_PROG1);			break;
	/* Not seen on the S5507, known from other ASUS keyboards */
	case 0x35: vivo_map_key(KEY_DISPLAY_OFF);		break;
	case 0x6b: vivo_map_key(KEY_F21);			break;
	case 0x6c: vivo_map_key(KEY_SLEEP);			break;
	case 0x82: vivo_map_key(KEY_CAMERA);			break;
	case 0x88: vivo_map_key(KEY_RFKILL);			break;
	default:
		/* Fn-lock, backlight and fan are handled in raw_event */
		return -1;
	}
	return 1;
}

/* ---------------------------------------------------------------------- */
/* sysfs                                                                  */

static ssize_t fn_lock_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct vivo_kbd *kbd = hid_get_drvdata(to_hid_device(dev));

	return sysfs_emit(buf, "%d\n", kbd->fn_lock);
}

static ssize_t fn_lock_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct vivo_kbd *kbd = hid_get_drvdata(to_hid_device(dev));
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ret = vivo_set_fn_lock(kbd, on);
	return ret ?: count;
}
static DEVICE_ATTR_RW(fn_lock);

static struct attribute *vivo_attrs[] = {
	&dev_attr_fn_lock.attr,
	NULL
};
ATTRIBUTE_GROUPS(vivo);

/* ---------------------------------------------------------------------- */

static void vivo_stop_hw(void *data)
{
	hid_hw_stop(data);
}

static int vivo_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct vivo_kbd *kbd;
	int ret;

	kbd = devm_kzalloc(&hdev->dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;

	kbd->hdev = hdev;
	kbd->fn_lock = fnlock_default;
	kbd->brightness = min_t(unsigned int, backlight_default,
			       VIVO_KBD_MAX_BRIGHTNESS);
	INIT_WORK(&kbd->work, vivo_work);
	hid_set_drvdata(hdev, kbd);

	ret = devm_mutex_init(&hdev->dev, &kbd->lock);
	if (ret)
		return ret;

	ret = hid_parse(hdev);
	if (ret)
		return dev_err_probe(&hdev->dev, ret, "hid_parse failed\n");

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		return dev_err_probe(&hdev->dev, ret, "hid_hw_start failed\n");

	ret = devm_add_action_or_reset(&hdev->dev, vivo_stop_hw, hdev);
	if (ret)
		return ret;

	kbd->led.name = "asus::kbd_backlight";
	kbd->led.max_brightness = VIVO_KBD_MAX_BRIGHTNESS;
	kbd->led.brightness = kbd->brightness;
	kbd->led.brightness_set_blocking = vivo_led_set;
	kbd->led.brightness_get = vivo_led_get;
	kbd->led.flags = LED_BRIGHT_HW_CHANGED;

	ret = devm_led_classdev_register(&hdev->dev, &kbd->led);
	if (ret)
		return dev_err_probe(&hdev->dev, ret, "failed to register LED\n");

	vivo_schedule(kbd, VIVO_WORK_INIT);
	return 0;
}

static void vivo_remove(struct hid_device *hdev)
{
	struct vivo_kbd *kbd = hid_get_drvdata(hdev);

	cancel_work_sync(&kbd->work);
}

static int vivo_resume(struct hid_device *hdev)
{
	struct vivo_kbd *kbd = hid_get_drvdata(hdev);

	vivo_schedule(kbd, VIVO_WORK_INIT);
	return 0;
}

static const struct hid_device_id vivo_devices[] = {
	{ HID_I2C_DEVICE(USB_VENDOR_ID_ASUSTEK_I2C, I2C_DEVICE_ID_VIVOBOOK_S15_KBD) },
	{ }
};
MODULE_DEVICE_TABLE(hid, vivo_devices);

static struct hid_driver vivo_driver = {
	.name = "hid-asus-vivobook-s15",
	.id_table = vivo_devices,
	.probe = vivo_probe,
	.remove = vivo_remove,
	.raw_event = vivo_raw_event,
	.input_mapping = vivo_input_mapping,
	.resume = pm_ptr(vivo_resume),
	.reset_resume = pm_ptr(vivo_resume),
	.driver = {
		.dev_groups = vivo_groups,
	},
};
module_hid_driver(vivo_driver);

MODULE_AUTHOR("Elliot Huang <keep.it.sns@gmail.com>");
MODULE_DESCRIPTION("HID driver for the ASUS Vivobook S15 S5507 keyboard");
MODULE_LICENSE("GPL");
