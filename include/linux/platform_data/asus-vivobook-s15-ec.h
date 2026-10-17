/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASUS_VIVOBOOK_S15_EC_H
#define __ASUS_VIVOBOOK_S15_EC_H

#include <linux/errno.h>
#include <linux/kconfig.h>

#if IS_REACHABLE(CONFIG_EC_ASUS_VIVOBOOK_S15)
/* Switch to the next fan profile (Fn+F). Returns the new profile or -errno. */
int asus_vivobook_ec_cycle_fan_profile(void);
#else
static inline int asus_vivobook_ec_cycle_fan_profile(void) { return -ENODEV; }
#endif

#endif
