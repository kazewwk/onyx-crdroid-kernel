/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MCA_ONYX_SUPPLY_HOLD_H
#define _MCA_ONYX_SUPPLY_HOLD_H

#include <linux/types.h>

struct device;
struct mutex;
struct mca_votable;

#define ONYX_HOLD_CLIENT "onyx_supply_hold"

enum onyx_hold_role { ONYX_BUCK, ONYX_QUICK, ONYX_GAUGE, ONYX_USB, ONYX_CP, ONYX_NR };
struct onyx_hold_sample {
	int soc; /* raw percent */
	int temp; /* deci-C */
	int ma; /* positive = discharging */
};

struct onyx_hold_source {
	struct device *dev;
	void *data;
	union {
		struct mca_votable *buck;
		struct {
			struct mutex *lock;
			bool (*idle)(void *data);
		} quick;
		int (*pump_on)(void *data, bool *on);
		int (*sample)(void *data, struct onyx_hold_sample *s);
		int (*online)(void *data, int *online);
	};
};

/* Called under the quick strategy data_lock at its final start gate. */
bool onyx_hold_quick_blocked(void);
void onyx_hold_register(enum onyx_hold_role role,
			const struct onyx_hold_source *source);
void onyx_hold_unregister(enum onyx_hold_role role, struct device *dev);
/* IRQ/GLINK safe: no I/O, voting or sleeping in a transport callback. */
void onyx_hold_wire_changed(void);
#endif
