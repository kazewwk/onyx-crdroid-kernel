// SPDX-License-Identifier: GPL-2.0-only
/* Onyx wired charge hold. No adapter, power-path or charge-pump setters. */
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <mca/common/mca_onyx_supply_hold.h>
#include <mca/common/mca_voter.h>

#define POLL_MS		2000
#define CURRENT_MA	100
#define BAD_SAMPLES	3

/* life_lock serializes sysfs lifetime; callbacks take only hold_lock. */
static DEFINE_MUTEX(life_lock);
static DEFINE_MUTEX(hold_lock);
static struct onyx_hold_source src[ONYX_NR];
static struct kobject *hold_kobj;
static struct mca_votable *inhibit;
static bool enabled, ready, suspended, buck_voted, quick_voted;
static const char *reason = "off";
static int discharge_samples, charge_samples;
static atomic_t wire_epoch = ATOMIC_INIT(0);
static int session_epoch;
static void hold_poll(struct work_struct *work);
static DECLARE_DELAYED_WORK(poll_work, hold_poll);
static void wire_check(struct work_struct *work);
static DECLARE_WORK(wire_work, wire_check);

bool onyx_hold_quick_blocked(void)
{
	return READ_ONCE(enabled) && mca_get_effective_result(inhibit) != 0;
}
EXPORT_SYMBOL_GPL(onyx_hold_quick_blocked);

/* MCA commits the ballot before calling hardware, even on callback failure. */
static int release(const char *why)
{
	int rc = 0;

	if (buck_voted)
		rc = mca_vote(src[ONYX_BUCK].buck, ONYX_HOLD_CLIENT, false, 0);
	if (quick_voted && mca_vote(inhibit, ONYX_HOLD_CLIENT, false, 0))
		rc = -EIO;
	buck_voted = false;
	quick_voted = false;
	discharge_samples = charge_samples = 0;
	reason = rc ? "vote_error" : why;
	return rc ? -EIO : 0;
}

static const char *sample(struct onyx_hold_sample *s)
{
	int online = -1;

	if (src[ONYX_USB].online(src[ONYX_USB].data, &online) ||
	    (online != 0 && online != 1))
		return "sensor_error";
	if (!online)
		return "unplugged";
	if (atomic_read(&wire_epoch) != session_epoch)
		return "wire_changed";
	if (src[ONYX_GAUGE].sample(src[ONYX_GAUGE].data, s))
		return "sensor_error";
	if (s->temp < 100)
		return "temp_low";
	if (s->temp >= 400)
		return "temp_high";
	if (s->soc < 30)
		return "low_soc";
	return NULL;
}

static void hold_poll(struct work_struct *work)
{
	struct onyx_hold_sample s;
	const char *why;
	bool pump = true;

	mutex_lock(&hold_lock);
	if (!buck_voted)
		goto out;
	why = sample(&s);
	if (!why && src[ONYX_CP].pump_on(src[ONYX_CP].data, &pump))
		why = "sensor_error";
	if (!why && pump)
		why = "fast_active";
	if (!why && (!onyx_hold_quick_blocked() ||
		     mca_get_effective_result(src[ONYX_BUCK].buck) != 0))
		why = "vote_error";
	if (!why) {
		discharge_samples = s.ma >= CURRENT_MA ? discharge_samples + 1 : 0;
		charge_samples = s.ma <= -CURRENT_MA ? charge_samples + 1 : 0;
		if (discharge_samples >= BAD_SAMPLES)
			why = "discharging";
		else if (charge_samples >= BAD_SAMPLES)
			why = "still_charging";
	}
	if (why)
		release(why);
	else
		schedule_delayed_work(&poll_work, msecs_to_jiffies(POLL_MS));
out:
	mutex_unlock(&hold_lock);
}

static void wire_check(struct work_struct *work)
{
	int online = -1;

	mutex_lock(&hold_lock);
	if (buck_voted && atomic_read(&wire_epoch) != session_epoch) {
		int rc = src[ONYX_USB].online(src[ONYX_USB].data, &online);

		release(rc || (online != 0 && online != 1) ? "sensor_error" :
			online ? "wire_changed" : "unplugged");
	}
	mutex_unlock(&hold_lock);
}

void onyx_hold_wire_changed(void)
{
	if (READ_ONCE(enabled)) {
		atomic_inc(&wire_epoch);
		if (READ_ONCE(ready))
			schedule_work(&wire_work);
	}
}
EXPORT_SYMBOL_GPL(onyx_hold_wire_changed);

static int acquire(void)
{
	struct onyx_hold_sample s;
	const char *why;
	bool pump = true;
	int rc;

	if (!ready || suspended) {
		reason = suspended ? "suspended" : "sensor_error";
		return -ENODEV;
	}
	if (buck_voted)
		return 0; /* A duplicate write must not reset the safety counters. */
	session_epoch = atomic_read(&wire_epoch);
	why = sample(&s);
	if (why) {
		reason = why;
		return -EAGAIN;
	}
	/* Do not force-stop CP: the OEM teardown can request fixed 5 then 9 V. */
	if (!mutex_trylock(src[ONYX_QUICK].quick.lock)) {
		reason = "fast_active";
		return -EBUSY;
	}
	if (src[ONYX_CP].pump_on(src[ONYX_CP].data, &pump)) {
		reason = "sensor_error";
		rc = -EIO;
	} else if (pump || !src[ONYX_QUICK].quick.idle(src[ONYX_QUICK].data)) {
		reason = "fast_active";
		rc = -EBUSY;
	} else {
		quick_voted = true;
		rc = mca_vote(inhibit, ONYX_HOLD_CLIENT, true, 1);
		if (rc || !onyx_hold_quick_blocked()) {
			release("vote_error");
			rc = -EIO;
		}
	}
	mutex_unlock(src[ONYX_QUICK].quick.lock);
	if (rc)
		return rc;
	/* Roll back even if the buck election's hardware callback fails. */
	buck_voted = true;
	rc = mca_vote(src[ONYX_BUCK].buck, ONYX_HOLD_CLIENT, true, 0);
	/* A numerically unchanged election need not have run its hardware callback.
	 * Replay only the current aggregate; never enable a path around other voters.
	 */
	if (rc || mca_get_effective_result(src[ONYX_BUCK].buck) != 0 ||
	    mca_rerun_election(src[ONYX_BUCK].buck)) {
		release("vote_error");
		return -EIO;
	}
	if (atomic_read(&wire_epoch) != session_epoch) {
		release("wire_changed");
		return -EAGAIN;
	}
	reason = "holding";
	mod_delayed_work(system_wq, &poll_work, msecs_to_jiffies(POLL_MS));
	return 0;
}

static ssize_t request_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	ssize_t n;

	mutex_lock(&hold_lock);
	n = sysfs_emit(buf, "%d\n", buck_voted && quick_voted);
	mutex_unlock(&hold_lock);
	return n;
}

static ssize_t request_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	int rc = 0;

	if ((count != 1 && (count != 2 || buf[1] != '\n')) ||
	    (buf[0] != '0' && buf[0] != '1'))
		return -EINVAL;
	mutex_lock(&hold_lock);
	if (buf[0] == '0')
		rc = release("off");
	else
		rc = acquire();
	mutex_unlock(&hold_lock);
	return rc < 0 ? rc : (ssize_t)count;
}

static ssize_t status_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	ssize_t n;
	bool held;

	mutex_lock(&hold_lock);
	held = buck_voted && quick_voted;
	n = sysfs_emit(buf, "requested=%d voted=%d reason=%s\n", held, held, reason);
	mutex_unlock(&hold_lock);
	return n;
}

static struct kobj_attribute request_attr = __ATTR_RW(request);
static struct kobj_attribute status_attr = __ATTR_RO(status);
static struct attribute *hold_attrs[] = { &request_attr.attr, &status_attr.attr, NULL };
static const struct attribute_group hold_group = { .attrs = hold_attrs };

void onyx_hold_register(enum onyx_hold_role role,
			const struct onyx_hold_source *source)
{
	int i;

	if (!READ_ONCE(enabled))
		return;
	mutex_lock(&life_lock);
	mutex_lock(&hold_lock);
	if (src[role].dev)
		goto out;
	src[role] = *source;
	for (i = 0; i < ONYX_NR; i++)
		if (!src[i].dev)
			goto out;
	/* Creation can remove an already-visible attribute on partial failure.
	 * Such a removal must not wait on a reader blocked by our hold_lock.
	 * life_lock keeps providers stable; acquire() still requires ready.
	 */
	mutex_unlock(&hold_lock);
	hold_kobj = kobject_create_and_add("onyx_supply_hold", kernel_kobj);
	if (hold_kobj && !sysfs_create_group(hold_kobj, &hold_group)) {
		mutex_lock(&hold_lock);
		reason = "off";
		WRITE_ONCE(ready, true);
		mutex_unlock(&hold_lock);
	} else {
		kobject_put(hold_kobj);
		hold_kobj = NULL;
		dev_warn(source->dev, "wired hold attributes unavailable\n");
	}
	mutex_unlock(&life_lock);
	return;
out:
	mutex_unlock(&hold_lock);
	mutex_unlock(&life_lock);
}
EXPORT_SYMBOL_GPL(onyx_hold_register);

void onyx_hold_unregister(enum onyx_hold_role role, struct device *dev)
{
	struct kobject *old = NULL;

	mutex_lock(&life_lock);
	mutex_lock(&hold_lock);
	if (src[role].dev == dev) {
		WRITE_ONCE(ready, false);
		release("sensor_error");
		memset(&src[role], 0, sizeof(src[role]));
		old = hold_kobj;
		hold_kobj = NULL;
	}
	mutex_unlock(&hold_lock);
	/* Removing sysfs may wait for a store that needs hold_lock. */
	if (old) {
		sysfs_remove_group(old, &hold_group);
		kobject_put(old);
	}
	mutex_unlock(&life_lock);
}
EXPORT_SYMBOL_GPL(onyx_hold_unregister);

static int hold_pm(struct notifier_block *nb, unsigned long event, void *arg)
{
	mutex_lock(&hold_lock);
	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
		suspended = true;
		if (buck_voted)
			release("suspended");
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		suspended = false;
		break;
	}
	mutex_unlock(&hold_lock);
	return NOTIFY_OK;
}
static struct notifier_block hold_pm_nb = { .notifier_call = hold_pm };

static int __init onyx_hold_init(void)
{
	const char *model;

	if (of_property_read_string(of_root, "model", &model) ||
	    strcmp(model, "Qualcomm Technologies, Inc. Onyx based on SM8735"))
		return 0;
	/* No callback: this private vote only prevents entrance to fast charge. */
	inhibit = mca_create_votable("onyx_quick_inhibit", MCA_VOTE_OR,
				     NULL, 0, NULL);
	if (!IS_ERR_OR_NULL(inhibit)) {
		if (!register_pm_notifier(&hold_pm_nb))
			WRITE_ONCE(enabled, true);
		else
			mca_destroy_votable(inhibit);
	}
	return 0; /* This optional interface must never prevent OEM probing. */
}
module_init(onyx_hold_init);

static void __exit onyx_hold_exit(void)
{
	if (!READ_ONCE(enabled))
		return;
	WRITE_ONCE(enabled, false);
	unregister_pm_notifier(&hold_pm_nb);
	cancel_work_sync(&wire_work);
	cancel_delayed_work_sync(&poll_work);
	mca_destroy_votable(inhibit);
}
module_exit(onyx_hold_exit);
