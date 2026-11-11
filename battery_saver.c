// SPDX-License-Identifier: GPL-2.0
/*
 * Adaptive Battery Saver and Power Optimization Module
 * Target: Qualcomm Snapdragon 845 (SDM845) - Xiaomi Poco F1 (beryllium)
 *
 * Provides dynamic CPU frequency clamping and core management to maximize
 * battery endurance while retaining responsive performance. User-facing
 * configuration is intentionally kept in the userspace batsave helper;
 * the kernel module only validates and applies bounded state.
 *
 * Full clean rollback on unload (rmmod) restoring original factory frequencies
 * and hotplugged cores.
 */

#define pr_fmt(fmt) "battery_saver: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/limits.h>

enum bs_profile {
	PROFILE_PASSTHROUGH = 0, /* Limits bypass / stock monitoring */
	PROFILE_BALANCED    = 1, /* Smooth daily use, ~30-40% peak power reduction */
	PROFILE_EXTREME     = 2, /* 2 big cores offline, lower freq cap (~55-65% savings) */
	PROFILE_ULTRA       = 3, /* Emergency battery saver, 1 big core, max low freq */
	PROFILE_CUSTOM      = 4, /* User-defined parameters */
	PROFILE_MAX
};

static const char * const profile_names[] = {
	[PROFILE_PASSTHROUGH] = "Passthrough (Stock / No Limits)",
	[PROFILE_BALANCED]    = "Balanced Powersave (Smooth Daily)",
	[PROFILE_EXTREME]     = "Extreme Powersave (Cores 6-7 Offline)",
	[PROFILE_ULTRA]       = "Ultra Battery Saver (1 Big Core Active)",
	[PROFILE_CUSTOM]      = "Custom User-Defined Profile",
};

struct bs_config {
	int profile;
	bool enabled;
	unsigned int little_max_freq; /* kHz */
	unsigned int little_min_freq; /* kHz */
	unsigned int big_max_freq;    /* kHz */
	unsigned int big_min_freq;    /* kHz */
	unsigned int big_cores_online; /* 1..4 big cores */
};

/* Default to Balanced Powersave */
static struct bs_config cfg = {
	.profile          = PROFILE_BALANCED,
	.enabled          = true,
	.little_max_freq  = 1209600, /* 1.21 GHz */
	.little_min_freq  = 300000,  /* 300 MHz stock min */
	.big_max_freq     = 1574400, /* 1.57 GHz */
	.big_min_freq     = 825600,  /* 825 MHz stock min */
	.big_cores_online = 4,       /* All 4 big cores active */
};

static cpumask_t orig_online_cpus;
static cpumask_t bs_offlined_cpus;
static DEFINE_MUTEX(bs_lock);
static atomic_t adjustments_count = ATOMIC_INIT(0);
static struct freq_qos_request bs_max_qos_reqs[NR_CPUS];
static struct freq_qos_request bs_min_qos_reqs[NR_CPUS];
static DECLARE_BITMAP(bs_qos_added, NR_CPUS);

/* SDM845 Big cluster cores: CPU 4, 5, 6, 7 */
static const unsigned int big_cpu_list[4] = {4, 5, 6, 7};

static void apply_preset_values(int prof)
{
	switch (prof) {
	case PROFILE_PASSTHROUGH:
		cfg.little_max_freq  = 0;
		cfg.little_min_freq  = 0;
		cfg.big_max_freq     = 0;
		cfg.big_min_freq     = 0;
		cfg.big_cores_online = 4;
		break;
	case PROFILE_BALANCED:
		cfg.little_max_freq  = 1209600;
		cfg.little_min_freq  = 300000;
		cfg.big_max_freq     = 1574400;
		cfg.big_min_freq     = 825600;
		cfg.big_cores_online = 4;
		break;
	case PROFILE_EXTREME:
		cfg.little_max_freq  = 1056000;
		cfg.little_min_freq  = 300000;
		cfg.big_max_freq     = 1056000;
		cfg.big_min_freq     = 825600;
		cfg.big_cores_online = 2; /* CPU 4 & 5 online, CPU 6 & 7 offlined */
		break;
	case PROFILE_ULTRA:
		cfg.little_max_freq  = 825600;
		cfg.little_min_freq  = 300000;
		cfg.big_max_freq     = 825600;
		cfg.big_min_freq     = 825600;
		cfg.big_cores_online = 1; /* CPU 4 online, CPU 5, 6 & 7 offlined */
		break;
	case PROFILE_CUSTOM:
	default:
		break;
	}
}

/*
 * Adjust Big cluster CPU online states based on big_cores_online setting.
 * Must NOT be called with the CPU hotplug read lock held.
 */
static void bs_sync_cores_locked(void)
{
	unsigned int target = READ_ONCE(cfg.big_cores_online);
	int i;

	if (!READ_ONCE(cfg.enabled) ||
	    READ_ONCE(cfg.profile) == PROFILE_PASSTHROUGH)
		target = 4;

	if (target > 4)
		target = 4;

	for (i = 0; i < 4; i++) {
		unsigned int cpu = big_cpu_list[i];

		if (cpu >= nr_cpu_ids)
			continue;

		if (i < target) {
			/* Should be online */
			if (!cpu_online(cpu)) {
				pr_info("Bringing CPU%u online\n", cpu);
				if (add_cpu(cpu) == 0)
					cpumask_clear_cpu(cpu, &bs_offlined_cpus);
			}
		} else {
			/* Should be offlined */
			if (cpu_online(cpu)) {
				pr_info("Offlining CPU%u for battery optimization\n", cpu);
				if (remove_cpu(cpu) == 0)
					cpumask_set_cpu(cpu, &bs_offlined_cpus);
			}
		}
	}
}

/*
 * Force cpufreq re-evaluation across all online CPUs.
 */
static void bs_update_policies_locked(void)
{
	unsigned int cpu;
	struct cpufreq_policy *policy;
	unsigned int max_freq, min_freq;
	int ret;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		max_freq = (policy->cpu >= 4) ? cfg.big_max_freq :
						   cfg.little_max_freq;
		min_freq = (policy->cpu >= 4) ? cfg.big_min_freq :
						   cfg.little_min_freq;
		if (!cfg.enabled || cfg.profile == PROFILE_PASSTHROUGH)
			max_freq = min_freq = 0;

		if (max_freq) {
			if (!test_bit(cpu, bs_qos_added)) {
				ret = freq_qos_add_request(&policy->constraints,
							   &bs_max_qos_reqs[cpu],
							   FREQ_QOS_MAX, max_freq);
				if (ret) {
					pr_err("Failed to add CPU%u max-frequency constraint: %d\n",
					       cpu, ret);
					cpufreq_cpu_put(policy);
					continue;
				}
				ret = freq_qos_add_request(&policy->constraints,
							   &bs_min_qos_reqs[cpu],
							   FREQ_QOS_MIN, min_freq);
				if (ret) {
					freq_qos_remove_request(&bs_max_qos_reqs[cpu]);
					pr_err("Failed to add CPU%u min-frequency constraint: %d\n",
					       cpu, ret);
					cpufreq_cpu_put(policy);
					continue;
				}
				set_bit(cpu, bs_qos_added);
			} else {
				ret = freq_qos_update_request(&bs_max_qos_reqs[cpu],
							      max_freq);
				if (ret)
					pr_err("Failed to update CPU%u max-frequency constraint: %d\n",
					       cpu, ret);
				ret = freq_qos_update_request(&bs_min_qos_reqs[cpu],
							      min_freq);
				if (ret)
					pr_err("Failed to update CPU%u min-frequency constraint: %d\n",
					       cpu, ret);
			}
			atomic_inc(&adjustments_count);
		} else if (test_bit(cpu, bs_qos_added)) {
			freq_qos_remove_request(&bs_min_qos_reqs[cpu]);
			freq_qos_remove_request(&bs_max_qos_reqs[cpu]);
			clear_bit(cpu, bs_qos_added);
		}

		cpufreq_cpu_put(policy);
	}
	cpus_read_unlock();
}

static void bs_apply_state(void)
{
	mutex_lock(&bs_lock);
	bs_sync_cores_locked();
	bs_update_policies_locked();
	mutex_unlock(&bs_lock);
}

/*****************************************************************************
 * Sysfs Module Parameter Callbacks
 *****************************************************************************/

/* Profile parameter handler */
static int param_set_profile(const char *val, const struct kernel_param *kp)
{
	int ret, new_profile;

	ret = kstrtoint(val, 0, &new_profile);
	if (ret)
		return ret;

	if (new_profile < 0 || new_profile >= PROFILE_MAX)
		return -EINVAL;

	mutex_lock(&bs_lock);
	cfg.profile = new_profile;
	apply_preset_values(new_profile);
	mutex_unlock(&bs_lock);

	pr_info("Profile set to [%d] %s\n", new_profile, profile_names[new_profile]);
	bs_apply_state();

	return 0;
}

static int param_get_profile(char *buffer, const struct kernel_param *kp)
{
	int p = READ_ONCE(cfg.profile);

	if (p < 0 || p >= PROFILE_MAX)
		p = PROFILE_CUSTOM;

	return scnprintf(buffer, PAGE_SIZE, "%d (%s)\n", p, profile_names[p]);
}

static const struct kernel_param_ops profile_ops = {
	.set = param_set_profile,
	.get = param_get_profile,
};
module_param_cb(profile, &profile_ops, &cfg.profile, 0644);
MODULE_PARM_DESC(profile, "Power profile: 0=Passthrough, 1=Balanced, 2=Extreme, 3=Ultra, 4=Custom");

/* Enabled parameter handler */
static int param_set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret;
	bool new_enabled;

	ret = kstrtobool(val, &new_enabled);
	if (ret)
		return ret;

	mutex_lock(&bs_lock);
	cfg.enabled = new_enabled;
	mutex_unlock(&bs_lock);

	pr_info("Battery saver %s\n", new_enabled ? "ENABLED" : "DISABLED (Bypass mode)");
	bs_apply_state();

	return 0;
}

static int param_get_enabled(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%d\n", READ_ONCE(cfg.enabled) ? 1 : 0);
}

static const struct kernel_param_ops enabled_ops = {
	.set = param_set_enabled,
	.get = param_get_enabled,
};
module_param_cb(enabled, &enabled_ops, &cfg.enabled, 0644);
MODULE_PARM_DESC(enabled, "Enable (1) or bypass (0) battery saver limits");

/* Little cluster max frequency */
static int param_set_little_max(const char *val, const struct kernel_param *kp)
{
	unsigned int freq;
	int ret;

	ret = kstrtouint(val, 0, &freq);
	if (ret)
		return ret;
	if (!freq)
		return -EINVAL;

	mutex_lock(&bs_lock);
	if (freq < cfg.little_min_freq) {
		mutex_unlock(&bs_lock);
		return -EINVAL;
	}
	cfg.little_max_freq = freq;
	cfg.profile = PROFILE_CUSTOM;
	mutex_unlock(&bs_lock);

	bs_apply_state();
	return 0;
}

static int param_get_little_max(char *buffer, const struct kernel_param *kp)
{
	unsigned int value;

	mutex_lock(&bs_lock);
	value = cfg.little_max_freq;
	mutex_unlock(&bs_lock);
	return scnprintf(buffer, PAGE_SIZE, "%u kHz\n", value);
}

static const struct kernel_param_ops little_max_ops = {
	.set = param_set_little_max,
	.get = param_get_little_max,
};
module_param_cb(little_max_freq, &little_max_ops, &cfg.little_max_freq, 0644);
MODULE_PARM_DESC(little_max_freq, "Maximum frequency for Little cluster in kHz");

/* Little cluster min frequency */
static int param_set_little_min(const char *val, const struct kernel_param *kp)
{
	unsigned int freq;
	int ret;

	ret = kstrtouint(val, 0, &freq);
	if (ret)
		return ret;

	mutex_lock(&bs_lock);
	if (!cfg.little_max_freq || freq > cfg.little_max_freq) {
		mutex_unlock(&bs_lock);
		return -EINVAL;
	}
	cfg.little_min_freq = freq;
	cfg.profile = PROFILE_CUSTOM;
	mutex_unlock(&bs_lock);

	bs_apply_state();
	return 0;
}

static int param_get_little_min(char *buffer, const struct kernel_param *kp)
{
	unsigned int value;

	mutex_lock(&bs_lock);
	value = cfg.little_min_freq;
	mutex_unlock(&bs_lock);
	return scnprintf(buffer, PAGE_SIZE, "%u kHz\n", value);
}

static const struct kernel_param_ops little_min_ops = {
	.set = param_set_little_min,
	.get = param_get_little_min,
};
module_param_cb(little_min_freq, &little_min_ops, &cfg.little_min_freq, 0644);
MODULE_PARM_DESC(little_min_freq, "Minimum frequency for Little cluster in kHz");

/* Big cluster max frequency */
static int param_set_big_max(const char *val, const struct kernel_param *kp)
{
	unsigned int freq;
	int ret;

	ret = kstrtouint(val, 0, &freq);
	if (ret)
		return ret;
	if (!freq)
		return -EINVAL;

	mutex_lock(&bs_lock);
	if (freq < cfg.big_min_freq) {
		mutex_unlock(&bs_lock);
		return -EINVAL;
	}
	cfg.big_max_freq = freq;
	cfg.profile = PROFILE_CUSTOM;
	mutex_unlock(&bs_lock);

	bs_apply_state();
	return 0;
}

static int param_get_big_max(char *buffer, const struct kernel_param *kp)
{
	unsigned int value;

	mutex_lock(&bs_lock);
	value = cfg.big_max_freq;
	mutex_unlock(&bs_lock);
	return scnprintf(buffer, PAGE_SIZE, "%u kHz\n", value);
}

static const struct kernel_param_ops big_max_ops = {
	.set = param_set_big_max,
	.get = param_get_big_max,
};
module_param_cb(big_max_freq, &big_max_ops, &cfg.big_max_freq, 0644);
MODULE_PARM_DESC(big_max_freq, "Maximum frequency for Big cluster in kHz");

/* Big cluster min frequency */
static int param_set_big_min(const char *val, const struct kernel_param *kp)
{
	unsigned int freq;
	int ret;

	ret = kstrtouint(val, 0, &freq);
	if (ret)
		return ret;

	mutex_lock(&bs_lock);
	if (!cfg.big_max_freq || freq > cfg.big_max_freq) {
		mutex_unlock(&bs_lock);
		return -EINVAL;
	}
	cfg.big_min_freq = freq;
	cfg.profile = PROFILE_CUSTOM;
	mutex_unlock(&bs_lock);

	bs_apply_state();
	return 0;
}

static int param_get_big_min(char *buffer, const struct kernel_param *kp)
{
	unsigned int value;

	mutex_lock(&bs_lock);
	value = cfg.big_min_freq;
	mutex_unlock(&bs_lock);
	return scnprintf(buffer, PAGE_SIZE, "%u kHz\n", value);
}

static const struct kernel_param_ops big_min_ops = {
	.set = param_set_big_min,
	.get = param_get_big_min,
};
module_param_cb(big_min_freq, &big_min_ops, &cfg.big_min_freq, 0644);
MODULE_PARM_DESC(big_min_freq, "Minimum frequency for Big cluster in kHz");

/* Big cores online count (1..4); keep at least one performance core available. */
static int param_set_big_cores(const char *val, const struct kernel_param *kp)
{
	unsigned int count;
	int ret;

	ret = kstrtouint(val, 0, &count);
	if (ret)
		return ret;

	if (count < 1 || count > 4)
		return -EINVAL;

	mutex_lock(&bs_lock);
	cfg.big_cores_online = count;
	cfg.profile = PROFILE_CUSTOM;
	mutex_unlock(&bs_lock);

	bs_apply_state();
	return 0;
}

static int param_get_big_cores(char *buffer, const struct kernel_param *kp)
{
	unsigned int value;

	mutex_lock(&bs_lock);
	value = cfg.big_cores_online;
	mutex_unlock(&bs_lock);
	return scnprintf(buffer, PAGE_SIZE, "%u (out of 4 big cores)\n", value);
}

static const struct kernel_param_ops big_cores_ops = {
	.set = param_set_big_cores,
	.get = param_get_big_cores,
};
module_param_cb(big_cores_online, &big_cores_ops, &cfg.big_cores_online, 0644);
MODULE_PARM_DESC(big_cores_online, "Number of Big cluster cores online (1..4)");

/* Read-only status report */
static int param_get_status(char *buffer, const struct kernel_param *kp)
{
	int p;
	unsigned int little_online = 0, big_online = 0;
	int i, len = 0;

	mutex_lock(&bs_lock);
	p = cfg.profile;
	mutex_unlock(&bs_lock);

	if (p < 0 || p >= PROFILE_MAX)
		p = PROFILE_CUSTOM;

	for (i = 0; i < 4; i++) {
		if (cpu_online(i))
			little_online++;
	}

	for (i = 4; i < 8; i++) {
		if (cpu_online(i))
			big_online++;
	}

	len += scnprintf(buffer + len, PAGE_SIZE - len,
			 "========================================\n"
			 " SDM845 Adaptive Battery Saver Dashboard\n"
			 "========================================\n"
			 " Status               : %s\n"
			 " Active Profile       : [%d] %s\n"
			 " Policy Adjustments   : %d times enforced\n\n"
			 " [Little Cluster: Silver (CPU 0-3)]\n"
			 "   Active Cores       : %u / 4 online\n"
			 "   Target Limits      : %u kHz - %u kHz\n\n"
			 " [Big Cluster: Gold (CPU 4-7)]\n"
			 "   Active Cores       : %u / 4 online\n"
			 "   Target Limits      : %u kHz - %u kHz\n"
			 "   Offlined by module : CPU4=%s CPU5=%s CPU6=%s CPU7=%s\n"
			 "========================================\n",
			 READ_ONCE(cfg.enabled) ? "ENABLED (Active)" : "DISABLED (Bypass)",
			 p, profile_names[p],
			 atomic_read(&adjustments_count),
			 little_online,
			 READ_ONCE(cfg.little_min_freq), READ_ONCE(cfg.little_max_freq),
			 big_online,
			 READ_ONCE(cfg.big_min_freq), READ_ONCE(cfg.big_max_freq),
			 cpumask_test_cpu(4, &bs_offlined_cpus) ? "OFF" : "ON",
			 cpumask_test_cpu(5, &bs_offlined_cpus) ? "OFF" : "ON",
			 cpumask_test_cpu(6, &bs_offlined_cpus) ? "OFF" : "ON",
			 cpumask_test_cpu(7, &bs_offlined_cpus) ? "OFF" : "ON");

	return len;
}

static const struct kernel_param_ops status_ops = {
	.get = param_get_status,
};
module_param_cb(status, &status_ops, NULL, 0444);
MODULE_PARM_DESC(status, "Read-only summary of battery saver operational state and telemetry");

/*****************************************************************************
 * Module Init & Exit
 *****************************************************************************/

static int __init battery_saver_init(void)
{
	pr_info("Initializing Adaptive Battery Saver for SDM845...\n");

	/* Snapshot original online CPU mask for exact rollback upon unload */
	cpumask_copy(&orig_online_cpus, cpu_online_mask);
	cpumask_clear(&bs_offlined_cpus);

	/* Initialize default preset configuration */
	apply_preset_values(cfg.profile);

	/* Apply initial settings and enforce limits */
	bs_apply_state();

	pr_info("Module loaded successfully. Profile: [%d] %s\n",
		cfg.profile, profile_names[cfg.profile]);
	pr_info("Sysfs control available at /sys/module/battery_saver/parameters/\n");

	return 0;
}

static void __exit battery_saver_exit(void)
{
	unsigned int cpu;

	pr_info("Unloading module, performing clean rollback to stock state...\n");

	/* 1. Remove QoS constraints before restoring stock policies. */
	mutex_lock(&bs_lock);
	for_each_possible_cpu(cpu) {
		if (test_bit(cpu, bs_qos_added)) {
			freq_qos_remove_request(&bs_min_qos_reqs[cpu]);
			freq_qos_remove_request(&bs_max_qos_reqs[cpu]);
			clear_bit(cpu, bs_qos_added);
		}
	}
	mutex_unlock(&bs_lock);

	/* 2. Restore all CPUs that were online before module loaded */
	mutex_lock(&bs_lock);
	for_each_possible_cpu(cpu) {
		if (cpumask_test_cpu(cpu, &orig_online_cpus) && !cpu_online(cpu)) {
			pr_info("Restoring CPU%u to online state\n", cpu);
			add_cpu(cpu);
		}
	}
	cpumask_clear(&bs_offlined_cpus);
	mutex_unlock(&bs_lock);

	/* 3. Re-evaluate policy across all online CPUs to restore stock frequencies */
	cpus_read_lock();
	for_each_online_cpu(cpu) {
		cpufreq_update_policy(cpu);
	}
	cpus_read_unlock();

	pr_info("Rollback complete. Stock CPU limits and core states restored.\n");
}

module_init(battery_saver_init);
module_exit(battery_saver_exit);

MODULE_AUTHOR("Antigravity");
MODULE_DESCRIPTION("Adaptive SDM845 Battery Saver and Power Optimization Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
