// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <linux/overflow.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <trace/events/power.h>
#include <trace/events/sched.h>

#include "sched.h"
#include "tune.h"
#include "cpufreq_schedutil.h"

static struct cpufreq_governor schedhorizon_gov;
unsigned long boosted_cpu_util(int cpu);

/*
 * cpufreq_notifier_fp is defined and exported by cpufreq_schedutil.c.
 * Only declare it here as extern so both governors can be built into the
 * same kernel image without a duplicate-symbol link error.
 */
extern void (*cpufreq_notifier_fp)(int cluster_id, unsigned long freq);

#define SUGOV_KTHREAD_PRIORITY	50

struct sugov_efficient_freq {
	int			n;
	unsigned int		freq[];
};

struct sugov_up_delay {
	int			n;
	u64			delay[];
};

struct sugov_tunables {
	struct gov_attr_set attr_set;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	struct sugov_efficient_freq __rcu *efficient_freq;
	struct sugov_up_delay __rcu *up_delay;
};

struct sugov_policy {
	struct cpufreq_policy *policy;

	struct sugov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;  /* For shared policies */
	u64 last_freq_update_time;
	s64 min_rate_limit_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;
	u64 first_hp_request_time;
	int			current_step;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool need_freq_update;
};

struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	unsigned int iowait_boost_max;
	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;
	unsigned long min_boost;

	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	struct cpufreq_policy *policy = sg_policy->policy;

	if (policy->governor != &schedhorizon_gov ||
	    !policy->governor_data)
		return false;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-cpu data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-cpu
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * For the slow switching platforms, the kthread is always scheduled on
	 * the right set of CPUs and any CPU can find the next frequency and
	 * schedule the kthread.
	 */
	if (sg_policy->policy->fast_switch_enabled &&
	    !cpufreq_can_do_remote_dvfs(sg_policy->policy))
		return false;

	if (sg_policy->work_in_progress)
		return false;

	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}

	/* No need to recalculate next freq for min_rate_limit_us
	 * at least. However we might still decide to further rate
	 * limit once frequency change direction is decided, according
	 * to the separate rate limits.
	 */

	delta_ns = time - sg_policy->last_freq_update_time;
	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - sg_policy->last_freq_update_time;

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static void sugov_update_commit(struct sugov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	int cid = arch_get_cluster_id(policy->cpu);

	if (sg_policy->next_freq == next_freq)
		return;

	if (sugov_up_down_rate_limit(sg_policy, time, next_freq))
		return;

	if (cpufreq_notifier_fp)
		cpufreq_notifier_fp(cid, next_freq);

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	mt_cpufreq_set_by_wfi_load_cluster(cid, next_freq);
	policy->cur = next_freq;
	trace_sched_util(cid, next_freq, time);
#else
	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (!next_freq)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
#endif

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;
}

static inline int match_nearest_efficient_step(unsigned int freq, int maxstep,
					       const unsigned int *freq_table)
{
	int i;

	for (i = 0; i < maxstep; i++) {
		if (freq_table[i] >= freq)
			break;
	}

	return min(i, maxstep - 1);
}

/*
 * do_freq_limit - schedhorizon efficient-frequency stepping.
 *
 * Restricts the requested frequency to a per-cluster "efficient" step
 * ladder (tunables->efficient_freq[]), only allowing escalation to the
 * next step once tunables->up_delay[current_step] has elapsed at the
 * current step. This keeps the CPU parked at power-efficient frequencies
 * unless sustained demand justifies ramping up, while never restricting
 * downward transitions.
 */
static inline void do_freq_limit(struct sugov_policy *sg_policy, unsigned int *freq, u64 time)
{
	struct sugov_tunables *tunables = sg_policy->tunables;
	const struct sugov_efficient_freq *ef;
	const struct sugov_up_delay *ud;
	int step, max_step;

	ef = rcu_dereference_sched(tunables->efficient_freq);
	ud = rcu_dereference_sched(tunables->up_delay);
	if (!ef || !ud || ef->n < 1 || ud->n < 1 || !ef->freq[0])
		return;

	max_step = min(ef->n, ud->n) - 1;
	step = clamp(sg_policy->current_step, 0, max_step);
	sg_policy->current_step = step;

	if (*freq > ef->freq[step] && !sg_policy->first_hp_request_time) {
		/* First request above the current efficient step */
		*freq = ef->freq[step];
		sg_policy->first_hp_request_time = time;
		return;
	}

	if (*freq < ef->freq[step]) {
		/* Already under the current efficient frequency: drop down */
		sg_policy->current_step =
			match_nearest_efficient_step(*freq, max_step + 1,
						     ef->freq);
		sg_policy->first_hp_request_time = 0;
		return;
	}

	if (sg_policy->first_hp_request_time &&
	    time < sg_policy->first_hp_request_time + ud->delay[step]) {
		/* Still within the hold-off window: restrict to current step */
		*freq = ef->freq[step];
		return;
	}

	if (step < max_step) {
		/* Hold-off window elapsed: unlock the next step */
		sg_policy->current_step = ++step;
		sg_policy->first_hp_request_time = time;
		if (*freq > ef->freq[step])
			*freq = ef->freq[step];
	}
}

#ifdef CONFIG_NONLINEAR_FREQ_CTL
/*
 * NOTE: schedhorizon's efficient_freq/up_delay stepping (do_freq_limit())
 * is NOT wired into cpufreq_schedutil_plus.c's upower-table-based
 * get_next_freq(). If this config is ever enabled, the stepping tunables
 * will silently have no effect. Not used on this kernel as of this port
 * (confirmed off), so left as upstream stock behavior.
 */
#include "cpufreq_schedutil_plus.c"
#else
/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 * @time: Timestamp of the current update, used for efficient-freq stepping.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;
#ifndef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	unsigned int idx, l_freq, h_freq;
#endif

	freq = freq * util / max;
	freq = freq / SCHED_CAPACITY_SCALE * capacity_margin;

	do_freq_limit(sg_policy, &freq, time);

	sg_policy->cached_raw_freq = freq;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	/*
	 * SSPM firmware resolves the raw frequency to the closest supported
	 * OPP itself (mt_cpufreq_find_close_freq() at the call site), so the
	 * freq_table-based "round down if <20% away" step below does not
	 * apply here and would just do redundant table lookups.
	 */
	return freq;
#else
	l_freq = cpufreq_driver_resolve_freq(policy, freq);
	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_H);
	h_freq = policy->freq_table[idx].frequency;
	h_freq = clamp(h_freq, policy->min, policy->max);
	if (l_freq <= h_freq || l_freq == policy->min)
		return l_freq;

	/*
	 * Use the frequency step below if the calculated frequency is <20%
	 * higher than it.
	 */
	if (mult_frac(100, freq - h_freq, l_freq - h_freq) < 20)
		return h_freq;

	return l_freq;
#endif
}
#endif

static void sugov_get_util(unsigned long *util, unsigned long *max, int cpu)
{
	unsigned long max_cap;

	max_cap = arch_scale_cpu_capacity(NULL, cpu);

	*util = boosted_cpu_util(cpu);
	if (idle_cpu(cpu))
		*util = 0;

	*util = min(*util, max_cap);
	*max = max_cap;
}

static void sugov_set_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	unsigned int max_boost;

	if (flags & SCHED_CPUFREQ_IOWAIT) {
		if (sg_cpu->iowait_boost_pending)
			return;

		sg_cpu->iowait_boost_pending = true;

		/*
		 * Boost FAIR tasks only up to the CPU clamped utilization.
		 *
		 * Since DL tasks have a much more advanced bandwidth control,
		 * it's safe to assume that IO boost does not apply to
		 * those tasks.
		 * Instead, since RT tasks are currently not utiliation clamped,
		 * we don't want to apply clamping on IO boost while there is
		 * blocked RT utilization.
		 */
		max_boost = sg_cpu->iowait_boost_max;
		max_boost = uclamp_util(cpu_rq(sg_cpu->cpu), max_boost);

		if (sg_cpu->iowait_boost) {
			sg_cpu->iowait_boost <<= 1;
			if (sg_cpu->iowait_boost > max_boost)
				sg_cpu->iowait_boost = max_boost;
		} else {
			sg_cpu->iowait_boost = sg_cpu->min_boost;
		}
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;

		/* Clear iowait_boost if the CPU apprears to have been idle. */
		if (delta_ns > TICK_NSEC) {
			sg_cpu->iowait_boost = 0;
			sg_cpu->iowait_boost_pending = false;
		}
	}
}

static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned int boost_util, boost_max;

	if (!sg_cpu->iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost_pending = false;
	} else {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < sg_cpu->min_boost) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	boost_util = sg_cpu->iowait_boost;
	boost_max = sg_cpu->iowait_boost_max;

	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cid;
#endif

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	busy = sugov_cpu_is_busy(sg_cpu);

	if (flags & SCHED_CPUFREQ_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		sugov_get_util(&util, &max, sg_cpu->cpu);
		util = uclamp_util(cpu_rq(sg_cpu->cpu), util);
		sugov_iowait_boost(sg_cpu, &util, &max);
		next_f = get_next_freq(sg_policy, util, max, time);
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
		next_f = clamp_val(next_f, policy->min, policy->max);
		cid = arch_get_cluster_id(sg_policy->policy->cpu);
		next_f = mt_cpufreq_find_close_freq(cid, next_f);
#endif
		/*
		 * Do not reduce the frequency if the CPU has not been idle
		 * recently, as the reduction is likely to be premature then.
		 */
		if (busy && next_f < sg_policy->next_freq &&
		    sg_policy->next_freq != UINT_MAX) {
			next_f = sg_policy->next_freq;

			/* Reset cached freq as next_freq has changed */
			sg_policy->cached_raw_freq = 0;
		}
	}

	sugov_update_commit(sg_policy, time, next_f);
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;
	unsigned int next_f;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cid;
#endif

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
			if (idle_cpu(j))
				continue;
		}
		if (j_sg_cpu->flags & SCHED_CPUFREQ_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;

#ifdef CONFIG_UCLAMP_TASK
		trace_schedutil_uclamp_util(j, j_util);
#endif

		j_util = uclamp_util(cpu_rq(j), j_util);

		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		sugov_iowait_boost(j_sg_cpu, &util, &max);
	}

	next_f = get_next_freq(sg_policy, util, max, time);

#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	next_f = clamp_val(next_f, policy->min, policy->max);
	cid = arch_get_cluster_id(sg_policy->policy->cpu);
	next_f = mt_cpufreq_find_close_freq(cid, next_f);
#endif
	return next_f;
}

static void sugov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;

	sugov_get_util(&util, &max, sg_cpu->cpu);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (sugov_should_update_freq(sg_policy, time)) {
		if (flags & SCHED_CPUFREQ_DL)
			next_f = sg_policy->policy->cpuinfo.max_freq;
		else
			next_f = sugov_next_freq_shared(sg_cpu, time);

		sugov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);

	sg_policy->work_in_progress = false;
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	/*
	 * For RT and deadline tasks, the schedutil governor shoots the
	 * frequency to maximum. Special care must be taken to ensure that this
	 * kthread doesn't result in the same behavior.
	 *
	 * This is (mostly) guaranteed by the work_in_progress flag. The flag is
	 * updated only at the end of the sugov_work() function and before that
	 * the schedutil governor rejects all other frequency scaling requests.
	 *
	 * There is a very rare case though, where the RT thread yields right
	 * after the work_in_progress flag is cleared. The effects of that are
	 * neglected for now.
	 */
	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

int schedhorizon_set_down_rate_limit_us(int cpu, unsigned int rate_limit_us)
{
	struct cpufreq_policy *policy;
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	struct gov_attr_set *attr_set;

	policy = cpufreq_cpu_get(cpu);
	if (!policy)
		return -EINVAL;

	if (policy->governor != &schedhorizon_gov) {
		cpufreq_cpu_put(policy);
		return -ENOENT;
	}

	mutex_lock(&global_tunables_lock);
	sg_policy = policy->governor_data;
	if (!sg_policy) {
		mutex_unlock(&global_tunables_lock);
		cpufreq_cpu_put(policy);
		return -EINVAL;
	}

	tunables = sg_policy->tunables;
	attr_set = &tunables->attr_set;

	mutex_lock(&attr_set->update_lock);
	tunables->down_rate_limit_us = rate_limit_us;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	mutex_unlock(&attr_set->update_lock);
	mutex_unlock(&global_tunables_lock);

	cpufreq_cpu_put(policy);
	return 0;
}
EXPORT_SYMBOL(schedhorizon_set_down_rate_limit_us);

int schedhorizon_set_up_rate_limit_us(int cpu, unsigned int rate_limit_us)
{
	struct cpufreq_policy *policy;
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	struct gov_attr_set *attr_set;

	policy = cpufreq_cpu_get(cpu);
	if (!policy)
		return -EINVAL;

	if (policy->governor != &schedhorizon_gov) {
		cpufreq_cpu_put(policy);
		return -ENOENT;
	}

	mutex_lock(&global_tunables_lock);
	sg_policy = policy->governor_data;
	if (!sg_policy) {
		mutex_unlock(&global_tunables_lock);
		cpufreq_cpu_put(policy);
		return -EINVAL;
	}

	tunables = sg_policy->tunables;
	attr_set = &tunables->attr_set;

	mutex_lock(&attr_set->update_lock);
	tunables->up_rate_limit_us = rate_limit_us;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}
	mutex_unlock(&attr_set->update_lock);
	mutex_unlock(&global_tunables_lock);

	cpufreq_cpu_put(policy);
	return 0;
}
EXPORT_SYMBOL(schedhorizon_set_up_rate_limit_us);

static int sugov_count_tokens(const char *buf)
{
	const char *p = buf;
	int ntok = 0;

	while (*p) {
		while (*p == ' ' || *p == '\t' || *p == '\n')
			p++;
		if (!*p)
			break;
		ntok++;
		while (*p && *p != ' ' && *p != '\t' && *p != '\n')
			p++;
	}

	return ntok;
}

static ssize_t efficient_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	const struct sugov_efficient_freq *ef;
	ssize_t len = 0;
	int i;

	rcu_read_lock_sched();
	ef = rcu_dereference_sched(tunables->efficient_freq);
	for (i = 0; ef && i < ef->n; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u ",
				 ef->freq[i]);
	if (!len)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0 ");
	rcu_read_unlock_sched();

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

static ssize_t efficient_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_efficient_freq *new_ef, *old_ef;
	struct sugov_policy *sg_policy;
	char *tok, *tmp, *str;
	int ntok, i = 0;

	ntok = sugov_count_tokens(buf);
	if (!ntok)
		return -EINVAL;

	new_ef = kzalloc(struct_size(new_ef, freq, ntok), GFP_KERNEL);
	if (!new_ef)
		return -ENOMEM;

	str = kstrdup(buf, GFP_KERNEL);
	if (!str) {
		kfree(new_ef);
		return -ENOMEM;
	}

	tmp = str;
	while ((tok = strsep(&tmp, " \t\n")) != NULL) {
		if (*tok == '\0')
			continue;
		if (i >= ntok)
			break;
		if (kstrtouint(tok, 10, &new_ef->freq[i]) ||
		    (i && new_ef->freq[i] < new_ef->freq[i - 1])) {
			kfree(str);
			kfree(new_ef);
			return -EINVAL;
		}
		i++;
	}
	kfree(str);

	new_ef->n = i;
	if (!new_ef->n) {
		kfree(new_ef);
		return -EINVAL;
	}

	old_ef = rcu_dereference_protected(tunables->efficient_freq,
					   lockdep_is_held(&attr_set->update_lock));
	rcu_assign_pointer(tunables->efficient_freq, new_ef);

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->current_step = 0;
		sg_policy->first_hp_request_time = 0;
	}

	if (old_ef) {
		synchronize_sched();
		kfree(old_ef);
	}

	return count;
}

static ssize_t up_delay_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	const struct sugov_up_delay *ud;
	ssize_t len = 0;
	int i;

	rcu_read_lock_sched();
	ud = rcu_dereference_sched(tunables->up_delay);
	for (i = 0; ud && i < ud->n; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%llu ",
				 ud->delay[i] / NSEC_PER_MSEC);
	if (!len)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0 ");
	rcu_read_unlock_sched();

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

static ssize_t up_delay_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_up_delay *new_ud, *old_ud;
	struct sugov_policy *sg_policy;
	char *tok, *tmp, *str;
	unsigned int val_ms;
	int ntok, i = 0;

	ntok = sugov_count_tokens(buf);
	if (!ntok)
		return -EINVAL;

	new_ud = kzalloc(struct_size(new_ud, delay, ntok), GFP_KERNEL);
	if (!new_ud)
		return -ENOMEM;

	str = kstrdup(buf, GFP_KERNEL);
	if (!str) {
		kfree(new_ud);
		return -ENOMEM;
	}

	tmp = str;
	while ((tok = strsep(&tmp, " \t\n")) != NULL) {
		if (*tok == '\0')
			continue;
		if (i >= ntok)
			break;
		if (kstrtouint(tok, 10, &val_ms)) {
			kfree(str);
			kfree(new_ud);
			return -EINVAL;
		}
		/* Stored as ns internally, entered as ms via sysfs */
		new_ud->delay[i] = (u64)val_ms * NSEC_PER_MSEC;
		i++;
	}
	kfree(str);

	new_ud->n = i;
	if (!new_ud->n) {
		kfree(new_ud);
		return -EINVAL;
	}

	old_ud = rcu_dereference_protected(tunables->up_delay,
					   lockdep_is_held(&attr_set->update_lock));
	rcu_assign_pointer(tunables->up_delay, new_ud);

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->current_step = 0;
		sg_policy->first_hp_request_time = 0;
	}

	if (old_ud) {
		synchronize_sched();
		kfree(old_ud);
	}

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr efficient_freq = __ATTR_RW(efficient_freq);
static struct governor_attr up_delay = __ATTR_RW(up_delay);

static struct attribute *sugov_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&efficient_freq.attr,
	&up_delay.attr,
	NULL
};

static struct attribute_group sugov_attr_group = {
	.attrs = sugov_attrs,
};

static const struct attribute_group *sugov_groups[] = {
	&sugov_attr_group,
	NULL
};

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set =
		container_of(kobj, struct gov_attr_set, kobj);
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	kfree(rcu_dereference_protected(tunables->efficient_freq, true));
	kfree(rcu_dereference_protected(tunables->up_delay, true));
	kfree(tunables);
}

static struct kobj_type sugov_tunables_ktype = {
	.default_groups = sugov_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	sg_policy->first_hp_request_time = 0;
	raw_spin_lock_init(&sg_policy->update_lock);
	sg_policy->current_step = 0;
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;

	/* Kthread is bound to all CPUs by default */
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;
	struct sugov_efficient_freq *ef;
	struct sugov_up_delay *ud;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (!tunables)
		return NULL;

	ef = kzalloc(struct_size(ef, freq, 1), GFP_KERNEL);
	ud = kzalloc(struct_size(ud, delay, 1), GFP_KERNEL);
	if (!ef || !ud) {
		kfree(ef);
		kfree(ud);
		kfree(tunables);
		return NULL;
	}

	ef->n = 1;
	ud->n = 1;
	rcu_assign_pointer(tunables->efficient_freq, ef);
	rcu_assign_pointer(tunables->up_delay, ud);

	gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);

	return tunables;
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	mutex_lock(&global_tunables_lock);

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->up_rate_limit_us = 500;
	tunables->down_rate_limit_us = 4000;

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   &policy->kobj, "%s",
				   schedhorizon_gov.name);
	if (ret)
		goto fail;

	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;

stop_kthread:
	sugov_kthread_stop(sg_policy);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	mutex_unlock(&global_tunables_lock);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;

	mutex_lock(&global_tunables_lock);

	gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(sg_policy);
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = UINT_MAX;
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;
	sg_policy->cached_raw_freq = 0;
	sg_policy->current_step = 0;
	sg_policy->first_hp_request_time = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu = cpu;
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->flags = SCHED_CPUFREQ_DL;
		sg_cpu->iowait_boost_max = capacity_orig_of(cpu);
		sg_cpu->min_boost =
			(SCHED_CAPACITY_SCALE * policy->cpuinfo.min_freq) /
			policy->cpuinfo.max_freq;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->need_freq_update = true;
}

static struct cpufreq_governor schedhorizon_gov = {
	.name = "schedhorizon",
	.owner = THIS_MODULE,
	.dynamic_switching = true,
	.init = sugov_init,
	.exit = sugov_exit,
	.start = sugov_start,
	.stop = sugov_stop,
	.limits = sugov_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDHORIZON
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &schedhorizon_gov;
}
#endif

static int __init sugov_register(void)
{
	return cpufreq_register_governor(&schedhorizon_gov);
}
fs_initcall(sugov_register);
