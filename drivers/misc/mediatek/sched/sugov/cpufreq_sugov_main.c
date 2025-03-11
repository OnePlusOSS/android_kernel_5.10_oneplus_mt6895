// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 */
/*
 *
 * Copyright (c) 2019 MediaTek Inc.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <sched/sched.h>
#include "cpufreq.h"
#include "common.h"
#include <linux/tick.h>
#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>
#include <trace/hooks/sched.h>
#include <trace/hooks/topology.h>
#include <linux/sched/cpufreq.h>
#include <linux/kthread.h>
#include <thermal_interface.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
#include <../kernel/oplus_cpu/sched/eas_opt/oplus_iowait.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
#include <../kernel/oplus_cpu/sched/eas_opt/oplus_cap.h>
#endif

#define CREATE_TRACE_POINTS
#include "sugov_trace.h"

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

static bool idle_util_enable = 1;
#ifdef CONFIG_SCHEDUTIL_USE_TL
/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 80
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
#endif

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	unsigned long		bw_dl;
	unsigned long		max;

	/* The field below is for single-CPU policies only: */
#if IS_ENABLED(CONFIG_NO_HZ_COMMON)
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-CPU data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-CPU
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * This is needed on the slow switching platforms too to prevent CPUs
	 * going offline from leaving stale IRQ work items behind.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
		return true;
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (sg_policy->flags & SCHED_CPUFREQ_DEF_FRAMEBOOST)
		return true;
#endif

	/* No need to recalculate next freq for min_rate_limit_us
	 * at least. However we might still decide to further rate
	 * limit once frequency change direction is decided, according
	 * to the separate rate limits.
	 */

	delta_ns = time - sg_policy->last_freq_update_time;
	return delta_ns >= READ_ONCE(sg_policy->min_rate_limit_ns);
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - sg_policy->last_freq_update_time;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (sg_policy->flags & SCHED_CPUFREQ_DEF_FRAMEBOOST)
		return false;
#endif

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (sg_policy->next_freq == next_freq)
		return false;

	if (sugov_up_down_rate_limit(sg_policy, time, next_freq))
		return false;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sugov_fast_switch(struct sugov_policy *sg_policy, u64 time,
			      unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;

	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	next_freq = cpufreq_driver_fast_switch(policy, next_freq);
	if (!next_freq)
		return;

	policy->cur = next_freq;

}

static void sugov_deferred_update(struct sugov_policy *sg_policy, u64 time,
				  unsigned int next_freq)
{
	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

#ifdef CONFIG_SCHEDUTIL_USE_TL
#ifdef CONFIG_NONLINEAR_FREQ_CTL
static inline unsigned int get_opp_capacity(struct cpufreq_policy *policy,
						int row)
{
	return (int)pd_get_opp_capacity(policy->cpu, row);
}
#else
static inline unsigned int get_opp_capacity(struct cpufreq_policy *policy,
						int row)
{
	unsigned int cap, orig_cap;
	unsigned long freq, max_freq;

	max_freq = policy->cpuinfo.max_freq;
	orig_cap = capacity_orig_of(policy->cpu);

	freq = policy->freq_table[row].frequency;
	cap = orig_cap * freq / max_freq;

	return cap;
}
#endif

static unsigned int util_to_targetload(
	struct sugov_tunables *tunables, unsigned int util)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads - 1 &&
		     util >= tunables->util_loads[i+1]; i += 2)
		;

	ret = tunables->util_loads[i];
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

unsigned int find_util_l(struct sugov_policy *sg_policy, unsigned int util)
{
	unsigned int capacity;
	int idx;

	for (idx = sg_policy->len-1; idx >= 0; idx--) {
	/*TODO: find the first bigger one in table, to match below orginal codes
	 *in cpufreq_schedutil_plus.c
	 *tbl = upower_get_core_tbl(cpu);
	 *for (idx = 0; idx < tbl->row_num ; idx++) {
	 *	cap = tbl->row[idx].cap;
	 *	if (!cap)
	 *		break;
	 *
	 *	target_idx = idx;
	 *
	 *	if (cap > util)
	 *		break;
	 *}
	*/
//		if (sg_policy->freq2util[idx].cap >= util)
		capacity = get_opp_capacity(sg_policy->policy, idx);
		if (capacity >= util){
			return capacity;
		}
	}
	return get_opp_capacity(sg_policy->policy, 0);
}

unsigned int find_util_h(struct sugov_policy *sg_policy, unsigned int util)
{
	unsigned int capacity;
	int idx;
	int target_idx = -1;

	for (idx = sg_policy->len-1; idx >= 0; idx--) {
		capacity = get_opp_capacity(sg_policy->policy, idx);
		if (capacity ==  util) {
			return util;
		}
		if (capacity < util) {
			target_idx = idx;
			continue;
		}
        if (target_idx == -1)
			return capacity;
		return get_opp_capacity(sg_policy->policy, target_idx);
	}
	return get_opp_capacity(sg_policy->policy, target_idx);
}

unsigned int find_closest_util(struct sugov_policy *sg_policy, unsigned int util
		, unsigned int policy)
{
	switch (policy) {
	case CPUFREQ_RELATION_L:
		return find_util_l(sg_policy, util);
	case CPUFREQ_RELATION_H:
		return find_util_h(sg_policy, util);
	default:
		return util;
	}
}

unsigned int choose_util(struct sugov_policy *sg_policy,
		unsigned int util)
{
	unsigned int prevutil, utilmin, utilmax;
	unsigned int tl;
	unsigned long orig_util = util;

	if (!sg_policy) {
		pr_err("sg_policy is null\n");
		return -EINVAL;
	}

	utilmin = 0;
	utilmax = UINT_MAX;

	do {
		prevutil = util;
		tl = util_to_targetload(sg_policy->tunables, util);
		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		util = find_closest_util(sg_policy, (orig_util * 100 / tl), CPUFREQ_RELATION_L);
		trace_choose_util(util, prevutil, utilmax, utilmin, tl);
		if (util > prevutil) {
			/* The previous frequency is too low. */
			utilmin = prevutil;

			if (util >= utilmax) {
				/*
				 * Find the highest frequency that is less
				 * than freqmax.
				 */
				util = find_closest_util(sg_policy, utilmax - 1,CPUFREQ_RELATION_H);
				if (util == utilmin) {
					/*
					 * The first frequency below freqmax
					 * has already been found to be too
					 * low.  freqmax is the lowest speed
					 * we found that is fast enough.
					 */
					util = utilmax;
					break;
				}
			}
		} else if (util < prevutil) {
			/* The previous frequency is high enough. */
			utilmax = prevutil;

			if (util <= utilmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				util = find_closest_util(sg_policy, utilmin + 1, CPUFREQ_RELATION_L);
				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				if (util == utilmax)
					break;
			}
		}

		/* If same frequency chosen as previous then done. */
	} while (util != prevutil);

	return util;
}
EXPORT_SYMBOL_GPL(choose_util);
#endif

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
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
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = policy->cpuinfo.max_freq;
	unsigned long next_freq = 0;

	mtk_map_util_freq((void *)sg_policy, util, freq, policy->related_cpus, &next_freq);
	if (next_freq) {
		freq = next_freq;
	} else {
		freq = map_util_freq(util, freq, max);
		if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
			return sg_policy->next_freq;
		sg_policy->cached_raw_freq = freq;
		freq = cpufreq_driver_resolve_freq(policy, freq);
	}

	return freq;
}

/*
 * This function computes an effective utilization for the given CPU, to be
 * used for frequency selection given the linear relation: f = u * f_max.
 *
 * The scheduler tracks the following metrics:
 *
 *   cpu_util_{cfs,rt,dl,irq}()
 *   cpu_bw_dl()
 *
 * Where the cfs,rt and dl util numbers are tracked with the same metric and
 * synchronized windows and are thus directly comparable.
 *
 * The cfs,rt,dl utilization are the running times measured with rq->clock_task
 * which excludes things like IRQ and steal-time. These latter are then accrued
 * in the irq utilization.
 *
 * The DL bandwidth number otoh is not a measured metric but a value computed
 * based on the task model parameters and gives the minimal utilization
 * required to meet deadlines.
 */
unsigned long mtk_cpu_util(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p)
{
	unsigned long dl_util, util, irq;
	struct rq *rq = cpu_rq(cpu);



	if (!uclamp_is_used() &&
	    type == FREQUENCY_UTIL && rt_rq_is_runnable(&rq->rt)) {
		return max;
	}

	/*
	 * Early check to see if IRQ/steal time saturates the CPU, can be
	 * because of inaccuracies in how we track these -- see
	 * update_irq_load_avg().
	 */
	irq = cpu_util_irq(rq);
	if (unlikely(irq >= max))
		return max;

	/*
	 * Because the time spend on RT/DL tasks is visible as 'lost' time to
	 * CFS tasks and we use the same metric to track the effective
	 * utilization (PELT windows are synchronized) we can directly add them
	 * to obtain the CPU's actual utilization.
	 *
	 * CFS and RT utilization can be boosted or capped, depending on
	 * utilization clamp constraints requested by currently RUNNABLE
	 * tasks.
	 * When there are no CFS RUNNABLE tasks, clamps are released and
	 * frequency will be gracefully reduced with the utilization decay.
	 */
	util = util_cfs + cpu_util_rt(rq);

	if (type == FREQUENCY_UTIL)
		util = mtk_uclamp_rq_util_with(rq, util, p);

	dl_util = cpu_util_dl(rq);

	/*
	 * For frequency selection we do not make cpu_util_dl() a permanent part
	 * of this sum because we want to use cpu_bw_dl() later on, but we need
	 * to check if the CFS+RT+DL sum is saturated (ie. no idle time) such
	 * that we select f_max when there is no idle time.
	 *
	 * NOTE: numerical errors or stop class might cause us to not quite hit
	 * saturation when we should -- something for later.
	 */
	if (util + dl_util >= max)
		return max;

	/*
	 * OTOH, for energy computation we need the estimated running time, so
	 * include util_dl and ignore dl_bw.
	 */
	if (type == ENERGY_UTIL)
		util += dl_util;

	/*
	 * There is still idle time; further improve the number by using the
	 * irq metric. Because IRQ/steal time is hidden from the task clock we
	 * need to scale the task numbers:
	 *
	 *              max - irq
	 *   U' = irq + --------- * U
	 *                 max
	 */
	util = scale_irq_capacity(util, irq, max);
	util += irq;

	/*
	 * Bandwidth required by DEADLINE must always be granted while, for
	 * FAIR and RT, we use blocked utilization of IDLE CPUs as a mechanism
	 * to gracefully reduce the frequency when no tasks show up for longer
	 * periods of time.
	 *
	 * Ideally we would like to set bw_dl as min/guaranteed freq and util +
	 * bw_dl as requested freq. However, cpufreq is not yet ready for such
	 * an interface. So, we only do the latter for now.
	 */
	if (type == FREQUENCY_UTIL)
		util += cpu_bw_dl(rq);

	return min(max, util);
}
EXPORT_SYMBOL(mtk_cpu_util);

DEFINE_PER_CPU(int, cpufreq_idle_cpu);
DEFINE_PER_CPU(int, pre_rt_throttled);
EXPORT_SYMBOL(cpufreq_idle_cpu);

DEFINE_PER_CPU(spinlock_t, cpufreq_idle_cpu_lock) = __SPIN_LOCK_UNLOCKED(cpufreq_idle_cpu_lock);
EXPORT_SYMBOL(cpufreq_idle_cpu_lock);
static unsigned long sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);
	unsigned long util = cpu_util_cfs(rq);
	unsigned long max = capacity_orig_of(sg_cpu->cpu);
	int rt_throttled_toggle = 0;

	sg_cpu->max = max;
	sg_cpu->bw_dl = cpu_bw_dl(rq);
	if (idle_util_enable){
		spin_lock(&per_cpu(cpufreq_idle_cpu_lock, sg_cpu->cpu));
		if (per_cpu(pre_rt_throttled, sg_cpu->cpu) != rq->rt.rt_throttled) {
			rt_throttled_toggle = 1;
			per_cpu(cpufreq_idle_cpu, sg_cpu->cpu) = (rq->nr_running) ? 0 : 1;
		}
		per_cpu(pre_rt_throttled, sg_cpu->cpu) = rq->rt.rt_throttled;
		if ((!rt_throttled_toggle && per_cpu(cpufreq_idle_cpu, sg_cpu->cpu)) ||
			(rt_throttled_toggle &&  !rq->nr_running)) {
			spin_unlock(&per_cpu(cpufreq_idle_cpu_lock, sg_cpu->cpu));
			return 0;
		}
		spin_unlock(&per_cpu(cpufreq_idle_cpu_lock, sg_cpu->cpu));
	}

	return mtk_cpu_util(sg_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);
}

/**
 * sugov_iowait_reset() - Reset the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @set_iowait_boost: true if an IO boost has been requested
 *
 * The IO wait boost of a task is disabled after a tick since the last update
 * of a CPU. If a new IO wait boost is requested after more then a tick, then
 * we enable the boost starting from IOWAIT_BOOST_MIN, which improves energy
 * efficiency by ignoring sporadic wakeups from IO.
 */
static bool sugov_iowait_reset(struct sugov_cpu *sg_cpu, u64 time,
			       bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	unsigned int ticks = TICK_NSEC;
	if (sysctl_iowait_reset_ticks)
		ticks = sysctl_iowait_reset_ticks * TICK_NSEC;
	if (delta_ns <= ticks)
#else
	/* Reset boost only if a tick has elapsed since last request */
	if (delta_ns <= TICK_NSEC)
#endif
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}

/**
 * sugov_iowait_boost() - Updates the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @flags: SCHED_CPUFREQ_IOWAIT if the task is waking up after an IO wait
 *
 * Each time a task wakes up after an IO operation, the CPU utilization can be
 * boosted to a certain utilization which doubles at each "frequent and
 * successive" wakeup from IO, ranging from IOWAIT_BOOST_MIN to the utilization
 * of the maximum OPP.
 *
 * To keep doubling, an IO boost has to be requested at least once per tick,
 * otherwise we restart from the utilization of the minimum OPP.
 */
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
			       unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sg_cpu->iowait_boost &&
	    sugov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	/* Boost only tasks waking up after IO */
	if (!set_iowait_boost)
		return;

	/* Ensure boost doubles only one time at each request */
	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	/* Double the boost at each request */
	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		return;
	}

	/* First wakeup after IO: start with minimum boost */
	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

/**
 * sugov_iowait_apply() - Apply the IO boost to a CPU.
 * @sg_cpu: the sugov data for the cpu to boost
 * @time: the update time from the caller
 * @util: the utilization to (eventually) boost
 * @max: the maximum value the utilization can be boosted to
 *
 * A CPU running a task which woken up after an IO operation can have its
 * utilization boosted to speed up the completion of those IO operations.
 * The IO boost value is increased each time a task wakes up from IO, in
 * sugov_iowait_apply(), and it's instead decreased by this function,
 * each time an increase has not been requested (!iowait_boost_pending).
 *
 * A CPU which also appears to have been idle for at least one tick has also
 * its IO boost utilization reset.
 *
 * This mechanism is designed to boost high frequently IO waiting tasks, while
 * being more conservative on tasks which does sporadic IO operations.
 */
static unsigned long sugov_iowait_apply(struct sugov_cpu *sg_cpu, u64 time,
					unsigned long util, unsigned long max)
{
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
#endif
	unsigned long boost;

	/* No boost currently required */
	if (!sg_cpu->iowait_boost)
		return util;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sugov_iowait_reset(sg_cpu, time, false))
		return util;

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	if (!sg_cpu->iowait_boost_pending &&
			(!sysctl_iowait_apply_ticks ||
			(time - sg_policy->last_update >
			(sysctl_iowait_apply_ticks * TICK_NSEC)))) {
#else
	if (!sg_cpu->iowait_boost_pending) {
#endif
		/*
		 * No boost pending; reduce the boost value.
		 */
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return util;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	/*
	 * @util is already in capacity scale; convert iowait_boost
	 * into the same scale so we can compare.
	 */
	boost = (sg_cpu->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
	return max(boost, util);
}

#if IS_ENABLED(CONFIG_NO_HZ_COMMON)
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

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu, struct sugov_policy *sg_policy)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
		sg_policy->limits_changed = true;
}

#if IS_ENABLED(CONFIG_MTK_OPP_MIN)
void mtk_set_cpu_min_opp(int cpu, unsigned long min_util)
{
	int gear_id, min_opp, i;
	unsigned long scale_cpu, freq;
	struct em_perf_domain *pd;
	struct em_perf_state *ps;

	gear_id = topology_physical_package_id(cpu);

	if (min_util == 0) {
		set_cpu_min_opp(gear_id, -1);
		return;
	}

	pd = em_cpu_get(cpu);
	if (!pd)
		return;
	scale_cpu = arch_scale_cpu_capacity(cpu);
	ps = &pd->table[pd->nr_perf_states - 1];
	freq = map_util_freq(min_util, ps->frequency, scale_cpu);

	/*
	 * Find the lowest performance state of the Energy Model
	 * above the
	 * requested frequency.
	 */
	for (i = 0; i < pd->nr_perf_states; i++) {
		ps = &pd->table[i];
		if (ps->frequency >= freq)
			break;
	}

	i = min(i, pd->nr_perf_states - 1);
	min_opp = pd->nr_perf_states - i - 1;
	set_cpu_min_opp(gear_id, min_opp);
}

void mtk_set_cpu_min_opp_single(struct sugov_cpu *sg_cpu)
{
	int cpu = sg_cpu->cpu;
	struct rq *rq = cpu_rq(cpu);
	unsigned long min_util;

	min_util = READ_ONCE(rq->uclamp[UCLAMP_MIN].value);
	mtk_set_cpu_min_opp(cpu, min_util);
}

void mtk_set_cpu_min_opp_shared(struct sugov_cpu *sg_cpu)
{
	int cpu, i;
	unsigned long min_util = 0, util;
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;

	for_each_cpu(i, policy->cpus) {
		struct rq *rq = cpu_rq(i);

		util = READ_ONCE(rq->uclamp[UCLAMP_MIN].value);
		min_util = max(min_util, util);
	}


	cpu = cpumask_first(policy->cpus);
	mtk_set_cpu_min_opp(cpu, min_util);
}
#else

void mtk_set_cpu_min_opp_single(struct sugov_cpu *sg_cpu)
{
}

void mtk_set_cpu_min_opp_shared(struct sugov_cpu *sg_cpu)
{
}

#endif

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct rq *rq;
	unsigned long umin, umax;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	unsigned long irq_flags;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util_thresh = 0;
	unsigned int avg_nr_running = 1;
	int cluster_id = topology_physical_package_id(cpumask_first(policy->cpus));
	unsigned long util_orig;
#endif
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	unsigned long util_bak;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_lock_irqsave(&sg_policy->update_lock, irq_flags);
#else
	raw_spin_lock(&sg_policy->update_lock);
#endif

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	sg_policy->flags = flags;
#endif

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (!sugov_should_update_freq(sg_policy, time)) {
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, irq_flags);
#else
		raw_spin_unlock(&sg_policy->update_lock);
#endif
		return;
	}

	/* Critical Task aware thermal throttling, notify thermal */
	mtk_set_cpu_min_opp_single(sg_cpu);

	/* Limits may have changed, don't skip frequency update */
	busy = !sg_policy->need_freq_update && sugov_cpu_is_busy(sg_cpu);

	util = sugov_get_util(sg_cpu);
	max = sg_cpu->max;
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	util_bak = util;
	util = sugov_iowait_apply(sg_cpu, time, util, max);
	if (unlikely(eas_opt_debug_enable))
		trace_printk("[eas_opt]: enable_iowait_boost=%d, cpu:%d, max:%d, cpu->util:%d,iowait_util:%d\n",
				sysctl_oplus_iowait_boost_enabled, sg_cpu->cpu, max, util_bak, util);
#else
	util = sugov_iowait_apply(sg_cpu, time, util, max);
#endif

	if (trace_sugov_ext_util_enabled()) {
		rq = cpu_rq(sg_cpu->cpu);

		umin = rq->uclamp[UCLAMP_MIN].value;
		umax = rq->uclamp[UCLAMP_MAX].value;
		trace_sugov_ext_util(sg_cpu->cpu, util, umin, umax);
	}
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_freq_policy_util(sg_cpu->sg_policy->flags, sg_policy->policy->cpus, &util);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	if (eas_opt_enable && (util_thresh_percent[cluster_id] != 100)) {
		rq = cpu_rq(sg_cpu->cpu);
		util_thresh = max * util_thresh_cvt[cluster_id] >> SCHED_CAPACITY_SHIFT;
		avg_nr_running = rq->nr_running;
		util_orig = util;
		util = (util_thresh < util) ?
			(util_thresh + ((avg_nr_running * (util - util_thresh) * nr_oplus_cap_multiple[cluster_id]) >> SCHED_CAPACITY_SHIFT)) : util;
		if (unlikely(eas_opt_debug_enable))
			trace_printk("[eas_opt]: cluster_id: %d, capacity: %d, util_thresh: %d, util_orig: %d, "
					"util: %d, avg_nr_running: %d, oplus_cap_multiple: %d,nr_oplus_cap_multiple: %d, util_thresh: %d\n",
					cluster_id, max, util_thresh, util_orig, util, avg_nr_running,
					oplus_cap_multiple[cluster_id], nr_oplus_cap_multiple[cluster_id], util_thresh_percent[cluster_id]);
	}
#endif

	next_f = get_next_freq(sg_policy, util, max);
	/*
	 * Do not reduce the frequency if the CPU has not been idle
	 * recently, as the reduction is likely to be premature then.
	 */
	if (busy && next_f < sg_policy->next_freq) {
		next_f = sg_policy->next_freq;

		/* Reset cached freq as next_freq has changed */
		sg_policy->cached_raw_freq = 0;
	}

	/*
	 * This code runs under rq->lock for the target CPU, so it won't run
	 * concurrently on two different CPUs for the same target and it is not
	 * necessary to acquire the lock in the fast switch case.
	 */
	if (sg_policy->policy->fast_switch_enabled) {
		sugov_fast_switch(sg_policy, time, next_f);
	} else {
		sugov_deferred_update(sg_policy, time, next_f);
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, irq_flags);
#else
	raw_spin_unlock(&sg_policy->update_lock);
#endif
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	struct rq *rq;
	unsigned long umin, umax;
	unsigned long util = 0, max = 1;
	unsigned int j;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	unsigned long util_thresh = 0;
	unsigned long util_orig = 0;
	unsigned int avg_nr_running = 1;
	unsigned int count_cpu = 0;
	int cluster_id = topology_physical_package_id(cpumask_first(policy->cpus));
#endif
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	unsigned long util_bak;
#endif

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
		rq = cpu_rq(j);
		avg_nr_running += rq->nr_running;
		count_cpu ++;
#endif

		j_util = sugov_get_util(j_sg_cpu);
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
		util_bak = j_util;
#endif
		j_max = j_sg_cpu->max;
		j_util = sugov_iowait_apply(j_sg_cpu, time, j_util, j_max);
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
		if (unlikely(eas_opt_debug_enable))
			trace_printk("[eas_opt]: enable_iowait_boost=%d, cpu:%d, max:%d, cpu->util:%d, iowait_util:%d\n",
					sysctl_oplus_iowait_boost_enabled, j_sg_cpu->cpu, j_sg_cpu->max, util_bak, j_util);
#endif

		if (trace_sugov_ext_util_enabled()) {
			rq = cpu_rq(j);

			umin = rq->uclamp[UCLAMP_MIN].value;
			umax = rq->uclamp[UCLAMP_MAX].value;
			trace_sugov_ext_util(j, j_util, umin, umax);
		}

		if (j_util * max >= j_max * util) {
			util = j_util;
			max = j_max;
		}
	}
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_freq_policy_util(sg_policy->flags, policy->cpus, &util);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_VT_CAP)
	if (eas_opt_enable && (util_thresh_percent[cluster_id] != 100) && count_cpu) {
		util_thresh = max * util_thresh_cvt[cluster_id] >> SCHED_CAPACITY_SHIFT;
		avg_nr_running = mult_frac(avg_nr_running, 1, count_cpu);
		util_orig = util;
		util = (util_thresh < util) ?
			(util_thresh + ((avg_nr_running * (util - util_thresh) * nr_oplus_cap_multiple[cluster_id]) >> SCHED_CAPACITY_SHIFT)) : util;
		if (unlikely(eas_opt_debug_enable))
			trace_printk("[eas_opt]: cluster_id: %d, capacity: %d, util_thresh: %d, util_orig: %d, util: %d, avg_nr_running: %d, "
			"oplus_cap_multiple: %d,nr_oplus_cap_multiple: %d, util_thresh: %d\n",
			cluster_id, max, util_thresh, util_orig, util, avg_nr_running,
			oplus_cap_multiple[cluster_id], nr_oplus_cap_multiple[cluster_id], util_thresh_percent[cluster_id]);
	}
#endif

	return get_next_freq(sg_policy, util, max);
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	unsigned long irq_flags;
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_lock_irqsave(&sg_policy->update_lock, irq_flags);
#else
	raw_spin_lock(&sg_policy->update_lock);
#endif

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	sg_policy->flags = flags;
#endif

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (sugov_should_update_freq(sg_policy, time)) {
		next_f = sugov_next_freq_shared(sg_cpu, time);
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
		sg_policy->last_update = time;
#endif

		if (sg_policy->policy->fast_switch_enabled)
			sugov_fast_switch(sg_policy, time, next_f);
		else
			sugov_deferred_update(sg_policy, time, next_f);
	}

	/* Critical Task aware thermal throttling, notify thermal */
	mtk_set_cpu_min_opp_shared(sg_cpu);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, irq_flags);
#else
	raw_spin_unlock(&sg_policy->update_lock);
#endif
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Hold sg_policy->update_lock shortly to handle the case where:
	 * incase sg_policy->next_freq is read here, and then updated by
	 * sugov_deferred_update() just before work_in_progress is set to false
	 * here, we may miss queueing the new update.
	 *
	 * Note: If a work was queued after the update_lock is released,
	 * sugov_work() will just be called again by kthread_work code; and the
	 * request will be proceed before the sugov thread sleeps.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

bool check_freq_update_for_time(struct update_util_data *hook, u64 time)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);

	return sg_cpu->last_update == time;
}
EXPORT_SYMBOL(check_freq_update_for_time);

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	WRITE_ONCE(sg_policy->min_rate_limit_ns, min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns));
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

static ssize_t
up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
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

static ssize_t
down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
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

#ifdef CONFIG_SCHEDUTIL_USE_TL
static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	for (i = 0; i < tunables->ntarget_loads; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret - 1, "%u%s", tunables->target_loads[i],
			i & 0x1 ? ":" : " ");

	snprintf(buf + ret - 1, PAGE_SIZE - ret - 1, "\n");
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;

	return tokenized_data;
err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static unsigned int freq2util(struct sugov_policy *sg_policy, unsigned int freq)
{
	int idx;
	unsigned int capacity, opp_freq;
#ifdef CONFIG_MTK_TINYSYS_SSPM_SUPPORT
	int cpu = sg_policy->policy->cpu;
	int cid = arch_get_cluster_id(cpu);

	freq = mt_cpufreq_find_close_freq(cid, freq);
#endif
	for (idx = sg_policy->len-1; idx >= 0; idx--) {
		capacity = get_opp_capacity(sg_policy->policy, idx);
		opp_freq = cpufreq_get_cpu_freq(sg_policy->policy->cpu, idx);
		if (freq <= opp_freq) {
			return capacity;
		}
	}
	return get_opp_capacity(sg_policy->policy, 0);
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set, const char *buf,
					size_t count)
{
	int ntokens, i;
	unsigned int *new_target_loads = NULL;
	unsigned long flags;
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int *new_util_loads = NULL;


	//get the first policy if this tunnables have mutil policies
	sg_policy = list_first_entry(&attr_set->policy_list, struct sugov_policy, tunables_hook);
	if (!sg_policy) {
		pr_err("sg_policy is null\n");
		return count;
	}

	new_target_loads = get_tokenized_data(buf, &ntokens);

	if (IS_ERR(new_target_loads))
		return PTR_ERR(new_target_loads);

	new_util_loads = kzalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!new_util_loads)
		return -ENOMEM;

	memcpy(new_util_loads, new_target_loads, sizeof(unsigned int) * ntokens);
	for (i = 0; i < ntokens - 1; i += 2) {
			new_util_loads[i+1] = freq2util(sg_policy, new_target_loads[i+1]);
	}

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);
	if (tunables->util_loads != default_target_loads)
		kfree(tunables->util_loads);

	tunables->target_loads = new_target_loads;
	tunables->ntarget_loads = ntokens;
	tunables->util_loads = new_util_loads;
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return count;
}

ssize_t set_sugov_tl(unsigned int cpu, char *buf)
{
	struct cpufreq_policy *policy;
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	struct gov_attr_set *attr_set;
	size_t count;

	if (!buf)
		return -EFAULT;

	policy = cpufreq_cpu_get(cpu);
	if (!policy)
		return -ENODEV;

	sg_policy = policy->governor_data;
	if (!sg_policy)
		return -EINVAL;

	tunables = sg_policy->tunables;
	if (!tunables)
		return -ENOMEM;

	attr_set = &tunables->attr_set;
	count = strlen(buf);

	return target_loads_store(attr_set, buf, count);
}
EXPORT_SYMBOL_GPL(set_sugov_tl);
#endif

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

#ifdef CONFIG_SCHEDUTIL_USE_TL
static struct governor_attr target_loads =
	__ATTR(target_loads, 0664, target_loads_show, target_loads_store);
#endif

static struct attribute *sugov_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
#ifdef CONFIG_SCHEDUTIL_USE_TL
	&target_loads.attr,
#endif
	NULL
};
ATTRIBUTE_GROUPS(sugov);

static struct kobj_type sugov_tunables_ktype = {
	.default_groups = sugov_groups,
	.sysfs_ops = &governor_sysfs_ops,
};

/********************** cpufreq governor interface *********************/

struct cpufreq_governor mtk_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
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
		pr_info("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_info("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
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

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_tunables_free(struct sugov_tunables *tunables)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;

	kfree(tunables);
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;
#ifdef CONFIG_SCHEDUTIL_USE_TL
	int cluster_id, first_cpu;
#endif

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

#ifdef CONFIG_SCHEDUTIL_USE_TL
	first_cpu = cpumask_first(policy->related_cpus);
	cluster_id = topology_physical_package_id(first_cpu);
	sg_policy->len = get_nr_caps(cluster_id);
#endif

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->up_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
	tunables->down_rate_limit_us = cpufreq_policy_transition_delay_us(policy);

#ifdef CONFIG_SCHEDUTIL_USE_TL
	tunables->target_loads = default_target_loads;
	tunables->ntarget_loads = ARRAY_SIZE(default_target_loads);
	//same with target_loads by default
	tunables->util_loads = default_target_loads;
	spin_lock_init(&tunables->target_loads_lock);
#endif

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   mtk_gov.name);
	if (ret)
		goto fail;

	policy->dvfs_possible_from_any_cpu = 1;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_tunables_free(tunables);

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_info("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_tunables_free(tunables);

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
	sg_policy->last_freq_update_time	= 0;
	sg_policy->next_freq			= 0;
	sg_policy->work_in_progress		= false;
	sg_policy->limits_changed		= false;
	sg_policy->need_freq_update		= false;
	sg_policy->cached_raw_freq		= 0;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	sg_policy->flags = 0;
#endif

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu			= cpu;
		sg_cpu->sg_policy		= sg_policy;
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

	synchronize_rcu();

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

	sg_policy->limits_changed = true;
}

struct cpufreq_governor mtk_gov = {
	.name			= "sugov_ext",
	.owner			= THIS_MODULE,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};

static int control_idle_cpus_util_open_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", idle_util_enable);
	return 0;
}

static int control_idle_cpus_util_open(struct inode *in, struct file *file)
{
	return single_open(file, control_idle_cpus_util_open_show, NULL);
}

static ssize_t control_idle_cpus_util_write(struct file *filp, const char *ubuf,
	size_t count, loff_t *data)
{
	char buf[16] = {0};
	int ret;
	unsigned int input = 0;

	if (!count)
		return count;
	if (count + 1 > 16)
		return -ENOMEM;
	ret = copy_from_user(buf, ubuf, count);
	if (ret)
		return -EFAULT;
	buf[count] = '\0';
	ret = kstrtouint(buf, 10, &input);
	if (ret)
		return -EFAULT;

	idle_util_enable = input;
 	return count;
}

static const struct proc_ops control_idle_util_enable = {
	.proc_open = control_idle_cpus_util_open,
	.proc_read = seq_read,
	.proc_write = control_idle_cpus_util_write
};

static int __init cpufreq_mtk_init(void)
{
	int ret = 0;
	struct proc_dir_entry *dir;

	dir = proc_mkdir("mtk_scheduler", NULL);
	if (!dir)
		return -ENOMEM;
	proc_create("control_idle_util_enable", 0644, dir, &control_idle_util_enable);
	ret = init_opp_cap_info(dir);
	if (ret)
		return ret;
#if IS_ENABLED(CONFIG_NONLINEAR_FREQ_CTL)
	ret = register_trace_android_vh_arch_set_freq_scale(
			mtk_arch_set_freq_scale, NULL);
	if (ret)
		pr_info("register android_vh_arch_set_freq_scale failed\n");
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
	update_ux_sched_cputopo();
#endif

	return cpufreq_register_governor(&mtk_gov);
}

static void __exit cpufreq_mtk_exit(void)
{
	clear_opp_cap_info();
	cpufreq_unregister_governor(&mtk_gov);
}

module_init(cpufreq_mtk_init);
module_exit(cpufreq_mtk_exit);

MODULE_LICENSE("GPL");
