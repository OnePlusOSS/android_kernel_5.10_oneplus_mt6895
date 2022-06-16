// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 */
#include <linux/pm_qos.h>
#include <trace/hooks/power.h>
#include <linux/kallsyms.h>
#include <linux/hashtable.h>

#include <perf_tracker_internal.h>
#include <perf_tracker_trace.h>
#ifdef CONFIG_OPLUS_FEATURE_FREQ_LIMIT_DEBUG
#include <trace/events/power.h>
#endif

struct h_node {
	unsigned long addr;
	char symbol[KSYM_SYMBOL_LEN];
	struct hlist_node node;
};

static DECLARE_HASHTABLE(tbl, 5);
static int is_inited;
static int is_hooked;
static struct freq_constraints *qos_in_cluster[MAX_CLUSTER_NR] = {0};

static const char *find_and_get_symobls(unsigned long caller_addr)
{
	struct h_node *cur_node = NULL;
	struct h_node *new_node = NULL;
	const char *cur_symbol = NULL;
	unsigned int cur_key = 0;

	cur_key = (unsigned int) caller_addr & 0x1f;
	// Try to find symbols from history records
	hash_for_each_possible(tbl, cur_node, node, cur_key) {
		if (cur_node->addr == caller_addr) {
			cur_symbol = cur_node->symbol;
			break;
		}
	}
	// Symbols not found. Add new records
	if (!cur_symbol) {
		new_node = kzalloc(sizeof(struct h_node), GFP_KERNEL);
		if (!new_node)
			return NULL;
		new_node->addr = caller_addr;
		sprint_symbol(new_node->symbol, caller_addr);
		cur_symbol = new_node->symbol;
		hash_add(tbl, &new_node->node, cur_key);
	}
	return cur_symbol;
}

static inline int find_qos_in_cluster(struct freq_constraints *qos)
{
	int cid = 0;

	for (cid = 0; cid < cluster_nr; cid++) {
		if (qos_in_cluster[cid] == qos)
			break;
	}
	return (cid < cluster_nr) ? cid : -1;
}

static void mtk_freq_qos_add_request(void *data, struct freq_constraints *qos,
	struct freq_qos_request *req, enum freq_qos_req_type type, int value, int ret)
{
	int cid = 0;
	const char *caller_info = find_and_get_symobls(
		(unsigned long)__builtin_return_address(1));
	if (caller_info) {
		cid = find_qos_in_cluster(qos);
		trace_freq_qos_user_setting(cid, type, value, caller_info);
	}
}

static void mtk_freq_qos_update_request(void *data, struct freq_qos_request *req, int value)
{
	int cid = 0;
	const char *caller_info = find_and_get_symobls(
		(unsigned long)__builtin_return_address(1));
	if (caller_info) {
		cid = find_qos_in_cluster(req->qos);
		trace_freq_qos_user_setting(cid, req->type, value, caller_info);
	}
}

int insert_freq_qos_hook(void)
{
	int ret = 0;

	if (is_hooked || !is_inited)
		return ret;
	ret = register_trace_android_vh_freq_qos_add_request(mtk_freq_qos_add_request, NULL);
	if (ret) {
		pr_info("mtk_freq_qos_add_requests: register hooks failed, returned %d\n", ret);
		goto register_failed;
	}
	ret = register_trace_android_vh_freq_qos_update_request(mtk_freq_qos_update_request, NULL);
	if (ret) {
		pr_info("mtk_freq_qos_update_requests: register hooks failed, returned %d\n", ret);
		goto register_failed;
	}
	is_hooked = 1;
	return ret;

register_failed:
	remove_freq_qos_hook();
	return ret;
}

void remove_freq_qos_hook(void)
{
	is_hooked = 0;
	unregister_trace_android_vh_freq_qos_add_request(mtk_freq_qos_add_request, NULL);
	unregister_trace_android_vh_freq_qos_update_request(mtk_freq_qos_update_request, NULL);
}


#ifdef CONFIG_OPLUS_FEATURE_FREQ_LIMIT_DEBUG

#define LCN 3 //Limit cluster number
#define LSN 5 //Limit statis number

struct limit_info {
	char comm[TASK_COMM_LEN];
	pid_t pid;
	pid_t tgid;
	int cid;
	int freq;
};

struct cluster_limit_info {
	int idx;
	int of; //overflow
	spinlock_t slock;
	struct limit_info info[LSN];
};

struct limit_task_desc {
	struct cluster_limit_info cinfo[LCN];
};

static struct limit_task_desc *g_ltd = NULL;

static int limit_info_stastic(struct task_struct *task, int cid, int freq)
{
	struct limit_info *info;
	struct cluster_limit_info *cinfo;

	if (cid >= LCN || cid < 0 || !g_ltd) {
		return -1;
	}
	cinfo = &g_ltd->cinfo[cid];

	spin_lock(&cinfo->slock);

	if (cinfo->idx >= LSN) {
		cinfo->idx = cinfo->idx % LSN;
		cinfo->of = 1;
	}
	info = &cinfo->info[cinfo->idx];

	memcpy(info->comm, task->comm, TASK_COMM_LEN);
	info->pid = task->pid;
	info->tgid = task->tgid;
	info->cid = cid;
	info->freq = freq;

	cinfo->idx++;

	spin_unlock(&cinfo->slock);

	return 0;
}


static void limit_info_clear(void)
{
	int i, j;
	struct cluster_limit_info *cinfo;

	if (!g_ltd)
		return;

	for (i = 0; i < LCN; i++) {
		cinfo = &g_ltd->cinfo[i];
		spin_lock(&cinfo->slock);
		cinfo->idx = 0;
		cinfo->of = 0;
		for (j = 0; j < LSN; j++) {
			memset(&cinfo->info[j], 0, sizeof(struct limit_info));
		}
		spin_unlock(&cinfo->slock);
	}
}

static inline void limit_info_print(struct limit_info *info)
{
	pr_info("MAX_FREQ_LIMIT_DUMP: pid=%d, comm=%s, tgid=%d, cid=%d, freq=%d\n",
		info->pid, info->comm, info->tgid, info->cid, info->freq);
}

static void limit_info_dump(void)
{
	int i, j;
	struct cluster_limit_info *cinfo;

	if (!g_ltd)
		return;

	for (i = 0; i < LCN; i++) {
		cinfo = &g_ltd->cinfo[i];
		spin_lock(&cinfo->slock);
		/* the old print firts */
		if (cinfo->of) {
			for (j = cinfo->idx; j < LSN; j++) {
				limit_info_print(&cinfo->info[j]);
			}
		}
		for (j = 0; j < cinfo->idx; j++) {
			limit_info_print(&cinfo->info[j]);
		}
		spin_unlock(&cinfo->slock);
	}

}

static void freq_qos_update_debug(void *data, struct freq_qos_request *req, int value)
{
	static int need_dump = 0;
	int cid = find_qos_in_cluster(req->qos);
	struct task_struct *cur = current;

	if (req->type == FREQ_QOS_MAX && value < 1000000) {
		/* it can be removed in official version */
		pr_info("MAX_FREQ_LIMIT: cur_pid=%d, cur_comm=%s, cur_tgid=%d, cid=%d, freq=%d\n",
				cur->pid, cur->comm, cur->tgid, cid, value);
		limit_info_stastic(cur, cid, value);
	}

	/* uise it to determine if trace is being captured, it shouldn't be hooked */
	if (trace_cpu_idle_enabled()) {
		if (need_dump == 0) {
			need_dump++;
			limit_info_dump();
			limit_info_clear();
		}
	} else {
		need_dump = 0;
	}
}

static int freq_qos_update_debug_init(void)
{
	int i;
	struct limit_task_desc *ltd = kzalloc(sizeof(struct limit_task_desc), GFP_KERNEL);
	if (!ltd) {
		pr_info("MAX_FREQ_LIMIT: kzalloc failed!\n");
		return -ENOMEM;
	}

	for (i = 0; i < LCN; i++) {
		spin_lock_init(&ltd->cinfo[i].slock);
	}

	g_ltd = ltd;

	register_trace_android_vh_freq_qos_update_request(freq_qos_update_debug, NULL);

	return 0;
}

static void freq_qos_update_debug_exit(void)
{
	if (!g_ltd)
		return;

	unregister_trace_android_vh_freq_qos_update_request(freq_qos_update_debug, NULL);

	kfree(g_ltd);
	g_ltd = NULL;
}

#endif /* CONFIG_OPLUS_FEATURE_FREQ_LIMIT_DEBUG */

static void init_cluster_qos_info(void)
{
	struct cpufreq_policy *policy;
	int cpu;
	int num = 0;

	for_each_possible_cpu(cpu) {
		if (num >= cluster_nr)
			break;
		policy = cpufreq_cpu_get(cpu);
		if (policy) {
			qos_in_cluster[num++] = &(policy->constraints);
			cpu = cpumask_last(policy->related_cpus);
			cpufreq_cpu_put(policy);
		}
	}
}

void init_perf_freq_tracker(void)
{
	is_hooked = 0;
	is_inited = 1;
	// Initialize hash table
	hash_init(tbl);
	init_cluster_qos_info();
#ifdef CONFIG_OPLUS_FEATURE_FREQ_LIMIT_DEBUG
	freq_qos_update_debug_init();
#endif
}

void exit_perf_freq_tracker(void)
{
	int bkt = 0;
	struct h_node *cur = NULL;
	struct hlist_node *tmp = NULL;

	is_inited = 0;

#ifdef CONFIG_OPLUS_FEATURE_FREQ_LIMIT_DEBUG
	freq_qos_update_debug_exit();
#endif
	remove_freq_qos_hook();
	// Remove hash table
	hash_for_each_safe(tbl, bkt, tmp, cur, node) {
		hash_del(&cur->node);
		kfree(cur);
	}
}
