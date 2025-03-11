/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 MediaTek Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM scheduler

#if !defined(_TRACE_SUGOV_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SUGOV_H
#include <linux/string.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#ifdef CONFIG_SCHEDUTIL_USE_TL
TRACE_EVENT(sugov_next_util_tl,
	    TP_PROTO(unsigned int cpu, unsigned long util, unsigned long max,
		     unsigned int target_util),
	    TP_ARGS(cpu, util, max, target_util),
	    TP_STRUCT__entry(
		    __field(unsigned int, cpu)
		    __field(unsigned long, util)
		    __field(unsigned long, max)
		    __field(unsigned int, target_util)
	    ),
	    TP_fast_assign(
		    __entry->cpu = cpu;
		    __entry->util = util;
		    __entry->max = max;
		    __entry->target_util = target_util;
	    ),
	    TP_printk("cpu=%u util=%lu max=%lu target_util=%u",
		      __entry->cpu,
		      __entry->util,
		      __entry->max,
		      __entry->target_util)
);

TRACE_EVENT(choose_util,
	    TP_PROTO(unsigned int util, unsigned int prevutil, unsigned int utilmax,
		     unsigned int utilmin, unsigned int tl),
	    TP_ARGS(util, prevutil, utilmax, utilmin, tl),
	    TP_STRUCT__entry(
		    __field(unsigned int, util)
		    __field(unsigned int, prevutil)
		    __field(unsigned int, utilmax)
		    __field(unsigned int, utilmin)
		    __field(unsigned int, tl)
	    ),
	    TP_fast_assign(
		    __entry->util = util;
		    __entry->prevutil = prevutil;
		    __entry->utilmax = utilmax;
		    __entry->utilmin = utilmin;
		    __entry->tl = tl;
	    ),
	    TP_printk("util=%u prevutil=%u utilmax=%u utilmin=%u tl=%u",
		      __entry->util,
		      __entry->prevutil,
		      __entry->utilmax,
		      __entry->utilmin,
		      __entry->tl)
);
#endif

TRACE_EVENT(sugov_ext_util,
	TP_PROTO(int cpu, unsigned long util,
		unsigned int min, unsigned int max),
	TP_ARGS(cpu, util, min, max),
	TP_STRUCT__entry(
		__field(int, cpu)
		__field(unsigned long, util)
		__field(unsigned int, min)
		__field(unsigned int, max)
	),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->util = util;
		__entry->min = min;
		__entry->max = max;
	),
	TP_printk(
		"cpu=%d util=%lu min=%u max=%u",
		__entry->cpu,
		__entry->util,
		__entry->min,
		__entry->max)
);
#endif /* _TRACE_SCHEDULER_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH sugov
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE sugov_trace
/* This part must be outside protection */
#include <trace/define_trace.h>
