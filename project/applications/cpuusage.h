#ifndef __CPUUSAGE_H__
#define __CPUUSAGE_H__
#include <rtthread.h>

#define CPU_USAGE_CALC_TICK	10
#define CPU_USAGE_LOOP		100

static void cpu_usage_idle_hook(void);
void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
rt_uint8_t cpu_usage_get_in_percentage(void);
void cpu_usage_init(void);

#endif //__CPUUSAGE_H__
