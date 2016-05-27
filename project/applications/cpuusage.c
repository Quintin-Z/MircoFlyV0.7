/*********************************************************************************

*
* COPYRIGHT (C) 2012 - 2013
*
*
* For       	: 
* Mode      	: 
* Description   : CPU使用率计算
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include <rtthread.h>
#include <rthw.h>
#include "cpuusage.h"


static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;

static void cpu_usage_idle_hook()
{
	rt_tick_t tick;
	rt_uint32_t count;
	volatile rt_uint32_t loop;

	if (total_count == 0)
	{
		/* get total count */
		rt_enter_critical();
		tick = rt_tick_get();
		while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
		{
			total_count ++;
			loop = 0;
			while (loop < CPU_USAGE_LOOP) loop ++;
		}
		rt_exit_critical();
	}

	count = 0;
	/* get CPU usage */
	tick = rt_tick_get();
	while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
	{
		count ++;
		loop  = 0;
		while (loop < CPU_USAGE_LOOP) loop ++;
	}

	/* calculate major and minor */
	if (count < total_count)
	{
		count = total_count - count;
		cpu_usage_major = (count * 100) / total_count;
		cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
	}
	else
	{
		total_count = count;

		/* no CPU usage */
		cpu_usage_major = 0;
		cpu_usage_minor = 0;
	}
}

void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor)
{
	RT_ASSERT(major != RT_NULL);
	RT_ASSERT(minor != RT_NULL);

	*major = cpu_usage_major;
	*minor = cpu_usage_minor;
}

rt_uint8_t cpu_usage_get_in_percentage()
{
	rt_uint8_t persentage = 0;
	rt_uint8_t persentage2 = 0;
	cpu_usage_get(&persentage, &persentage2);
	return persentage;
}

int cpuUsage()
{
	return cpu_usage_get_in_percentage();
}
#ifdef RT_USING_FINSH
#include <finsh.h>
/* 输出CPUUsage函数到finsh shell中 之前函数名太长无法使用 不得已这样处理 */
FINSH_FUNCTION_EXPORT(cpuUsage, Get the cpu usage info in percentage);
#endif

void cpu_usage_init()
{
	/* set idle thread hook */
	rt_thread_idle_sethook(cpu_usage_idle_hook);
}
