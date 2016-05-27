/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2014, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-18     ArdaFu       the first version for TM4C129X
 */

#include <rtthread.h>
#include <board.h>
#include <components.h>
#include "stabilizer_task.h"
#include "cpuusage.h"
#include "rtt_nrf24l01.h"
#ifdef RT_USING_LWIP

#endif

extern void led_thread_creat(void);
extern void imu_thread_creat(void);
extern void  transfer_thread_creat(void);
extern void rt_hw_sdcard_init(void);
extern void test_thread_creat(void);
extern void imu_timeout_read(void* parameter);
extern void com_thread_creat(void);
extern void led_thread_creat(void);
//rt_timer_t timer_imu;
//struct rt_semaphore static_sem_led;


/* thread phase init */
void rt_init_thread_entry(void *parameter)
{
    /* Initialization RT-Thread Components */
    rt_components_init();
	
	//初始化文件系统
#ifdef RT_USING_DFS_ELMFAT
		/* mount sd card fat partition 1 as root directory */
		if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
		{
			rt_kprintf("File System initialized!\n");
		}
		else
		{
			rt_kprintf("File System initialzation failed!\n");
		}
#endif
		
#ifdef RT_USING_LWIP
		
#endif

	//rt_thread_delay(1000);
	//rt_enter_critical();	
	nrf24_init("nrf24","ssi00");
	//rt_exit_critical();

	com_thread_creat();
	
	
		cpu_usage_init();
	//transfer_thread_creat();
	stabilizer_thread_creat();	
	
	//test_thread_creat();
	led_thread_creat();
		
}

int rt_application_init(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("init",
                           rt_init_thread_entry, RT_NULL,
                           2048, 15, 20);
    if (tid != RT_NULL) rt_thread_startup(tid);
	

    return 0;
}
