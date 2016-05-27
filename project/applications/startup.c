/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2010-03-04     Magicoe      for LPC17xx
 * 2014-07-18     ArdaFu       Port to TM4C129X
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "battery.h"
#include "drv_eeprom.h"
#include "rtt_tm4c_spi.h"

void check_endian()
{
	union check
    {  int    i;
       char ch;
    }m;
    m.i=1;
    if(m.ch==0x00) rt_kprintf("System is Big-Endian!\n");
    if(m.ch==0x01) rt_kprintf("System is Little-Endian!\n");
}
extern int rt_application_init(void);
extern void rt_hw_sdcard_init(void);
extern void rt_hw_pwm_out_init(void);
extern void rt_hw_rf_init(void);
extern void rt_hw_led_init(void);
extern void rt_hw_battery_device_init(void);
/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{

    /* initialize board */
    rt_hw_board_init();
	
    /* show version */
    rt_show_version();
	
#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif
	
	check_endian();
	
	eerprom_init();
	
	rt_hw_spi_init();
	
	//rt_hw_rf_init();
	
	rt_hw_pwm_out_init();
	
	rt_hw_battery_device_init();
	
	rt_hw_led_init();
	
#ifdef RT_USING_DFS
	/* init sd card device */
	rt_hw_sdcard_init();
#endif
    /* initialize scheduler system */
    rt_system_scheduler_init();
	
    /* initialize system timer*/
    rt_system_timer_init();
	
    /* initialize application */
    rt_application_init();
	
    /* initialize timer thread */
    rt_system_timer_thread_init();

    /* initialize idle thread */
    rt_thread_idle_init();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return ;
}

int main(void)
{
    /* disable interrupt first */
    rt_hw_interrupt_disable();

    /* startup RT-Thread RTOS */
    rtthread_startup();

    return 0;
}
