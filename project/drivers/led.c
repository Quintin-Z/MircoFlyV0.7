/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-xx-xx     jiequan      the first version
 */
#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "led.h"
 void rt_hw_led_init(void)
{
	ROM_SysCtlPeripheralEnable(RED_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(GREEN_GPIO_PERIPH);
	ROM_SysCtlPeripheralEnable(BLUE_GPIO_PERIPH);	
	ROM_GPIOPinTypeGPIOOutput(RED_GPIO_BASE, RED_GPIO_PIN | BLUE_GPIO_PIN|GREEN_GPIO_PIN);

}
void rt_hw_led_off(rt_uint32_t n)
{
    switch (n)
    {
    case 0:
		ROM_GPIOPinWrite(RED_GPIO_BASE, RED_GPIO_PIN, RED_GPIO_PIN);
        break;
    case 1:
        ROM_GPIOPinWrite(BLUE_GPIO_BASE, BLUE_GPIO_PIN, BLUE_GPIO_PIN);
        break;
    case 2:
        ROM_GPIOPinWrite(GREEN_GPIO_BASE, GREEN_GPIO_PIN, GREEN_GPIO_PIN);
        break;
    default:
        break;
    }
}
void rt_hw_led_on(rt_uint32_t n)
{
    switch (n)
    {
    case 0:
		ROM_GPIOPinWrite(RED_GPIO_BASE, RED_GPIO_PIN, 0);
        break;
    case 1:
        ROM_GPIOPinWrite(BLUE_GPIO_BASE, BLUE_GPIO_PIN, 0);
        break;
    case 2:
        ROM_GPIOPinWrite(GREEN_GPIO_BASE, GREEN_GPIO_PIN, 0);
        break;
    default:
        break;
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
extern struct rt_semaphore static_sem_led;
static void led_thread_entry(void *parameter)
{
    unsigned int count = 0;

	rt_hw_led_init();
	rt_hw_led_off(0);
	rt_hw_led_off(1);
	rt_hw_led_off(2);

    while (1)
    {
		rt_sem_take(&static_sem_led, RT_WAITING_FOREVER);
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n", count);
#endif
        count++;
        rt_hw_led_on(0);
				if(count%2)
					rt_hw_led_on(2);
				if(count%4)
					rt_hw_led_on(1);
        rt_thread_delay( RT_TICK_PER_SECOND / 2 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(0);
				if(count%2)
					rt_hw_led_off(2);
				if(count%4)
					rt_hw_led_off(1);
        rt_thread_delay( RT_TICK_PER_SECOND / 2 );
		//rt_sem_detach(&static_sem_led);
    }
}

void led_thread_creat(void)
{
    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
static rt_uint8_t led_inited = 0;
void led(rt_uint32_t led, rt_uint32_t value)
{
    /* init led configuration if it's not inited. */
    if (!led_inited)
    {
        rt_hw_led_init();
        led_inited = 1;
    }

    if ( led == 0 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(0);
            break;
        case 1:
            rt_hw_led_on(0);
            break;
        default:
            break;
        }
    }

    if ( led == 1 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(1);
            break;
        case 1:
            rt_hw_led_on(1);
            break;
        default:
            break;
        }
    }
    if ( led == 2 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(2);
            break;
        case 1:
            rt_hw_led_on(2);
            break;
        default:
            break;
        }
    }

    if ( led == 3 )
    {
        /* set led status */
        switch (value)
        {
        case 0:
            rt_hw_led_off(3);
            break;
        case 1:
            rt_hw_led_on(3);
            break;
        default:
            break;
        }
    }
}
FINSH_FUNCTION_EXPORT(led, set led[0 - 3] on[1] or off[0].)
#endif
