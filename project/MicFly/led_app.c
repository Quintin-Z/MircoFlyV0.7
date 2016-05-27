#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "rtt_led.h"
static rt_uint8_t led_stack[256];
static struct rt_thread led_thread;

extern rt_uint8_t lock;

void rt_led_thread_entry(void *parameter)
{
	rt_device_t led;
	
	led = rt_device_find("led");
	
	rt_device_open(led,RT_DEVICE_OFLAG_RDWR);
	
	for(;;)
	{
		
		rt_device_control(led,3,(void *)0);
		if(lock == 1)
		{
			rt_device_control(led,LED_ON,(void *)1);
		}else
		{
			rt_device_control(led,LED_OFF,(void *)1);
		}
		//rt_device_control(led,3,(void *)1);
		rt_thread_delay(500);
	}
}

void led_thread_creat(void)
{
    rt_err_t result;
	result = rt_thread_init(&led_thread,
                            "led",
                            rt_led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&led_stack[0],
                            sizeof(led_stack),
                            15,
                            20);
	if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }						
}
