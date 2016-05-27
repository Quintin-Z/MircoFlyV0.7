#ifndef __RTT_LED_H__
#define __RTT_LED_H__
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define LED_ON     0x01
#define LED_OFF    0x02
#define LED_TOGGLE 0x03

struct rt_led_device
{
	struct rt_device led_device;
	rt_timer_t timer;
	rt_uint8_t mode[3];
	rt_uint8_t frequency[3];
	
};

#endif //__RTT_LED_H__
