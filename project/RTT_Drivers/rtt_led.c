/*
 * File      : 

 * Change Logs:
 * Date           Author       Notes
 * 2014-08-10     ZzQuan      
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "rtt_led.h"

struct rt_led_device led_device;

void rt_hw_led_off(rt_uint8_t n)
{
    switch (n)
    {
    case 0:
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        break;
    case 1:
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        break;
    case 2:
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        break;
    default:
        break;
    }
}
void rt_hw_led_on(rt_uint8_t n)
{
    switch (n)
    {
    case 0:
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        break;
    case 1:
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        break;
    case 2:
        ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
        break;
    default:
        break;
    }
}

void rt_hw_led_toggle(rt_uint8_t n)
{
    switch (n)
    {
    case 0:
		if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1)&GPIO_PIN_1)
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
		}
		else
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
		}
        break;
    case 1:
		if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_2)&GPIO_PIN_2)
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
		}
		else
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
		}
        break;
    case 2:
		if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_3)&GPIO_PIN_3)
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}
		else
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
		}
        break;
    default:
        break;
    }
}



/* RT-Thread device interface */
static rt_err_t rt_led_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_led_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_led_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_led_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);
	
	switch (cmd) {
	case LED_ON:
		rt_hw_led_on((rt_uint8_t) args);
		break;

	case LED_OFF:
		rt_hw_led_off((rt_uint8_t) args);
		break;

	case LED_TOGGLE:
		rt_hw_led_toggle((rt_uint8_t) args);
		break;
	default:
		return RT_ERROR;
	}

    return RT_EOK;
}

static rt_size_t rt_led_read(rt_device_t dev,rt_off_t pos,void* buffer,rt_size_t size)
{
	 return RT_EOK;
}

static rt_size_t rt_led_write(rt_device_t dev,rt_off_t pos,const void* buffer,rt_size_t size)
{
	return RT_EOK;
}

int rt_hw_led_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	
    led_device.led_device.type    = RT_Device_Class_Char;
    led_device.led_device.init    = rt_led_init;
    led_device.led_device.open    = rt_led_open;
    led_device.led_device.close   = rt_led_close;
    led_device.led_device.read    = rt_led_read;
    led_device.led_device.write   = rt_led_write;
    led_device.led_device.control = rt_led_control;
	
	rt_device_register(&led_device.led_device,"led",RT_DEVICE_FLAG_RDWR);
}
INIT_BOARD_EXPORT(rt_hw_led_init);
