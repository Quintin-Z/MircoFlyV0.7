#include <rthw.h>
#include <rtthread.h>

#include <stdbool.h>
#include <stdint.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include <driverlib/rom.h>
#include "driverlib/timer.h"
#include "battery.h"
struct rt_device battery_device;
/* RT-Thread device interface */
static rt_err_t rt_battery_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_battery_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_battery_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_battery_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);
	
    return RT_EOK;
}

static rt_size_t rt_battery_read(rt_device_t dev,rt_off_t pos,void* buffer,rt_size_t size)
{
	rt_uint8_t i;
	rt_uint32_t *value;
	value =(rt_uint32_t *) buffer;
	for(i=0;i<size;i++)
	{
		*value = read_battert_adc_value();
		value++;
	
	}
	return i+1;
}

static rt_size_t rt_battery_write(rt_device_t dev,rt_off_t pos,const void* buffer,rt_size_t size)
{
	return RT_EOK;
}

void rt_hw_battery_device_init(void)
{
	battery_adc_initialize();
	
    battery_device.type    = RT_Device_Class_Char;
    battery_device.init    = rt_battery_init;
	battery_device.open    = rt_battery_open;
    battery_device.close   = rt_battery_close;
    battery_device.read    = rt_battery_read;
    battery_device.write   = rt_battery_write;
    battery_device.control = rt_battery_control;
	
	rt_device_register(&battery_device,"battery",RT_DEVICE_FLAG_RDWR);

}
