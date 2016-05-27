/*
 * FILE								: battery.c
 * DESCRIPTION				: --
 * Author							: Lynx  84693469@qq.com
 * Copyright					:
 *
 * History
 * --------------------
 * Rev								: 0.00
 * Date								: 10/19/2013
 *
 * create.
 * --------------------
 */

//-----------------Include files-------------------------//
#include "battery.h"
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

rt_uint8_t battery_adc_initialize(void)
{
	
	ROM_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	
	
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE,64);
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	
	ADCSequenceConfigure(ADC0_BASE, 3,ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC0_BASE,3, 0, ADC_CTL_CH1| ADC_CTL_IE |ADC_CTL_END);
	
	ROM_ADCSequenceEnable(ADC0_BASE, 3);
	rt_kprintf("battery_adc_initialize\n");
	return 0;
}

rt_uint32_t read_battert_adc_value()
{
	uint32_t battertValue;
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false));
	ADCSequenceDataGet(ADC0_BASE, 3, &battertValue);
	return battertValue;
	
}


#ifdef RT_USING_FINSH
#include <finsh.h>
#include <stdio.h>
//static rt_uint8_t adc_inited = 0;
static void battery()
{
	uint32_t battertValue;
	float voltageValue;
	char buff[100];
	//for(;;)
	{
	battertValue=read_battert_adc_value();
	rt_kprintf("value=%d\n",battertValue);
	if(battertValue<=4095)
	{
		voltageValue=(float) battertValue/4096*3.25f*2;
		sprintf(buff,"voltageValue=%3fV\n",voltageValue);
		rt_kprintf("%s",buff);
	}
}
}
FINSH_FUNCTION_EXPORT(battery, read battery value.)
#endif

