#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "PWM_Out.h"
void Tim_Pwm_Out_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	
	//ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);	
	
	ROM_GPIOPinConfigure(GPIO_PG2_M1PWM0);
	ROM_GPIOPinConfigure(GPIO_PG3_M1PWM1);
	ROM_GPIOPinConfigure(GPIO_PG4_M1PWM2);
	ROM_GPIOPinConfigure(GPIO_PG5_M1PWM3);
	
	ROM_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_2 );
	ROM_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_3 );
	ROM_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_4 );
	ROM_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_5 );
	
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, Moto_PwmMax); //Down 模式最大65535 Up_Down模式最大为2*65535
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,1);//ROM_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 10);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,1);//ROM_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 10);
	
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, Moto_PwmMax); //Down 模式最大65535 Up_Down模式最大为2*65535
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,1);//ROM_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 10);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,1);//ROM_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_0) / 10);
	
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}


void Moto_PwmRflash(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM)
{
	if(MOTO1_PWM>=Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>=Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>=Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>=Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<=0)	MOTO1_PWM = Moto_PwmMin;
	if(MOTO2_PWM<=0)	MOTO2_PWM = Moto_PwmMin;
	if(MOTO3_PWM<=0)	MOTO3_PWM = Moto_PwmMin;
	if(MOTO4_PWM<=0)	MOTO4_PWM = Moto_PwmMin;
	
	//rt_kprintf("1=%d 2=%d 3=%d 4=%d\n",MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,MOTO1_PWM/* *1.25 */);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,MOTO2_PWM/* *1.25 */);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2,MOTO3_PWM/* *1.25 */);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3,MOTO4_PWM);
}

struct rt_device pwm_out_device;

/* RT-Thread Device Driver Interface */
static rt_err_t rt_pwm_out_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t rt_pwm_out_open(rt_device_t dev, rt_uint16_t oflag)
{

	return RT_EOK;
}

static rt_err_t rt_pwm_out_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_size_t rt_pwm_out_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{

	return size;
}

static rt_size_t rt_pwm_out_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{

	return size;
}

static rt_err_t rt_pwm_out_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd) 
		{
        case PWM_IOCTL_SET_FREQ: 
			{
				if (args == 0) 
					return RT_ERROR; 
				Moto_PwmRflash(1,1,1,1);			
					break;
			}
        case PWM_IOCTL_STOP: 
		{
            Moto_PwmRflash(0,0,0,0); 
            break;
		}
	}
	return RT_EOK;
}

void rt_hw_pwm_out_init(void)
{
		Tim_Pwm_Out_Init();
		
		/* register pwm_out device */
		pwm_out_device.type  = RT_Device_Class_Unknown;
		pwm_out_device.init 	= rt_pwm_out_init;
		pwm_out_device.open 	= rt_pwm_out_open;
		pwm_out_device.close = rt_pwm_out_close;
		pwm_out_device.read 	= rt_pwm_out_read;
		pwm_out_device.write = rt_pwm_out_write;
		pwm_out_device.control = rt_pwm_out_control;
		/* no private */
		pwm_out_device.user_data = RT_NULL;
		
		rt_device_register(&pwm_out_device, "pwm_out", 
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
		

}
#ifdef RT_USING_FINSH
#include <finsh.h>
static rt_uint8_t pwm_out_inited = 0;
void pwm(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM)
{
    /* init pwm out configuration if it's not inited. */
    if (!pwm_out_inited)
    {
        rt_hw_pwm_out_init();
        pwm_out_inited = 1;
    }

		
    Moto_PwmRflash(MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM);


}
FINSH_FUNCTION_EXPORT(pwm, set pwm.)
#endif
