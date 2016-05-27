#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/spi.h>
#include "rtt_nrf24l01.h"

rt_err_t data_input(rt_device_t dev, rt_size_t size)
{
	char buff[32];
	rt_device_read(dev,0,buff,size);
	rt_kprintf("2:");
	for(rt_uint8_t i=0;i<32;i++)
	{
	rt_kprintf("%c",buff[i]);
	}
	rt_kprintf("\n");
	//rt_kprintf("size=%d\n",size);
	return RT_EOK;
}
void rt_test_thread_entry(void *parameter)
{
	rt_device_t led;
	
	led = rt_device_find("led");
	
	rt_device_open(led,RT_DEVICE_OFLAG_RDWR);
	
	for(;;)
	{
		rt_device_control(led,3,(void *)0);
		rt_device_control(led,3,(void *)1);
		rt_device_control(led,3,(void *)2);
		rt_thread_delay(500);
	}
#if 0
	rt_device_t nrf;
	char buff[32]="zhongjiequan nrf24l01 test:0";
	nrf = rt_device_find("nrf24");
	if(nrf != RT_NULL)
    {
        rt_device_set_rx_indicate(nrf, data_input);	
		rt_device_open(nrf, RT_DEVICE_OFLAG_RDWR);
    }else
	{
		rt_kprintf("spi device nrf24 not found!\n");
	}
	//rt_device_set_rx_indicate()
	
	
	for(;;)
	{	

		buff[27]++;
		if(buff[27]>'9')
		{
			buff[27]='0';
		}
		rt_device_write(nrf,0,buff,32);
		
		rt_thread_delay(10);
	}
#endif

	
	/*
	struct rt_spi_device * spi_device;
	struct rt_spi_message message;
	struct rt_spi_configuration cfg;
	
	//rt_thread_delay(10000);
    spi_device = (struct rt_spi_device *)rt_device_find("ssi00");
	if(spi_device == RT_NULL)
    {
        rt_kprintf("spi device ssi00 not found!\n");
   
    }
	
	cfg.data_width=8;
	cfg.max_hz=8000000;
	cfg.mode = 0x00000000;
	rt_spi_configure(spi_device, &cfg);
	
	
	
	rt_uint8_t cmd_buffer[4];
	cmd_buffer[0]=0x1a;
	cmd_buffer[1]=0x1b;
	cmd_buffer[2]=0x1c;
	cmd_buffer[3]=0x1d;
	
	
	for(;;)
	{	
		rt_spi_send(spi_device,cmd_buffer,4);
		//rt_kprintf("rt_spi_send\n");
		rt_thread_delay(1000);
	}*/
}
static rt_uint8_t test_stack[2048];
static struct rt_thread test_thread;
void test_thread_creat(void)
{
    rt_err_t result;
	result = rt_thread_init(&test_thread,
                            "test_thread",
                            rt_test_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&test_stack[0],
                            sizeof(test_stack),
                            15,
                            2000);
	if (result == RT_EOK)
    {
        rt_thread_startup(&test_thread);
    }						
	

}
