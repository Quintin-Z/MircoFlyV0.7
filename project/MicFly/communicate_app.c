#include <rtthread.h>
#include "transfer_package_api.h"
#include "imu_types.h"
void data_processing_creat(void);

extern T_float_angle angleActual;
extern Axis3i16   gyroMpu;
extern Axis3i16   accelMpu;
extern Axis3i16   accelSlip;
extern Axis3i16   accelLPF;
extern Axis3i16   accelLPFAligned;
extern Axis3i16   magMpu;
extern Axis3i16   magCorrect;
extern Axis3i16 *gyroOffset;

extern rt_uint8_t lock;
struct rf_rx_msg
{
	rt_device_t dev;
	rt_size_t size;
};

static rt_mq_t rf_rx_mq;

static rt_uint8_t com_stack[1024];
static struct rt_thread com_thread;
rt_err_t data_processing(rt_device_t dev, rt_size_t size)
{
	struct rf_rx_msg msg;
	msg.dev = dev;
	msg.size = size;
	rt_mq_send(rf_rx_mq, &msg, sizeof(struct rf_rx_msg));
	
	//rt_uint8_t buff[32];
	//rt_device_read(dev,0,buff,size);
	//data_receive_anl(buff,size);
	return RT_EOK;
}

void rt_com_thread_entry(void *parameter)
{
	rt_device_t nrf;//设备描述符
	rt_device_t bat;
	
	rt_uint32_t adcValue[3];
	T_RC_Voltage rc_voltage_buff;
	T_RC_Status rc_status_buff;
	T_RC_Sensor rc_sensor_buff;
	T_RC_Acc_Anl rc_sensor_anl_buff;
	rt_uint8_t *send_buff;
	rt_uint8_t pack_size;
	rt_uint8_t voltageDiv=0;
	rt_uint8_t SensorDiv=0;
	rt_uint8_t statusDiv=0;
	rt_uint8_t sensorAnlDiv=0;
	
	char buff[100];	
	
	
	
	data_processing_creat();//创建遥控数据处理线程	
	rf_rx_mq = rt_mq_create("rf_rx",sizeof(struct rf_rx_msg),sizeof(struct rf_rx_msg),RT_IPC_FLAG_FIFO);
	
	nrf = rt_device_find("nrf24");
	if(nrf != RT_NULL)
    {
        rt_device_set_rx_indicate(nrf, data_processing);		
		rt_device_open(nrf, RT_DEVICE_OFLAG_RDWR);
    }else
	{
		rt_kprintf("spi device nrf24 not found!\n");
	}

	bat = rt_device_find("battery");
	if(bat != RT_NULL)
    {
		rt_device_open(bat, RT_DEVICE_OFLAG_RDONLY);
    }else
	{
		rt_kprintf("bat not found!\n");
	}
	
	for(;;)
	{
		voltageDiv++;
		SensorDiv++;
		statusDiv++;
		sensorAnlDiv++;
		send_buff = rt_malloc(32);
		
		if(sensorAnlDiv >=1)
		{
			sensorAnlDiv = 0;
			//原始数据
			rc_sensor_anl_buff.ACC.X=accelMpu.x;
			rc_sensor_anl_buff.ACC.Y=accelMpu.y;
			rc_sensor_anl_buff.ACC.Z=accelMpu.z;

			//滤波后数据
			rc_sensor_anl_buff.ACC_S.X=accelSlip.x;
			rc_sensor_anl_buff.ACC_S.Y=accelSlip.y;
			rc_sensor_anl_buff.ACC_S.Z=accelSlip.z;

			rc_sensor_anl_buff.ACC_L.X = accelLPF.x;
			rc_sensor_anl_buff.ACC_L.Y = accelLPF.y;
			rc_sensor_anl_buff.ACC_L.Z = accelLPF.z;
			pack_size = data_acc_anl_package(&rc_sensor_anl_buff,send_buff);
			//rt_kprintf("size=%d",pack_size);
			rt_device_write(nrf,RT_NULL,send_buff,pack_size);
			//rt_free(rc_sensor_anl_buff);
		}
		
		if(statusDiv >= 1)
		{
			statusDiv = 0;
			rc_status_buff.ANGLE.rol = angleActual.rol;
			rc_status_buff.ANGLE.pit = angleActual.pit;
			rc_status_buff.ANGLE.yaw = angleActual.yaw;
			pack_size = data_status_package(&rc_status_buff,send_buff);
			rt_device_write(nrf,RT_NULL,send_buff,pack_size);
		}
		
		if(SensorDiv >= 2)
		{
			SensorDiv = 0;
			rc_sensor_buff.ACC.X=accelMpu.x;
			rc_sensor_buff.ACC.Y=accelMpu.y;
			rc_sensor_buff.ACC.Z=accelMpu.z;
			rc_sensor_buff.GYR.X=gyroMpu.x - gyroOffset->x;
			rc_sensor_buff.GYR.Y=gyroMpu.y - gyroOffset->y;
			rc_sensor_buff.GYR.Z=gyroMpu.z - gyroOffset->z;
			rc_sensor_buff.MAG.X=magMpu.x;
			rc_sensor_buff.MAG.Y=magMpu.y;
			rc_sensor_buff.MAG.Z=magMpu.z;	
			pack_size = data_sensor_package(&rc_sensor_buff,send_buff);
			rt_device_write(nrf,RT_NULL,send_buff,pack_size);
		}
		
		if(voltageDiv >= 4)
		{
			voltageDiv = 0;
			rt_device_read(bat,0,adcValue,3);
			rc_voltage_buff.Voltage1 = (float) adcValue[0]/4096*3.25f*2;
			rc_voltage_buff.Voltage2 = (float) adcValue[1]/4096*3.25f*2;
			rc_voltage_buff.Voltage3 = (float) adcValue[2]/4096*3.25f*2;
			//sprintf(buff,"V=%f\n",rc_voltage_buff.Voltage3+rc_voltage_buff.Voltage2+rc_voltage_buff.Voltage1);
			//rt_kprintf("%s",buff);
			if((rc_voltage_buff.Voltage3+rc_voltage_buff.Voltage2+rc_voltage_buff.Voltage1) <=10.05 )
			{
				lock =0;
				//rt_kprintf("lock =%d\n",lock);
			}
			pack_size = data_voltage_package(&rc_voltage_buff,send_buff);
			rt_device_write(nrf,RT_NULL,send_buff,pack_size);
		}
		rt_free(send_buff);
		rt_thread_delay(5);
	}
}
void com_thread_creat(void)
{
    rt_err_t result;
    result = rt_thread_init(&com_thread,
                            "com",
                            rt_com_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&com_stack[0],
                            sizeof(com_stack),
                            5,
                            10);
    if (result == RT_EOK)    {
        rt_thread_startup(&com_thread);
    }
}

void data_processing_thread_entry(void *parameter)
{
	struct rf_rx_msg msg;
	rt_err_t result = RT_EOK;
	rt_uint8_t *recv_buff;
	rt_device_t led;
	
	led = rt_device_find("led");
	
	rt_device_open(led,RT_DEVICE_OFLAG_RDWR);
	for(;;)
	{
		result = rt_mq_recv(rf_rx_mq, &msg, sizeof(struct rf_rx_msg), RT_WAITING_FOREVER);
		if (result == RT_EOK)
		{
			rt_device_control(led,3,(void *)0);
			recv_buff = rt_malloc(msg.size);
			rt_device_read(msg.dev,0,recv_buff,msg.size);
			data_receive_anl(recv_buff,msg.size);
			rt_free(recv_buff);
		}
	}
}

static rt_uint8_t process_stack[1024];
static struct rt_thread process_thread;
void data_processing_creat(void)
{
    rt_err_t result;
    result = rt_thread_init(&process_thread,
                            "data",
                            data_processing_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&process_stack[0],
                            sizeof(process_stack),
                            5,
                            10);
    if (result == RT_EOK)
    {
        rt_thread_startup(&process_thread);
    }
}
