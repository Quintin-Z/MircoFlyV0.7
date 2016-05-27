#include <stdio.h>
#include <rtthread.h>
#include "nRF24L01.h"
#include "parameterDef.h"
#include "battery.h"
#include "imu_types.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

T_RC_Data m_rc_data;
T_RC_Data g_rc_data;

extern Axis3i16   gyroMpu;
extern Axis3i16   accelMpu;
extern Axis3i16   magMpu;
extern Axis3i16   magCorrect;
extern T_float_angle angleActual;


static rt_uint8_t transfer_stack[2048];
static struct rt_thread transfer_thread;

rt_uint8_t data_to_send[50];


void Data_Receive_Anl(rt_uint8_t *data_buf,rt_uint8_t num)
{
//	rt_int16_t rc_value_temp;
	rt_uint8_t sum = 0;
	
	for(rt_uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	
	if(!(sum==*(data_buf+num-1)))
		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
		return;		//判断帧头
/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X03)
	{
		m_rc_data.THROTTLE = (rt_int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		m_rc_data.YAW = (rt_int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		m_rc_data.ROLL = (rt_int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		m_rc_data.PITCH = (rt_int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		m_rc_data.AUX1 = (rt_int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		m_rc_data.AUX2 = (rt_int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		m_rc_data.AUX3 = (rt_int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		m_rc_data.AUX4 = (rt_int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		m_rc_data.AUX5 = (rt_int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		m_rc_data.AUX6 = (rt_int16_t)(*(data_buf+21)<<8)|*(data_buf+22);
		//Rc_Fun(&Rc_D,&Rc_C);
	}
}

void NRF24_Check_Event(void)
{
	rt_uint8_t buff[34];
	rt_uint8_t sta = SPI_Read(NRF_READ_REG + NRFRegSTATUS);

	//rt_kprintf("sta=0x%x\n",sta);
	if(sta & (0x01 << 6))
	{
		rt_uint8_t rx_len = SPI_Read(R_RX_PL_WID);
		if(rx_len<33)
		{
			SPI_Read_Buf(RD_RX_PLOAD,buff,rx_len);// read receive payload from RX_FIFO buffer
			rt_kprintf("Data_Receive:");
			for(rt_uint8_t i=0;i<32;i++)
			{
				rt_kprintf("%c",buff[i]);
			}
			rt_kprintf("\n");
			//LED1_ONOFF();
			Data_Receive_Anl(buff,rx_len);
		}
		else 
		{
			SPI_RW_Reg(FLUSH_RX,0xff);//清空缓冲区
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<5))
	{
		//LED1_ONOFF();
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<4))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			SPI_RW_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	SPI_RW_Reg(RF_WRITE_REG + NRFRegSTATUS, sta);
}

void Data_Send_Status(T_RC_Status *data)
{
	rt_uint8_t _cnt=0;
	data_to_send[_cnt++]=0XAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	rt_int16_t _temp;
	_temp =  data->ANGLE.rol*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp =  data->ANGLE.pit*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp =  data->ANGLE.yaw*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = data->ALT_CSB;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(data->ALT_PRS);
	data_to_send[_cnt++]=BYTE2(data->ALT_PRS);	
	data_to_send[_cnt++]=BYTE1(data->ALT_PRS);
	data_to_send[_cnt++]=BYTE0(data->ALT_PRS);
	
	if(data->ARMED==0)			
		data_to_send[_cnt++]=0xA0;	//锁定
	else if(data->ARMED==1)		
		data_to_send[_cnt++]=0xA1;

	data_to_send[3] = _cnt-4;
	
	rt_uint8_t sum = 0;
	for(rt_uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	NRF24L01_TxPacket(data_to_send);
}

void Data_Send_Senser(T_RC_Sensor *data)
{
	rt_uint8_t _cnt=0;
	data_to_send[_cnt++]=0XAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	rt_int16_t _temp;
	_temp = data->ACC.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = data->GYR.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = data->MAG.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	rt_uint8_t sum = 0;
	for(rt_uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	NRF24L01_TxPacket(data_to_send);
	
	
}

void Data_Send_Voltage(T_RC_Voltage *data)
{
	rt_uint8_t _cnt=0;
	data_to_send[_cnt++]=0XAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;//电压功能字
	data_to_send[_cnt++]=0;
	
	rt_uint16_t _temp;//电压肯定为正
	_temp = data->Voltage1*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = data->Voltage2*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp =  data->Voltage3*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	rt_uint8_t sum = 0;
	for(rt_uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	NRF24L01_TxPacket(data_to_send);
	
}

void rt_transfer_thread_entry(void *parameter)
{
//	rt_uint16_t ms_20=0,ms_100=0,ms_1000=0;
//	char buff[33];
	T_RC_Status data;
//	T_RC_Voltage voltage;
	T_RC_Sensor Sensor;
//	rt_uint32_t _temp[3];
//	rt_uint8_t i;
	
	rt_thread_delay(1000);
	
	for(;;)
	{
		//NRF24_Check_Event();
		/*Pms_20++;
		if(ms_20==100)
		{
			ms_20=0;
			for(i=0;i<3;i++)
			{	
				_temp[i]=read_battert_adc_value();
			}
			voltage.Voltage1=(float)_temp[0]/4095.0f*3.3f*2;
			voltage.Voltage2=(float)_temp[1]/4095.0f*3.3f*2;
			voltage.Voltage3=(float)_temp[2]/4095.0f*3.3f*2;
			Data_Send_Voltage(&voltage);
		}*/

#if 1
		data.ANGLE.rol=angleActual.rol;
		data.ANGLE.pit=angleActual.pit;
		data.ANGLE.yaw=angleActual.yaw;
		data.ALT_CSB=1234;
		data.ALT_PRS=1234;
		data.ARMED=0;
		Data_Send_Status(&data);

		Sensor.ACC.X=accelMpu.x;
		Sensor.ACC.Y=accelMpu.y;
		Sensor.ACC.Z=accelMpu.z;
		Sensor.GYR.X=gyroMpu.x;
		Sensor.GYR.Y=gyroMpu.y;
		Sensor.GYR.Z=gyroMpu.z;
		Sensor.MAG.X=magMpu.x;
		Sensor.MAG.Y=magMpu.y;
		Sensor.MAG.Z=magMpu.z;	

		Sensor.MAG.X=magCorrect.x;
		Sensor.MAG.Y=magCorrect.y;
		Sensor.MAG.Z=magCorrect.z;			
		Data_Send_Senser(&Sensor);
		
#endif
		rt_thread_delay(20);
		
	}
}
void transfer_thread_creat(void)
{
    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&transfer_thread,
                            "transfer",
                            rt_transfer_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&transfer_stack[0],
                            sizeof(transfer_stack),
                            19,
                            100);
    if (result == RT_EOK)
    {
        rt_thread_startup(&transfer_thread);
    }
		
}
