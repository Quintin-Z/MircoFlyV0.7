#include "transfer_package_api.h"


T_RC_Data rc_data;
void data_receive_anl(rt_uint8_t *data_buf,rt_uint8_t num)
{
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
		rc_data.THROTTLE = (rt_int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		rc_data.YAW = (rt_int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
		rc_data.ROLL = (rt_int16_t)(*(data_buf+8)<<8)|*(data_buf+9);
		rc_data.PITCH = (rt_int16_t)(*(data_buf+10)<<8)|*(data_buf+11);
		rc_data.AUX1 = (rt_int16_t)(*(data_buf+12)<<8)|*(data_buf+13);
		rc_data.AUX2 = (rt_int16_t)(*(data_buf+14)<<8)|*(data_buf+15);
		rc_data.AUX3 = (rt_int16_t)(*(data_buf+16)<<8)|*(data_buf+17);
		rc_data.AUX4 = (rt_int16_t)(*(data_buf+18)<<8)|*(data_buf+19);
		rc_data.AUX5 = (rt_int16_t)(*(data_buf+20)<<8)|*(data_buf+21);
		rc_data.AUX6 = (rt_int16_t)(*(data_buf+21)<<8)|*(data_buf+22);
		//rt_kprintf("throttle=%d,yaw=%d,roll=%d,pitch=%d\n",rc_data.THROTTLE,rc_data.YAW,rc_data.ROLL,rc_data.PITCH);
		//Rc_Fun(&Rc_D,&Rc_C);
	}
}

rt_uint8_t data_status_package(T_RC_Status *data,rt_uint8_t data_to_send[])
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
	
	return _cnt;
}

rt_uint8_t data_sensor_package(T_RC_Sensor *data,rt_uint8_t data_to_send[])
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
	
	return _cnt;
}

rt_uint8_t data_voltage_package(T_RC_Voltage *data,rt_uint8_t data_to_send[])
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
	
	_temp = data->Voltage3*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	rt_uint8_t sum = 0;
	for(rt_uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	return _cnt;
}

rt_uint8_t data_acc_anl_package(T_RC_Acc_Anl *data,rt_uint8_t data_to_send[])
{
	rt_uint8_t _cnt=0;
	rt_int16_t _temp;
	data_to_send[_cnt++]=0XAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;//高级收码功能字
	data_to_send[_cnt++]=0;
	
///////////////////////////////////////////////////////
	_temp = data->ACC.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = data->ACC_S.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = data->ACC_S.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC_S.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = data->ACC_L.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = data->ACC_L.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->ACC_L.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

///////////////////////////////////////////////////////
/*	_temp = data->GYR.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR_F.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR_F.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->GYR_F.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
///////////////////////////////////////////////////////
	_temp = data->MAG.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG_F.X;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG_F.Y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data->MAG_F.Z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);*/
////////////////////////////////////////////////////////
	data_to_send[3] = _cnt-4;
	
	rt_uint8_t sum = 0;
	for(rt_uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	return _cnt;
}
