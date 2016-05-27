#ifndef PARAMETERDEF_H
#define PARAMETERDEF_H
#include <stdbool.h>
#include <stdint.h>


typedef struct{
				float rol;
				float pit;
				float yaw;}T_float_angle;
typedef struct{
				float X;
				float Y;
				float Z;}T_float_xyz;
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}T_int16_xyz;
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t YAW;
				int16_t THROTTLE;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;}T_RC_Data;
typedef struct{
				T_float_angle ANGLE;//16
				int16_t       ALT_CSB;//超声波高度
				int32_t       ALT_PRS;//气压计高度，单位厘米
				int8_t	      ARMED;	//=0,锁定
		
				
}T_RC_Status;
typedef struct{
	T_int16_xyz ACC;
	T_int16_xyz GYR;
	T_int16_xyz MAG;
}T_RC_Sensor;

typedef struct{
	T_int16_xyz ACC;
	T_int16_xyz ACC_S;
	T_int16_xyz ACC_L;
}T_RC_Acc_Anl;

typedef struct{
	float Voltage1;
	float Voltage2;
	float Voltage3;	
}T_RC_Voltage;

/*
typedef struct PID{
				float P;
				float pout;
				float I;
				float iout;
				float D;
				float dout;
				float IMAX;
				float OUT;}PID;
*/

extern T_float_angle g_angle;				
#endif
