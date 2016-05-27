#include "imu_api.h"
#include "control_api.h"
#include <rthw.h>
#include <rtthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "imu_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "parameterDef.h"
#include "calibration_routines.h"
#include "led.h"
#include <dfs_posix.h> 
#include "PWM_Out.h"
#include "pid.h"
#include "controller.h"

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

/* Utils Conversion macro */
#define C_BITS_TO_16(X) ((X)<<(16-9))
#define C_16_TO_BITS(X) ((X)>>(16-9)&((1<<9)-1))


T_float_angle angleActual;
T_float_angle angleDesired;
Axis3f g_gyro,g_acc,g_mag;

PidObject g_pid;

static rt_uint8_t stabilizer_stack[2048+4000];
static struct rt_thread stabilizer_thread;

extern float q0,q1,q2,q3;
extern Axis3i16   gyroMpu;

extern T_RC_Data rc_data;


//add in 2015.2.3
float rollRateDesired;
float pitchRateDesired;
float yawRateDesired;
uint32_t motorPowerM4;
uint32_t motorPowerM2;
uint32_t motorPowerM1;
uint32_t motorPowerM3;
uint16_t actuatorThrust=0;
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);
static void distributePower(const uint16_t thrust, const int16_t roll,const int16_t pitch, const int16_t yaw);

rt_uint8_t lock;

void rt_stabilizer_thread_entry(void *parameter)
{	
	float r,p,y;
	int16_t  actuatorRoll;
	int16_t  actuatorPitch;
	int16_t  actuatorYaw;
//	float *_x, *_y,*_z;
//	float sphere_x=0, sphere_y=0, sphere_z=0;
	//float sphere_radius;
	char buff[100];
	uint16_t i=0;
	//int fd;
	//float mx,my,mz,norm,mag_yaw;
	uint8_t attitudeCounter=0;
	
	controllerInit();
	imu9Init();
	//陀螺仪消除静差
#if 0
	rt_thread_delay(500);
	gyro_offset();
#endif
	gyro_get_offset();	
	
	rollRateDesired=0;
	pitchRateDesired=0;
	yawRateDesired=0;
	
	/*
	_x = rt_malloc(sizeof(_x)*501);
	_y = rt_malloc(sizeof(_y)*501);
	_z = rt_malloc(sizeof(_z)*501);
	
	fd = open("/mag_raw.txt", O_WRONLY | O_CREAT,0);
	for(i=0;i<=500;i++)
	{
	imu9Read(&g_gyro,&g_acc,&g_mag);//读取9轴传感器数据
	_x[i] = g_mag.x;
	_y[i] = g_mag.y;
	_z[i] = g_mag.z;
	sprintf(buff,"%d %f %f %f\n",i,_x[i],_y[i],_z[i]);
	write(fd, buff, strlen(buff));
	rt_kprintf("%s",buff);
	rt_thread_delay(50);
	}
	close(fd);

	i=sphere_fit_least_squares(_x,_y,_z,500,100,0,&sphere_x,&sphere_y,&sphere_z,&sphere_radius);

	fd = open("/sphere.txt", O_WRONLY | O_CREAT,0);
	sprintf(buff,"i=%d,sphere_x=%f,sphere_y=%f,sphere_z=%f,sphere_radius=%f\n",i,sphere_x,sphere_y,sphere_z,sphere_radius);
	rt_kprintf("%s",buff);
	write(fd, buff, strlen(buff));
	rt_thread_delay(500);
	close(fd);	
	
	rt_free(_x);
	rt_free(_y);
	rt_free(_z);
	*/
	lock=0;
loop:
	rt_thread_delay(100);
	if(lock ==0)
	{
		if(rc_data.YAW >=1800)
		{
			rt_thread_delay(1000);
			
			if(rc_data.YAW >=1800)
			{
				lock=1;
				goto start;
			}
			else
			{
				goto loop;
			}
		}
		else
		{
			goto loop;
		}
	}
	else
	{
		goto loop;
	}

start:
	for(;;)
	{
		
		
		//AHRSupdate(g_gyro.x,g_gyro.y,g_gyro.z,g_acc.x,g_acc.y,g_acc.z,g_mag.x--sphere_x,g_mag.y-sphere_y,g_mag.z-sphere_z,&gngleActual);
		/*
		mx = g_mag.x-sphere_x;
		my = g_mag.y-sphere_y;
		mz = g_mag.z-sphere_z;

		norm = sqrt(mx*mx + my*my + mz*mz);          
		mx /= norm;
		my /= norm;
		mz /= norm;
		mag_yaw=atan2f(my,mx)*57.3f;	*/
		
		if(lock == 1)
		{
		imu9Read(&g_gyro,&g_acc,&g_mag);//读取9轴传感器数据
		angleDesired.rol = (rc_data.ROLL - 1500) * 0.06f;
		angleDesired.pit = (rc_data.PITCH - 1500) * 0.06f;
		angleDesired.yaw = angleActual.yaw + (rc_data.YAW - 1500) * 0.2f;
		actuatorThrust = (rc_data.THROTTLE-1000)*60;
		if(rc_data.THROTTLE<1000)
		{
			actuatorThrust=0;
		}
		
		sensfusion6UpdateQ(g_gyro.x,g_gyro.y,g_gyro.z,g_acc.x,g_acc.y,g_acc.z,0.002f);
		sensfusion6GetEulerRPY(&angleActual.rol,&angleActual.pit,&angleActual.yaw);
		
		
		
		if (++attitudeCounter >= 2)
		{
			controllerCorrectAttitudePID(angleActual.rol, angleActual.pit, angleActual.yaw,
										 angleDesired.rol, angleDesired.pit, angleDesired.yaw,
                                         &rollRateDesired, &pitchRateDesired, &yawRateDesired);//角度PID,输出为角速度
			attitudeCounter = 0;
		}
		 yawRateDesired = angleDesired.yaw ;
		
		controllerCorrectRatePID(g_gyro.x, g_gyro.y, g_gyro.z,rollRateDesired, pitchRateDesired, yawRateDesired);
		
		
		controllerGetActuatorOutput(&actuatorRoll, &actuatorPitch, &actuatorYaw);

		distributePower(actuatorThrust, actuatorRoll, actuatorPitch, actuatorYaw);
		i++;
		if(i==100)
		{
			i=0;
			//sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
			//sprintf(buff,"q0-q3:%f,%f,%f,%f\n",q0,q1,q2,q3);
			//sprintf(buff,"1-rol=%f,pit=%f,yaw=%f\n",angleActual.rol,angleActual.pit,angleActual.yaw);
			//rt_kprintf("%s\n",buff);
			//sprintf(buff,"g_gyro=%f,%f,%f\nyaw=%f\n",g_gyro.x,g_gyro.y,g_gyro.z,yaw);
			//sprintf(buff,"mag_yaw=%f\n",mag_yaw);
			
			//sprintf(buff,"2-rol=%f,pit=%f,yaw=%f",angleDesired.rol,angleDesired.pit,angleDesired.yaw);
			//rt_kprintf("%s\n",buff);
			//rt_kprintf("%s\n",buff);

			//sprintf(buff,"rollRateD=%f,pitchRateD=%f,yawRateD=%f",rollRateDesired,pitchRateDesired,yawRateDesired);
			//sprintf(buff,"g_x=%f,g_y=%f,g_z=%f",g_gyro.x,g_gyro.y,g_gyro.z);
			//rt_kprintf("%s\n",buff);
			//rt_kprintf("M1=%d,M2=%d,M3=%d,M4=%d\n",motorPowerM1,motorPowerM2,motorPowerM4,motorPowerM4);
			//rt_kprintf("actuatorRoll=%d, actuatorPitch=%d, actuatorYaw\=%d\n",actuatorRoll, actuatorPitch, actuatorYaw);
		}
	
		rt_thread_delay(2);
	}
	else
	{
		Moto_PwmRflash(0,0,0,0);
		goto loop;
	}
}	
}
void stabilizer_thread_creat(void)
{
    rt_err_t result;
    result = rt_thread_init(&stabilizer_thread,
                            "stable",
                            rt_stabilizer_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&stabilizer_stack[0],
                            sizeof(stabilizer_stack),
                            4,
                            1000);
    if (result == RT_EOK)
    {
        rt_thread_startup(&stabilizer_thread);
    }
}


static void distributePower(const uint16_t thrust,  int16_t roll,int16_t pitch,  int16_t yaw)
{
	roll = roll >> 1;
	pitch = pitch >> 1;
	motorPowerM1 =  limitThrust(thrust - roll + pitch - yaw);
	motorPowerM2 =  limitThrust(thrust + roll - pitch - yaw);
	motorPowerM3 =  limitThrust(thrust - roll - pitch + yaw);
	motorPowerM4 =  limitThrust(thrust + roll + pitch + yaw);
	
	if(thrust<=0)
	{
		Moto_PwmRflash(0,0,0,0);
	}else
	{
		//rt_kprintf("M1=%d,M2=%d,M3=%d,M4=%d\n",motorPowerM1,motorPowerM2,motorPowerM4,motorPowerM4);
		Moto_PwmRflash(C_16_TO_BITS(motorPowerM1)*4,C_16_TO_BITS(motorPowerM2)*4,C_16_TO_BITS(motorPowerM3)*4,C_16_TO_BITS(motorPowerM4)*4);	
	}
}

static uint16_t limitThrust(int32_t value)
{
  if(value > UINT16_MAX)
  {
    value = UINT16_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

// Constrain value between min and max
static float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

// Deadzone
static float deadband(float value, const float threshold)
{
  if (fabs(value) < threshold)
  {
    value = 0;
  }
  else if (value > 0)
  {
    value -= threshold;
  }
  else if (value < 0)
  {
    value += threshold;
  }
  return value;
}