#ifndef __IMU_API_H__
#define __IMU_API_H__
#include "parameterDef.h"
#include <math.h>
#include <stdbool.h>
#include "filter.h"
#include "imu_types.h"
#define RtA 		57.324841f				
#define AtR    		0.0174533f				
#define Acc_G 		0.0011963f				
#define Gyro_G 		0.0610351f				
#define Gyro_Gr		0.0010653f		
#define FILTER_NUM  200

#define IMU_ACC_IIR_LPF_ATTENUATION (500 / (2 * 3.1415 * 10))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)

void Prepare_Data(Axis3i16 *acc_in,Axis3i16 *acc_out);

void IMU6update(Axis3f *gyr, Axis3f *acc, T_float_angle *angle);
void IMU9update(Axis3f *gyr, Axis3f *acc, Axis3f *mag, T_float_angle *angle);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,T_float_angle *angle);
void Gyrupdate(Axis3f *gyr, T_float_angle *angle);
void imu9Init(void);
void gyro_offset(void);
void gyro_get_offset(void);
void imu9Read(Axis3f* gyroOut, Axis3f* accOut,Axis3f* magOut);

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);//add in 2015.2.3
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);//add in 2015.2.3
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
#endif
