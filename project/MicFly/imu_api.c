#include "imu_api.h"

#include <rtthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "imu_types.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"

#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"

#include "filter.h"

#include "drv_eeprom.h"

#define M_PI 3.14159265358979323846f


#define MPU6050_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6050_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6050_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6050_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6050_G_PER_LSB_16     (float)((2 * 16) / 65536.0)
	
#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_2000
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_8
	
#define IMU_1G_RAW            (int16_t)(1.0 / MPU6050_G_PER_LSB_8)

#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
typedef struct
{
  Axis3i16   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} BiasObj;

Axis3i16   gyroMpu;
Axis3i16   accelMpu;
Axis3i16   accelSlip;
Axis3i16   accelLPF;
Axis3i16   accelLPFAligned;
Axis3i16   magMpu;
Axis3i16   magCorrect;

Axis3i16 accOffset;
Axis3i16 *gyroOffset;


Axis3i32   accelStoredFilterValues;
uint8_t    imuAccLpfAttFactor;


#define MPU9150_I2C_ADDRESS     0x68

tI2CMInstance g_sI2CInst;
tMPU9150 g_sMPU9150Inst;
//tCompDCM g_sCompDCMInst;
volatile uint_fast8_t g_vui8I2CDoneFlag;
volatile uint_fast8_t g_vui8ErrorFlag;
volatile uint_fast8_t g_vui8DataFlag;

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// TODO: Make math util file
static float invSqrt(float x);
static float safe_asin(float v);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);


static bool isInit;

void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }
    g_vui8ErrorFlag = ui8Status;
}

void
MPU9150I2CIntHandler(void)
{
    I2CMIntHandler(&g_sI2CInst);
}

void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //rt_kprintf("\033[31;1m");
    rt_kprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);
    while(1)
    {
        //ROM_SysCtlReset();
		//
        // Do Nothing
        //
    }
}

int
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line )
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        // Do Nothing
        
    }
	
    if(g_vui8ErrorFlag)
    {
		//Handler(void);
		MPU9150AppErrorHandler(pcFilename, ui32Line);
		
		g_vui8I2CDoneFlag = 0;
		return 1;
    }
    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
	return 0;
}

void
imu9Init(void)
{
  if(isInit)
    return;
  
  // Wait for sensors to startup
  	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
	
	//
  
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
	
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE,GPIO_PIN_5);
	I2CIntRegister(I2C2_BASE,MPU9150I2CIntHandler);
	//I2CGlitchFilterEnable(I2C2_BASE);
  
	ROM_I2CMasterEnable(I2C2_BASE);
	
	I2CMInit(&g_sI2CInst, I2C2_BASE, INT_I2C2, 0xff, 0xff,ROM_SysCtlClockGet());
	rt_kprintf("i2c bus busy status=%d\n",I2CMasterBusBusy(I2C2_BASE));
  
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,MPU9150AppCallback, &g_sMPU9150Inst);
  	// Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__ );	
  
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_21_20;//配置数字低通滤波器
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_2000;//配置陀螺仪量程
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ | MPU9150_ACCEL_CONFIG_AFS_SEL_8G);//配置加速度量程
	/*从MPU9150_O_CONFIG寄存器地址开始写起,连续写三个寄存器 0x1A-0x1C*/
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3, MPU9150AppCallback, &g_sMPU9150Inst);
	// Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);	
	
	
	g_sMPU9150Inst.pui8Data[0] = 0;
    g_sMPU9150Inst.pui8Data[1] = 0;
    g_sMPU9150Inst.pui8Data[2] = 0;
	MPU9150Read(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3, MPU9150AppCallback, &g_sMPU9150Inst);
	// Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);
	
	rt_kprintf("MPU9150_O_CONFIG      (0x1A):0x%02x\nMPU9150_O_GYRO_CONFIG (0x1B):0x%02x\nMPU9150_O_ACCEL_CONFIG(0x1C):0x%02x\n",
				g_sMPU9150Inst.pui8Data[0],g_sMPU9150Inst.pui8Data[1],g_sMPU9150Inst.pui8Data[2]);
	
	//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 1K/(1+1)=500HZ
	g_sMPU9150Inst.pui8Data[0] = 0x01; //
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, g_sMPU9150Inst.pui8Data, 1, MPU9150AppCallback, &g_sMPU9150Inst);
    //Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);	
	
	
	g_sMPU9150Inst.pui8Data[0] = MPU9150_PWR_MGMT_1_CLKSEL_XG; //
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_PWR_MGMT_1, g_sMPU9150Inst.pui8Data, 1, MPU9150AppCallback, &g_sMPU9150Inst);
    //Wait for transaction to complete
    MPU9150AppI2CWait(__FILE__, __LINE__);	
	
	
	imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;
	
	cosPitch = cos(0 * 3.14/180);
	sinPitch = sin(0 * 3.14/180);
	cosRoll = cos(0 * 3.14/180);
	sinRoll = sin(0 * 3.14/180);
	
	isInit=1;
}

void mag_correct(Axis3i16 *magIn,Axis3i16 *magOut)
{
#if 1
	int32_t m_y;
	int32_t m_z;
	//magOut->x=magIn->x+28;
	//m_y=(975*magIn->y-115000)/1000;
	//m_y=(1105*magIn->y-105000)/1000;
	//m_z=(333*magIn->z-50)/1000;
	
	magOut->x=magIn->x+30;
	m_y=(950*magIn->y-99750)/1000;
	m_z=(1111*magIn->z+150000)/1000;
	magOut->y=m_y;
	magOut->z=m_z;
	//magOut->z=magIn->z;
	
#else 
	magOut->x = magIn->x;
	magOut->y = magIn->y;
	magOut->z = magIn->z;
#endif
}


void
imu9Read(Axis3f* gyroOut, Axis3f* accOut,Axis3f* magOut)
{
	//读取9轴原始数据
	MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);//触发读取
	MPU9150AppI2CWait(__FILE__, __LINE__);//等待读取成功
	
	MPU9150DataGyroGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &gyroMpu.x,(uint_fast16_t *) &gyroMpu.y,(uint_fast16_t *) &gyroMpu.z);
	MPU9150DataAccelGetRaw(&g_sMPU9150Inst,(uint_fast16_t *)&accelMpu.x,(uint_fast16_t *)&accelMpu.y,(uint_fast16_t *)&accelMpu.z);
	MPU9150DataMagnetoGetRaw(&g_sMPU9150Inst,(uint_fast16_t *)&magMpu.x,(uint_fast16_t *)&magMpu.y,(uint_fast16_t *)&magMpu.z);
	
	//mag_correct(&magMpu,&magCorrect);
	
	
	
	//对加速度进行IIR滤波
	imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,(int32_t)imuAccLpfAttFactor);
//	imuAccAlignToGravity(&accelLPF, &accelLPFAligned);
	//Prepare_Data(&accelMpu,&accelSlip);

	  // Re-map outputs
	gyroOut->x = ((int16_t)gyroMpu.x  - gyroOffset->x) * IMU_DEG_PER_LSB_CFG;
	gyroOut->y = ((int16_t)gyroMpu.y  - gyroOffset->y) * IMU_DEG_PER_LSB_CFG;
	gyroOut->z = ((int16_t)gyroMpu.z  - gyroOffset->z) * IMU_DEG_PER_LSB_CFG;
	//rt_kprintf("gyroOut:%d,%d,%d\n",gyroOffset.x,gyroMpu.y  - gyroOffset.y,gyroMpu.z  - gyroOffset.z);
	//rt_kprintf("gyroOffset:%d:%d,%d,%d\n",&gyroOffset->x,gyroOffset->x,gyroOffset->y,gyroOffset->z);
	accOut->x = ((int16_t)accelLPF.x /*- accelBias.bias.x*/) * IMU_G_PER_LSB_CFG;
	accOut->y = ((int16_t)accelLPF.y /*- accelBias.bias.y*/) * IMU_G_PER_LSB_CFG;
	accOut->z = ((int16_t)accelLPF.z /*- accelBias.bias.z*/) * IMU_G_PER_LSB_CFG;
	
	magOut->x = (int16_t) magMpu.x * 0.0000003f;
	magOut->y = (int16_t) magMpu.y * 0.0000003f;
	magOut->z = (int16_t) magMpu.z * 0.0000003f;

}


void gyro_offset(void)
{
	uint16_t i;
	int32_t gx,gy,gz;
	uint32_t eepromData[2];
	gx=0;
	gy=0;
	gz=0;
	for(i=0;i<5000;i++)
	{
		//读取9轴原始数据
		MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);//触发读取
		MPU9150AppI2CWait(__FILE__, __LINE__);//等待读取成功
		MPU9150DataGyroGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &gyroMpu.x,(uint_fast16_t *) &gyroMpu.y,(uint_fast16_t *) &gyroMpu.z);
		gx+=gyroMpu.x;
		gy+=gyroMpu.y;
		gz+=gyroMpu.z;
	}
	gx /= 5000;
	gy /= 5000;
	gz /= 5000;
	
	gx = (uint16_t) gx;
	gy = (uint16_t) gy;
	gz = (uint16_t) gz;
	rt_kprintf("\ngx=%x,gy=%x,gz=%x\n",gx,gy,gz);
	eepromData[0]=(gx<<16) + gy;
	eepromData[1]=(gz<<16) + 0x1234;
	eeprom_write_byte(eepromData,0,8);
	rt_kprintf("write to eeprom at 0x00-0x01:%8x,%8x\n",eepromData[0],eepromData[1]);
}

void gyro_get_offset(void)
{
	uint32_t eepromData[2];
	
	eeprom_read_byte(eepromData,0,8);
	gyroOffset = rt_malloc(sizeof(Axis3i16));
	rt_kprintf("eepromData[0]=%x,eepromData[1]=%x\n",eepromData[0],eepromData[1]);
	gyroOffset->x=(int16_t) (eepromData[0]>>16);
	gyroOffset->y=(int16_t) (eepromData[0] & 0x0000ffff);
	gyroOffset->z=(int16_t) (eepromData[1]>>16);
	rt_kprintf("%d:gyroOffset.x=%d,gyroOffset.y=%d,gyroOffset.z=%d\n",&gyroOffset->x,gyroOffset->x,gyroOffset->y,gyroOffset->z);

}

void mag_offset(void)
{
	
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

#if 1

void Prepare_Data(Axis3i16 *acc_in,Axis3i16 *acc_out)
{
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = acc_in->x;
	ACC_Y_BUF[filter_cnt] = acc_in->y;
	ACC_Z_BUF[filter_cnt] = acc_in->z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	acc_out->x = temp1 / FILTER_NUM;
	acc_out->y = temp2 / FILTER_NUM;
	acc_out->z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}

////////////////////////////////////////////////////////////////////////////////
#define Kp 2.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period???????

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMU6update(Axis3f *gyr, Axis3f *acc, T_float_angle *angle)
{
	float ax = acc->x,ay = acc->y,az = acc->z;
	float gx = gyr->x,gy = gyr->y,gz = gyr->z;
	//float yaw;
	float norm;
	//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	//float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	//float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if(ax*ay*az==0)return;
	
	//陀螺仪角度转换为弧度
	
	gx = gx * 3.14159265 / 180.0f;
	gy = gy * 3.14159265 / 180.0f;
	gz = gz * 3.14159265 / 180.0f;

	//重力加速度归一化*/
	norm = invSqrt(ax*ax + ay*ay + az*az);
	
	ax = ax *norm;
	ay = ay * norm;
	az = az * norm;
	
	if(norm>16500)
	{
		//Rc_C.ARMED=0;
	}
	
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	//叉乘
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	//比例运算
	exInt = exInt + ex * Ki;								  
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	//陀螺仪融合
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//一阶龙格库塔法更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//归一化处理
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	angle->pit = safe_asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 ;
	angle->rol = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 ;
	angle->yaw = atan2f(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)*57.3;
	
	
	
	
}


#define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain

float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki

// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * M_PI / 180;
  gy = gy * M_PI / 180;
  gz = gz * M_PI / 180;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 180 / M_PI;
  *pitch = -atan(gx / sqrt(gy*gy + gz*gz)) * 180 / M_PI;
  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / M_PI;
}


float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*gx + ay*gy + az*gz) - 1.0);
}



void IMU9update(Axis3f *gyr, Axis3f *acc, Axis3f *mag, T_float_angle *angle)
{
	float ax = acc->x,ay = acc->y,az = acc->z;
	float gx = gyr->x,gy = gyr->y,gz = gyr->z;
	float mx = mag->x,my = mag->y,mz = mag->z;
	float norm;
	float hx, hy, hz, bx, bz;//磁力相关
	float vx, vy, vz;
	float wx, wy, wz;//磁力相关
	float ex, ey, ez;//误差向量
//	float m_ex, m_ey, m_ez;//误差向量
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if(ax*ay*az==0)return;
	
	//陀螺仪角度转换为弧度
	gx = gx * 3.14159265 / 180.0f;
	gy = gy * 3.14159265 / 180.0f;
	gz = gz * 3.14159265 / 180.0f;

	//重力加速度归一化,利用比力来获取角度*/
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax *norm;
	ay = ay * norm;
	az = az * norm;

	//磁力计归一化,
	norm = invSqrt(mx*mx + my*my + mz*mz);          
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
    
	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        
	
	if(norm>16500)
	{
		//重力加速度过大,标志着遇到强烈撞击,停机保护
		//Rc_C.ARMED=0;
	}
	
	//获取当前机体方向向量
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	//磁力从C系到B系
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2); 
	
	
	//矩阵叉乘得出机体与重力角度的误差
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	//m_ex = (my*wz - mz*wy);
	//m_ey = (mz*wx - mx*wz);
	//m_ez = (mx*wy - my*wx);
	
	//char buff[100];
	//sprintf(buff,"m_ex=%f,m_ey=%f,m_ez=%f\n",m_ex,m_ey,m_ez);
	//rt_kprintf("%s",buff);
	
	//对误差进行微分运算后积分误差
	exInt = exInt + ex * Ki;								  
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	//对陀螺仪进行加速度互补滤波
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//使用一阶龙格库塔法更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//对四元数进行归一化处理
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	//获取欧拉角,单位为角度
	angle->pit = safe_asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
	angle->rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
	angle->yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)*57.3;
}

float Quaternion[4];
float Beta;
void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,T_float_angle *angle)
{
	//float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	
	float q4;
	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q0;
	float _2q2 = 2.0f * q1;
	float _2q3 = 2.0f * q2;
	float _2q4 = 2.0f * q3;
	float _2q1q3 = 2.0f * q0 * q2;
	float _2q3q4 = 2.0f * q2 * q3;
	float q1q1 = q0 * q0;
	float q1q2 = q0 * q1;
	float q1q3 = q0 * q2;
	float q1q4 = q0 * q3;
	float q2q2 = q1 * q1;
	float q2q3 = q1 * q2;
	float q2q4 = q1 * q3;
	float q3q3 = q2 * q2;
	float q3q4 = q2 * q3;
	float q4q4 = q3 * q3;

	q4=q3;
	// Normalise accelerometer measurement
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = (float)sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = (float)sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = 1.0f / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * 0.002f;
	q2 += qDot2 * 0.002f;
	q3 += qDot3 * 0.002f;
	q4 += qDot4 * 0.002f;
	norm = 1.0f / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	Quaternion[0] = q1 * norm;
	Quaternion[1] = q2 * norm;
	Quaternion[2] = q3 * norm;
	Quaternion[3] = q4 * norm;
	
	
	angle->pit = safe_asin(-2 * q2 * q4 + 2 * q1* q3)* 57.3 /*- AngleOffset_Pit*/; // pitch
	angle->rol = atan2(2 * q3 * q4 + 2 * q1 * q2, -2 * q2 * q2 - 2 * q3* q3 + 1)* 57.3 /*- AngleOffset_Rol*/; // roll
	angle->yaw = -atan2(2*q2*q3 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动	
	
	
}
		
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,T_float_angle *angle) 
{
        float norm;
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;

        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;   
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;          
			
		gx = gx * 3.14159 / 180.0f;
		gy = gy * 3.14159 / 180.0f;
		gz = gz * 3.14159 / 180.0f;
	
	
        // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
        norm = sqrt(mx*mx + my*my + mz*mz);          
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;         
        
        // compute reference direction of flux
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
        bx = sqrt((hx*hx) + (hy*hy));
        bz = hz;        
        
        // estimated direction of gravity and flux (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
		
        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
        
        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        
        // integral error scaled integral gain
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        // integrate quaternion rate and normalise
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
		
	angle->pit = safe_asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 /*- AngleOffset_Pit*/; // pitch
	angle->rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 /*- AngleOffset_Rol*/; // roll
	angle->yaw = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动	
}


void Gyrupdate(Axis3f *gyr, T_float_angle *angle)
{
	float gx = gyr->x,gy = gyr->y,gz = gyr->z;//单位是度
	float norm;
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	gx = gx * 3.1415 / 180.0f;
	gy = gy * 3.1415 / 180.0f;
	gz = gz * 3.1415 / 180.0f;
	
	//一阶龙格库塔法更新四元数
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//归一化处理
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	angle->pit = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
	angle->rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	angle->yaw = atan2(2*(q1*q2 - q0*q3), 2*(q0*q0 + q1*q1) - 1)*57.3;
}




//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}
#endif
#ifdef RT_USING_FINSH
#include <finsh.h>
static void gyr_offset(void) 
{
	gyro_offset();
}
FINSH_FUNCTION_EXPORT(gyr_offset, gyr offset.)
#endif