#include <rtthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "parameterDef.h"

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

#include "nRF24L01.h"
#include "imu_api.h"

/*
rt_int16_t acc_x,acc_y,acc_z;
rt_int16_t gyr_x,gyr_y,gyr_z;
rt_int16_t mag_x,mag_y,mag_z;
*/

T_int16_xyz g_gyr;
T_int16_xyz g_acc;
T_int16_xyz g_mag;

T_int16_xyz g_gyr_offset;

extern rt_timer_t timer_imu;

//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
//uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

volatile uint_fast8_t g_vui8Flag;
//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        100

uint32_t g_ui32PrintSkipCounter;



//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOe(void)
{
    unsigned long ulStatus;
	//int acc_x,acc_y,acc_z,temp;

    ulStatus = GPIOIntStatus(GPIO_PORTE_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTE_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_3)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
		
			
		/*acc_x=(g_sMPU9150Inst.pui8Data[0]<<8)+g_sMPU9150Inst.pui8Data[1];
		acc_y=(g_sMPU9150Inst.pui8Data[2]<<8)+g_sMPU9150Inst.pui8Data[3];
		acc_z=(g_sMPU9150Inst.pui8Data[4]<<8)+g_sMPU9150Inst.pui8Data[5];
		temp=(g_sMPU9150Inst.pui8Data[6]<<8)|g_sMPU9150Inst.pui8Data[7];
		rt_kprintf("acc_x=%d,acc_y=%d,acc_z=%d,temp=%x\n",acc_x,acc_y,acc_z,temp);*/
	}
}

void imu_timeout_read(void* parameter)
{
	MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
	g_vui8Flag=1;
}

void
IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_5)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
		
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    rt_kprintf("\033[31;1m");
    rt_kprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);
    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
			//rt_kprintf("Do Nothing\n");
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

void MPU9150_Init(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
	
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
	
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE,GPIO_PIN_5);
	I2CIntRegister(I2C2_BASE,MPU9150I2CIntHandler);
	
	ROM_I2CMasterEnable(I2C2_BASE);
	I2CMInit(&g_sI2CInst, I2C2_BASE, INT_I2C2, 0xff, 0xff,ROM_SysCtlClockGet());
	
	//
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);
    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);
    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);
		    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);		

	/*
	//配置为中断输出
	//
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
																		
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);
		

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);*/
		
		
}

void imu(void);
static rt_uint8_t imu_stack[2048];
static struct rt_thread imu_thread;
T_float_angle g_angle;
/*50Hz互补滤波姿态输出*/
void rt_imu_thread_entry(void *parameter)
{
	rt_uint16_t i;
	uint_fast32_t ui32CompDCMStarted;
	float pfData[12];
	float *pfAccel, *pfGyro, *pfMag, *pfEulers;	
	
	pfAccel = pfData;
	pfGyro = pfData + 3;
	pfMag = pfData + 6;
	pfEulers = pfData + 9;
	MPU9150_Init();
	CompDCMInit(&g_sCompDCMInst, 1.0f / 500.0f, 0.02f, 0.96f, 0.02f);
	ui32CompDCMStarted = 0;
	//g_sMPU9150Inst.ui8GyroFsSel=3;//陀螺仪采用2000 deg/s
	rt_thread_delay(10000*10);
	rt_timer_start(timer_imu);
	for(i=0;i<200;i++)
	{
		while(!g_vui8I2CDoneFlag){}
		g_vui8I2CDoneFlag = 0;
		MPU9150DataGyroGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &g_gyr.X,
                                              (uint_fast16_t *) &g_gyr.Y,
                                              (uint_fast16_t *) &g_gyr.Z);
			
		//g_gyr_offset.Z+=g_gyr.Z;
		Prepare_Data(&g_gyr,&g_gyr_offset);
	}
	//g_gyr_offset.Z=g_gyr_offset.Z/200;
	rt_kprintf("g_gyr_offset.Z=%d\n",g_gyr_offset.Z);
	while(1)
    {
		/*等待传感器读取完毕,采用RT-THREAD定时器读取*/
       /* while(!g_vui8I2CDoneFlag){}
        g_vui8I2CDoneFlag = 0;
		*/
		while(!g_vui8Flag){}
        g_vui8I2CDoneFlag = 0;
		/*
		g_acc.X=(g_sMPU9150Inst.pui8Data[0]<<8)+g_sMPU9150Inst.pui8Data[1];
		rt_kprintf("g_acc.X 1=%d\n",g_acc.X);*/	
		/*获取传感器原始数据*/
		MPU9150DataGyroGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &g_gyr.X,
                                              (uint_fast16_t *) &g_gyr.Y,
                                              (uint_fast16_t *) &g_gyr.Z);
		
		MPU9150DataAccelGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &g_acc.X,
                                               (uint_fast16_t *) &g_acc.Y,
                                               (uint_fast16_t *) &g_acc.Z);
			

		MPU9150DataMagnetoGetRaw(&g_sMPU9150Inst,(uint_fast16_t *) &g_mag.X,
                                                 (uint_fast16_t *) &g_mag.Y,
                                                 (uint_fast16_t *) &g_mag.Z);
			
		//rt_kprintf("g_gyr.Z-g_gyr_offset.Z=%d\n",g_gyr.Z-g_gyr_offset.Z);	

		
			
        // Get floating point version of the Accel Data in m/s^2.
        MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,pfAccel + 2);
        
        // Get floating point version of angular velocities in rad/sec     
        MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,pfGyro + 2);
        
        // Get floating point version of magnetic fields strength in tesla
        MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,pfMag + 2);
		
		//rt_kprintf("g_gyr.Z-g_gyr_offset.Z=%d\n",g_gyr.Z-g_gyr_offset.Z);	
		g_angle.yaw += (g_gyr.Z-g_gyr_offset.Z)/131.0*0.002f;
		
		
        // Check if this is our first data ever.
        if(ui32CompDCMStarted == 0)
        {
            // Set flag indicating that DCM is started.
            // Perform the seeding of the DCM with the first data set.
            ui32CompDCMStarted = 1;
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
                                 pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
                              pfGyro[2]);
            CompDCMStart(&g_sCompDCMInst);
        }
        else
        {
            // DCM Is already started.  Perform the incremental update.
            CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],-pfGyro[2]);
            CompDCMUpdate(&g_sCompDCMInst);
        }
		
		//将方向余弦矩阵转换为欧垃圾，单位为弧度
		CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,pfEulers + 2);
		
		// Convert Eulers to degrees. 180/PI = 57.29...
		// Convert Yaw to 0 to 360 to approximate compass headings.
		pfEulers[0] *= 57.295779513082320876798154814105f;
		pfEulers[1] *= 57.295779513082320876798154814105f;
		//pfEulers[2] *= 57.295779513082320876798154814105f;
		
		/*if(pfEulers[2] < 0)
        {
			pfEulers[2] += 360.0f;
		}*/
		
		/*T_int16_xyz gyr_o;
		Prepare_Data(&g_gyr,&gyr_o);
*/
		
		g_angle.rol = pfEulers[0];
		g_angle.pit = pfEulers[1];
		//g_angle.yaw = pfEulers[2];	
			  
    }
}

void imu_thread_creat(void)
{
    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&imu_thread,
                            "imu",
                            rt_imu_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&imu_stack[0],
                            sizeof(imu_stack),
                            18,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&imu_thread);
    }
		
}

#ifdef RT_USING_FINSH
#include <finsh.h>
#include <stdio.h>
void imu(void)
{
	//rt_int16_t temp=0;
	float temp_c=0;
	char buff[20];

	/*MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
	acc_x=(g_sMPU9150Inst.pui8Data[0]<<8)+g_sMPU9150Inst.pui8Data[1];
	acc_y=(g_sMPU9150Inst.pui8Data[2]<<8)+g_sMPU9150Inst.pui8Data[3];
	acc_z=(g_sMPU9150Inst.pui8Data[4]<<8)+g_sMPU9150Inst.pui8Data[5];
	
	gyr_x=(g_sMPU9150Inst.pui8Data[8]<<8)+g_sMPU9150Inst.pui8Data[9];
	gyr_y=(g_sMPU9150Inst.pui8Data[10]<<8)+g_sMPU9150Inst.pui8Data[11];
	gyr_z=(g_sMPU9150Inst.pui8Data[12]<<8)+g_sMPU9150Inst.pui8Data[13];
	
	temp=(g_sMPU9150Inst.pui8Data[6]<<8)|g_sMPU9150Inst.pui8Data[7];
	temp_c=(float) temp/340 + 36.53f;
	
	mag_x=(g_sMPU9150Inst.pui8Data[15]<<8)+g_sMPU9150Inst.pui8Data[16];
	mag_y=(g_sMPU9150Inst.pui8Data[17]<<8)+g_sMPU9150Inst.pui8Data[18];
	mag_z=(g_sMPU9150Inst.pui8Data[19]<<8)+g_sMPU9150Inst.pui8Data[20];*/

	rt_kprintf("acc_x=%d,acc_y=%d,acc_z=%d\n",g_acc.X,g_acc.Y,g_acc.Z);	
	rt_kprintf("gyr_x=%d,gyr_y=%d,gyr_z=%d\n",g_gyr.X,g_gyr.Y,g_gyr.Z);	
	rt_kprintf("mag_x=%d,mag_y=%d,mag_z=%d\n",g_mag.X,g_mag.Y,g_mag.Z);	
	sprintf(buff,"%10f",temp_c);
	rt_kprintf("temp=%sC\n",buff );


}
FINSH_FUNCTION_EXPORT(imu, look for mpu9150 data.)
#endif
