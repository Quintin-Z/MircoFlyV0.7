#ifndef PWM_OUT_H
#define PWM_OUT_H

#include <stdint.h>

#define Moto_PwmMax 2000

#define Moto_PwmMin 5
#define PWM_IOCTL_STOP 0x01
#define PWM_IOCTL_SET_FREQ 0x02
void Tim_Pwm_Out_Init(void);
void Moto_PwmRflash(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM);

#endif
