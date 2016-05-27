#ifndef __TRANSFER_PACKAGE_API_H__
#define __TRANSFER_PACKAGE_API_H__
#include <stdio.h>
#include <rtthread.h>
#include "parameterDef.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void data_receive_anl(rt_uint8_t *data_buf,rt_uint8_t num);
rt_uint8_t data_status_package(T_RC_Status *data,rt_uint8_t data_to_send[]);
rt_uint8_t data_sensor_package(T_RC_Sensor *data,rt_uint8_t data_to_send[]);
rt_uint8_t data_voltage_package(T_RC_Voltage *data,rt_uint8_t data_to_send[]);
rt_uint8_t data_acc_anl_package(T_RC_Acc_Anl *data,rt_uint8_t data_to_send[]);
#endif//__TRANSFER_PACKAGE_API_H__
