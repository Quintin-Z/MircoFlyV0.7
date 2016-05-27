#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <board.h>

extern rt_uint8_t battery_adc_initialize(void);
extern rt_uint32_t read_battert_adc_value(void);

#endif //__BATTERY_H__
