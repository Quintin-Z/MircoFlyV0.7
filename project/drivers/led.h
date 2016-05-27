//
// GPIO, Timer, Peripheral, and Pin assignments for the colors
//
#include <rtthread.h>
#include <stdint.h>
#include <stdbool.h>
#define RED_GPIO_PERIPH         SYSCTL_PERIPH_GPIOC
#define BLUE_GPIO_PERIPH        SYSCTL_PERIPH_GPIOC
#define GREEN_GPIO_PERIPH       SYSCTL_PERIPH_GPIOC



#define RED_GPIO_BASE           GPIO_PORTC_BASE
#define BLUE_GPIO_BASE          GPIO_PORTC_BASE
#define GREEN_GPIO_BASE         GPIO_PORTC_BASE


#define RED_GPIO_PIN            GPIO_PIN_4
#define BLUE_GPIO_PIN           GPIO_PIN_5
#define GREEN_GPIO_PIN          GPIO_PIN_6
 void rt_hw_led_init(void);
void rt_hw_led_on(rt_uint32_t n);
void rt_hw_led_off(rt_uint32_t n);
