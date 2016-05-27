/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2013 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2014-07-18     ArdaFu       Port to TM4C129X
 * 2014-08-10     ZzQuan       Port to TM4C123X
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"

#define SYS_CLOCK_DEFAULT 80000000

#define FAULT_NMI               2           // NMI fault
#define FAULT_HARD              3           // Hard fault
#define FAULT_MPU               4           // MPU fault
#define FAULT_BUS               5           // Bus fault
#define FAULT_USAGE             6           // Usage fault
#define FAULT_SVCALL            11          // SVCall
#define FAULT_DEBUG             12          // Debug monitor
#define FAULT_PENDSV            14          // PendSV
#define FAULT_SYSTICK           15          // System Tick

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

extern void PendSV_Handler(void);
extern void HardFault_Handler(void);

/**
 * This function will initial LPC40xx board.
 */
void rt_hw_board_init()
{
    ROM_IntMasterDisable();
    IntRegister(FAULT_HARD, HardFault_Handler); 
    IntRegister(FAULT_PENDSV, PendSV_Handler);
    IntRegister(FAULT_SYSTICK, SysTick_Handler);

	//
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
	ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
    
    // set sysclock to 80M
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    /* init systick */
    ROM_SysTickDisable();
    ROM_SysTickPeriodSet(SysCtlClockGet()/RT_TICK_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();
    
    /* init console */
	rt_hw_uart_init();
    //redirect RTT stdio to CONSOLE device
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();
}
/* init console to support rt_kprintf */
