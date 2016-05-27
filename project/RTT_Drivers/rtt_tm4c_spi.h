#ifndef __RTT_TM4C_SPI_H__
#define __RTT_TM4C_SPI_H__

#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

struct tm4c_spi_bus
{
    struct rt_spi_bus parent;
	uint32_t port_periph;
	uint32_t ssi_periph;
	uint32_t port_base;
	uint32_t ssi_base;
	uint8_t  ssi_clk_port;
	uint8_t  ssi_fss_port;
	uint8_t  ssi_rx_port;
	uint8_t  ssi_tx_port;
	uint32_t ssi_clk_pin_config;
	uint32_t ssi_fss_pin_config;
	uint32_t ssi_rx_pin_config;
	uint32_t ssi_tx_pin_config;
	
#ifdef SPI_USE_DMA
    DMA_Channel_TypeDef * DMA_Channel_TX;
    DMA_Channel_TypeDef * DMA_Channel_RX;
    uint32_t DMA_Channel_TX_FLAG_TC;
    uint32_t DMA_Channel_TX_FLAG_TE;
    uint32_t DMA_Channel_RX_FLAG_TC;
    uint32_t DMA_Channel_RX_FLAG_TE;
#endif /* SPI_USE_DMA */
};
struct tm4c_spi_cs
{
    uint32_t cs_port_base;
    uint8_t cs_port;
};


void rt_hw_spi_init(void);
#endif //__RTT_TM4C_SPI_H__
