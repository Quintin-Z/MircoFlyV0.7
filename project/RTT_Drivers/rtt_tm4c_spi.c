#include "rtt_tm4c_spi.h"
#define USING_SPI0
static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops tm4c_spi_ops =
{
    configure,
    xfer
};

#ifdef USING_SPI0
static struct tm4c_spi_bus tm4c_spi_bus_0;
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI1
static struct tm4c_spi_bus tm4c_spi_bus_1;
#endif /* #ifdef USING_SPI1 */

#ifdef USING_SPI2
static struct tm4c_spi_bus tm4c_spi_bus_2;
#endif /* #ifdef USING_SPI2 */

#ifdef USING_SPI3
static struct tm4c_spi_bus tm4c_spi_bus_3;
#endif /* #ifdef USING_SPI3 */


static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    struct tm4c_spi_bus *tm4c_spi_bus = (struct tm4c_spi_bus *)device->bus;
	
	//rt_kprintf("configure:tm4c_spi_bus->ssi_base=%x\n",tm4c_spi_bus->ssi_base);
	//rt_kprintf("configuration->%d\n",configuration->data_width);
	//rt_kprintf("configuration->max_hz=%d\n",configuration->max_hz);
	//rt_kprintf("configuration->mode=%d\n",configuration->mode);
	//初始化SSI外设
	ROM_SysCtlPeripheralEnable(tm4c_spi_bus->port_periph);//PORT外设使能
	ROM_SysCtlPeripheralEnable(tm4c_spi_bus->ssi_periph);//SSI外设使能
	
	//SPI配置端口配置
	ROM_GPIOPinConfigure(tm4c_spi_bus->ssi_clk_pin_config);//SCK
	//ROM_GPIOPinConfigure(tm4c_spi_bus->ssi_fss_pin_config);//FSS
    ROM_GPIOPinConfigure(tm4c_spi_bus->ssi_rx_pin_config);//MISO
    ROM_GPIOPinConfigure(tm4c_spi_bus->ssi_tx_pin_config);//MOSI
	
	ROM_GPIOPinTypeSSI(tm4c_spi_bus->port_base, tm4c_spi_bus->ssi_clk_port | /*tm4c_spi_bus->ssi_fss_port |*/ 
											   tm4c_spi_bus->ssi_rx_port | tm4c_spi_bus->ssi_tx_port);
	
	GPIOPadConfigSet(tm4c_spi_bus->ssi_base, tm4c_spi_bus->ssi_rx_port, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(tm4c_spi_bus->ssi_base, tm4c_spi_bus->ssi_clk_port | tm4c_spi_bus->ssi_tx_port ,GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

	
	ROM_SSIConfigSetExpClk(tm4c_spi_bus->ssi_base, SysCtlClockGet(), configuration->mode,SSI_MODE_MASTER, configuration->max_hz, configuration->data_width);
	ROM_SSIEnable(tm4c_spi_bus->ssi_base);
	
    return RT_EOK;
};
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
	struct tm4c_spi_bus * tm4c_spi_bus = (struct tm4c_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
   
    struct tm4c_spi_cs * tm4c_spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;
	
	//rt_kprintf("xfer:config->data_width=%d\n",config->data_width);
	
	 if(message->cs_take)
    {
		//rt_kprintf("cs_take\n");
		ROM_GPIOPinWrite(tm4c_spi_cs->cs_port_base, tm4c_spi_cs->cs_port, 0);
		
    }
	
	if(config->data_width <= 8)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint8_t data = 0x00;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }
				uint32_t temp=0;
				//rt_kprintf("tm4c_spi_bus->ssi_base=%x,data=0x%x\n",tm4c_spi_bus->ssi_base,data);
				SSIDataPut(tm4c_spi_bus->ssi_base,data);
				//rt_kprintf("2\n");
				SSIDataGet(tm4c_spi_bus->ssi_base, &temp);
				//rt_kprintf("3\n");
				data = (rt_uint8_t)temp;
                //Wait until the transmit buffer is empty
                //while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
               // SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
                //while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                // Get the received data
                //data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
        else if(config->data_width <= 16)
        {
            const rt_uint16_t * send_ptr = message->send_buf;
            rt_uint16_t * recv_ptr = message->recv_buf;

            while(size--)
            {
                rt_uint16_t data = 0xFFFF;

                if(send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }
				uint32_t temp=0;
				ROM_SSIDataPut(tm4c_spi_bus->ssi_base,data);
				ROM_SSIDataGet(tm4c_spi_bus->ssi_base, &temp);
				data = (rt_uint16_t)temp;	
                //Wait until the transmit buffer is empty
				//while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
                // Send the byte
                //SPI_I2S_SendData(SPI, data);

                //Wait until a data is received
				//while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);
                //Get the received data
				//data = SPI_I2S_ReceiveData(SPI);

                if(recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }
        }
		    /* release CS */
    if(message->cs_release)
    {
		//rt_kprintf("cs_release\n");
       // GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
		ROM_GPIOPinWrite(tm4c_spi_cs->cs_port_base, tm4c_spi_cs->cs_port, tm4c_spi_cs->cs_port);
    }

    return message->length;
};

rt_err_t tm4c_spi_register(uint32_t spi_base,struct tm4c_spi_bus * tm4c_spi,const char * spi_bus_name)
{
	return rt_spi_bus_register(&tm4c_spi->parent, spi_bus_name, &tm4c_spi_ops);
}
//struct rt_spi_device spi0;

void rt_hw_spi_init(void)
{
#if 0
	tm4c_spi_bus_0.port_periph = SYSCTL_PERIPH_GPIOA;
	tm4c_spi_bus_0.port_base = GPIO_PORTA_BASE;
	tm4c_spi_bus_0.ssi_periph = SYSCTL_PERIPH_SSI0;
	tm4c_spi_bus_0.ssi_base = SSI0_BASE;


	tm4c_spi_bus_0.ssi_clk_port = GPIO_PIN_2;
	tm4c_spi_bus_0.ssi_fss_port = GPIO_PIN_3;
	tm4c_spi_bus_0.ssi_rx_port = GPIO_PIN_4;
	tm4c_spi_bus_0.ssi_tx_port = GPIO_PIN_5;
	
	tm4c_spi_bus_0.ssi_clk_pin_config = GPIO_PA2_SSI0CLK;
	tm4c_spi_bus_0.ssi_fss_pin_config = GPIO_PA3_SSI0FSS;
	tm4c_spi_bus_0.ssi_rx_pin_config = GPIO_PA4_SSI0RX;
	tm4c_spi_bus_0.ssi_tx_pin_config =GPIO_PA5_SSI0TX ;
#else
	tm4c_spi_bus_0.port_periph = SYSCTL_PERIPH_GPIOB;
	tm4c_spi_bus_0.port_base = GPIO_PORTB_BASE;
	tm4c_spi_bus_0.ssi_periph = SYSCTL_PERIPH_SSI2;
	tm4c_spi_bus_0.ssi_base = SSI2_BASE;

	tm4c_spi_bus_0.ssi_clk_port = GPIO_PIN_4;
	tm4c_spi_bus_0.ssi_fss_port = GPIO_PIN_5;
	tm4c_spi_bus_0.ssi_rx_port = GPIO_PIN_6;
	tm4c_spi_bus_0.ssi_tx_port = GPIO_PIN_7;
	
	tm4c_spi_bus_0.ssi_clk_pin_config = GPIO_PB4_SSI2CLK;
	tm4c_spi_bus_0.ssi_fss_pin_config = GPIO_PB5_SSI2FSS;
	tm4c_spi_bus_0.ssi_rx_pin_config = GPIO_PB6_SSI2RX;
	tm4c_spi_bus_0.ssi_tx_pin_config =GPIO_PB7_SSI2TX ;
#endif	
	//tm4c_spi_bus_0.parent.parent
	

	
	tm4c_spi_register(SSI2_BASE, &tm4c_spi_bus_0, "ssi0");

	
    /* attach cs */
    {
        static struct rt_spi_device spi_device;
        static struct tm4c_spi_cs  spi_cs;
		
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//PORT外设使能
		//配置CS口为输出；
		
		ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);//使用软件模拟SPI片选CSN
		
		spi_cs.cs_port_base = GPIO_PORTB_BASE;
		spi_cs.cs_port = GPIO_PIN_5;

        rt_spi_bus_attach_device(&spi_device, "ssi00", "ssi0", (void*)&spi_cs);
    }
}
