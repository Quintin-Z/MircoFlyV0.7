/*
 * File      : rtt_nrf24l01.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-xx     Zzzzzz      first version.
 */
#include <stdint.h>
#include "rtt_nrf24l01.h"

static struct spi_wireless_device  spi_wireless_device;


/*nrf24l01 api interface*/

static void wireless_lock(struct spi_wireless_device * wireless_device)
{
    rt_mutex_take(&wireless_device->lock, RT_WAITING_FOREVER);
}
static void wireless_unlock(struct spi_wireless_device * wireless_device)
{
    rt_mutex_release(&wireless_device->lock);
}
static void nrf24l01_take_ce(struct spi_wireless_device *wireless_device)
{
	ROM_GPIOPinWrite(wireless_device->hardware.ce_port_base,wireless_device->hardware.ce_port,0);
}
static void nrf24l01_relase_ce(struct spi_wireless_device *wireless_device)
{
	ROM_GPIOPinWrite(wireless_device->hardware.ce_port_base,wireless_device->hardware.ce_port,wireless_device->hardware.ce_port);
}

static uint8_t nrf24l01_read_iqr(struct spi_wireless_device *wireless_device)
{
	uint8_t value;
	value = ROM_GPIOPinRead(wireless_device->hardware.iqr_port_base,wireless_device->hardware.iqr_port);
	return value;
}

static uint8_t nrf24l01_read_reg(struct spi_wireless_device *wireless_device,uint8_t reg)
{
	uint8_t value;
	rt_spi_send_then_recv(wireless_device->rt_spi_device,&reg,1,&value,1);
	
	return value;
}
static uint8_t nrf24l01_write_reg(struct spi_wireless_device *wireless_device,uint8_t reg, uint8_t value)
{
	uint8_t recv_status;
	
	rt_spi_send_then_send(wireless_device->rt_spi_device,&reg,1,&value,1);

	return recv_status;
}

static uint8_t nrf24l01_Read_Buf(struct spi_wireless_device *wireless_device,uint8_t reg, uint8_t *pBuf, uint8_t _ts)
{
	uint8_t recv_status;

	rt_spi_send_then_recv(wireless_device->rt_spi_device,&reg,1,pBuf,_ts);

	return(recv_status);                    
}

static uint8_t nrf24l01_Write_Buf(struct spi_wireless_device *wireless_device,uint8_t reg, uint8_t *pBuf, uint8_t _ts)
{
	uint8_t recv_status;

	rt_spi_send_then_send(wireless_device->rt_spi_device,&reg,1,pBuf,_ts);
	

	return(recv_status);
}
/*
static void rt_nrf24l01_rx_mode(struct spi_wireless_device *wireless_device)
{
	nrf24l01_take_ce(wireless_device);	 
	
	nrf24l01_Write_Buf(wireless_device,RF_WRITE_REG + RX_ADDR_P0,wireless_device->addrdata.rx_addr0,
	                   sizeof(wireless_device->hardware.ce_port));
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + EN_AA,0x01);
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RF_CH, 40); // Select RF channel 40
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�ȣ�32�ֽڳ���
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RF_SETUP, 0x0f);
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + CONFIG, 0x0f);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	nrf24l01_relase_ce(wireless_device);

}
*/
void rt_nrf24l01_init(struct spi_wireless_device *wireless_device,rt_uint8_t mode,rt_uint8_t ch)
{
	nrf24l01_take_ce(wireless_device);
	nrf24l01_Write_Buf(wireless_device,RF_WRITE_REG + TX_ADDR,wireless_device->addrdata.tx_addr,
	                   sizeof(wireless_device->addrdata.tx_addr));
	
	nrf24l01_Write_Buf(wireless_device,RF_WRITE_REG + RX_ADDR_P0,wireless_device->addrdata.rx_addr0,
	                   sizeof(wireless_device->addrdata.rx_addr0));
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ�� 
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RF_CH,ch);       //����RFͨ��Ϊ40
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RX_PW_P0,RX_PLOAD_WIDTH);//���ý������ݳ���
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
	
	if(mode == 1)//RX
	{	
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�ȣ�32�ֽڳ���
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + CONFIG, 0x0f);      //IRQ�շ�����жϿ���,16λCRC,������

	}else if(mode == 2) //TX
	{
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�ȣ�32�ֽڳ���
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	}else if(mode==3)		//RX2
	{
		nrf24l01_write_reg(wireless_device,FLUSH_TX,0xff);
		nrf24l01_write_reg(wireless_device,FLUSH_RX,0xff);
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
			
		nrf24l01_read_reg(wireless_device,0x50);
		nrf24l01_read_reg(wireless_device,0x73);
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+0x1c,0x01);
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+0x1d,0x06);
		
	}else//TX2
	{
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
		nrf24l01_write_reg(wireless_device,FLUSH_TX,0xff);
		nrf24l01_write_reg(wireless_device,FLUSH_RX,0xff);
			
		nrf24l01_read_reg(wireless_device,0x50);
		nrf24l01_read_reg(wireless_device,0x73);
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+0x1c,0x01);
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+0x1d,0x06);		
	}
	nrf24l01_relase_ce(wireless_device);	
}
#if 1
bool rt_nrf24l01_tx_packet(struct spi_wireless_device *wireless_device,uint8_t * tx_buf,uint8_t length)
{
	uint8_t sta;
 	uint8_t pin;
	nrf24l01_take_ce(wireless_device);	
  	//nrf24l01_Write_Buf(wireless_device,WR_TX_PLOAD, tx_buf, length);//д���ݵ�TX BUF  length���ֽ�
	nrf24l01_Write_Buf(wireless_device,0xa8, tx_buf, length);//д���ݵ�TX BUF  length���ֽ�
 	nrf24l01_relase_ce(wireless_device);//��������	
	/*
	do
	{
		pin=nrf24l01_read_iqr(wireless_device);
	}while(pin&wireless_device->hardware.iqr_port);//�ȴ��������,���жϿ�
	
	sta=nrf24l01_read_reg(wireless_device,NRFRegSTATUS); //��ȡ״̬�Ĵ�����ֵ	
	nrf24l01_write_reg(wireless_device,RF_WRITE_REG+NRFRegSTATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_RT)//�ﵽ����ط�����
	{
		nrf24l01_write_reg(wireless_device,FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_RT; 
	}
	if(sta&TX_DS)//�������
	{
		return TX_DS;
	}
	return 0xff;//����ԭ����ʧ��*/
	return 1;
}
#else
bool rt_nrf24l01_tx_packet(struct spi_wireless_device *wireless_device,uint8_t * tx_buf,uint8_t length)
{
	uint8_t status;
	status=nrf24l01_read_reg(wireless_device,NRFRegSTATUS);
	if(status&MAX_RT)
	{
		//��������ط�����
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+NRFRegSTATUS,MAX_RT);   //��״̬�Ĵ���
		return false;
	}
	
	nrf24l01_take_ce(wireless_device);			//StandBy Iģʽ	
    nrf24l01_Write_Buf(wireless_device,WR_TX_PLOAD, tx_buf, length); 			 // װ������	
	nrf24l01_relase_ce(wireless_device);		 //�ø�CE���������ݷ���
	
	status=nrf24l01_read_reg(wireless_device,NRFRegSTATUS);
	if(status&TX_DS)
	{
		//���ͳɹ�
		nrf24l01_write_reg(wireless_device,RF_WRITE_REG+NRFRegSTATUS,TX_DS);   //��״̬�Ĵ���
		return true;
	}
	else
	{
		rt_kprintf("rt_nrf24l01_tx_packet failed:status=0x%x\n",status);
		return false;	
	}
}
#endif

uint8_t rt_nrf24l01_check_event(struct spi_wireless_device *wireless_device)
{
	rt_uint8_t sta = nrf24l01_read_reg(wireless_device,NRF_READ_REG + NRFRegSTATUS);

	if(sta & (0x01 << 6))
	{
		rt_uint8_t rx_len = nrf24l01_read_reg(wireless_device,R_RX_PL_WID);
		if(rx_len<33)
		{
			nrf24l01_Read_Buf(wireless_device,RD_RX_PLOAD,wireless_device->rxdata.rx_buffer,rx_len);// read receive payload from RX_FIFO buffer
			wireless_device->rxdata.save_index=rx_len;
			wireless_device->wireless_device.rx_indicate(&wireless_device->wireless_device,rx_len);
			
			/*rt_kprintf("drv:");
			for(rt_uint8_t i=0;i<rx_len;i++)
			{
				rt_kprintf("0x%x ",wireless_device->rxdata.rx_buffer[i]);
			}
			
			rt_kprintf("\n");*/
		}
		else 
		{
			nrf24l01_write_reg(wireless_device,FLUSH_RX,0xff);//��ջ�����
		}
	}

	if(sta & (1<<5))
	{
		//LED1_ONOFF();
	}

	if(sta & (1<<4))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			nrf24l01_write_reg(wireless_device,FLUSH_TX,0xff);
		}
	}

	nrf24l01_write_reg(wireless_device,RF_WRITE_REG + NRFRegSTATUS, sta);
	return 0x00;
}



//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t rt_nrf24l01_check(struct spi_wireless_device *wireless_device)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	
	wireless_lock(wireless_device);
	nrf24l01_Write_Buf(wireless_device,RF_WRITE_REG+TX_ADDR,buf,5);
	nrf24l01_Read_Buf(wireless_device,TX_ADDR,buf,5); 
	for(i=0;i<5;i++)
	{
		//rt_kprintf("buf[%d]=%x\n",i,buf[i]);
		if(buf[i]!=0XA5)
			break;	 							   
	}
	wireless_unlock(wireless_device);
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	



/* RT-Thread device interface */
static rt_err_t nrf24l01_wireless_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t nrf24l01_wireless_open(rt_device_t dev, rt_uint16_t oflag)
{
	struct spi_wireless_device * nrf24xx = (struct spi_wireless_device *)dev;

	if (nrf24xx->thread_tid != RT_NULL) 
		{
			rt_thread_startup(nrf24xx->thread_tid);
			return RT_EOK;
		}
}

static rt_err_t nrf24l01_wireless_close(rt_device_t dev)
{	
	struct spi_wireless_device * nrf24xx = (struct spi_wireless_device *)dev;
	
	if (nrf24xx->thread_tid != RT_NULL) 
		rt_thread_suspend(nrf24xx->thread_tid);
    return RT_EOK;
	
}

static rt_err_t nrf24l01_wireless_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    return RT_EOK;
}

static rt_size_t nrf24l01_wireless_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
	rt_uint8_t* ptr;
	rt_err_t err_code;
	struct spi_wireless_device* nrf;
	struct nrf_rx_data* rx_data;
	
	ptr = buffer;
	err_code = RT_EOK;
	nrf = (struct spi_wireless_device*)dev;
	rx_data = &nrf->rxdata;
	
	if (dev->flag & RT_DEVICE_FLAG_RDWR)
	{
		wireless_lock(&spi_wireless_device);
		
		for(rt_uint8_t i=0;i<size;i++)
		{
			*ptr++ = rx_data->rx_buffer[i];
			wireless_unlock(&spi_wireless_device);
		}
		
	}else
	{
		return RT_ERROR;
	}
	rt_set_errno(err_code);
	return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_size_t nrf24l01_wireless_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
	struct spi_wireless_device *nrf = (struct spi_wireless_device *)dev;
	//wireless_lock(&spi_wireless_device);//����
	if(rt_nrf24l01_tx_packet(nrf,(uint8_t *)buffer,size) == true)
	{
		//rt_nrf24l01_rx_mode(&spi_wireless_device);
		wireless_unlock(&spi_wireless_device);//����
		return size;
	}
	else
	{
		//rt_nrf24l01_rx_mode(&spi_wireless_device);
		wireless_unlock(&spi_wireless_device);//����
		return 0;
	}
	
}

static void rf_thread_entry(void *parameter)
{
	rt_kprintf("rf poll thread start.\n");
	for(;;)
	{
		rt_nrf24l01_check_event(&spi_wireless_device);
		rt_thread_delay(1);
	}
}

/*
static void rf_check_timeout(void *parameter)
{
	rt_nrf24l01_check_event(&spi_wireless_device);
	
}*/
rt_err_t nrf24_init(const char * wireless_device_name,const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
	spi_wireless_device.initNum = 1;

    /* initialize mutex */
    if (rt_mutex_init(&spi_wireless_device.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init wireless lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!!\n", spi_device_name);
        return -RT_ENOSYS;
    }
	
    spi_wireless_device.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = 0;
        cfg.max_hz = 8000000; /* 8M */
        rt_spi_configure(spi_wireless_device.rt_spi_device, &cfg);
    }
	// config ce port
	{
		spi_wireless_device.hardware.ce_port_periph = SYSCTL_PERIPH_GPIOB;
		spi_wireless_device.hardware.ce_port_base = GPIO_PORTB_BASE;
		spi_wireless_device.hardware.ce_port = GPIO_PIN_3;
		ROM_SysCtlPeripheralEnable(spi_wireless_device.hardware.ce_port_periph);
		ROM_GPIOPinTypeGPIOOutput(spi_wireless_device.hardware.ce_port_base,spi_wireless_device.hardware.ce_port);
		
	}
	// config irq port
	{
		spi_wireless_device.hardware.iqr_port_periph = SYSCTL_PERIPH_GPIOB;
		spi_wireless_device.hardware.iqr_port_base = GPIO_PORTB_BASE;
		spi_wireless_device.hardware.iqr_port = GPIO_PIN_2;
		ROM_SysCtlPeripheralEnable(spi_wireless_device.hardware.iqr_port_periph);
		ROM_GPIOPinTypeGPIOInput(spi_wireless_device.hardware.iqr_port_base,spi_wireless_device.hardware.iqr_port);
	}
    /* init wireless mod */
	{
		
		spi_wireless_device.addrdata.rx_addr0[0]=0xaa;
		spi_wireless_device.addrdata.rx_addr0[1]=0xbb;
		spi_wireless_device.addrdata.rx_addr0[2]=0xcc;
		spi_wireless_device.addrdata.rx_addr0[3]=0x00;
		spi_wireless_device.addrdata.rx_addr0[4]=0x01;
		
		spi_wireless_device.addrdata.tx_addr[0]=0xaa;
		spi_wireless_device.addrdata.tx_addr[1]=0xbb;
		spi_wireless_device.addrdata.tx_addr[2]=0xcc;
		spi_wireless_device.addrdata.tx_addr[3]=0x00;
		spi_wireless_device.addrdata.tx_addr[4]=0x01;
		//SysCtlDelay(SysCtlClockGet()*2);//��ʱ1S
		
		do
		{	rt_kprintf("The %d time check\n",spi_wireless_device.initNum++);
			//spi_wireless_device.initNum++;
			if(spi_wireless_device.initNum ==10)
			{
				rt_kprintf("nrf24l01 mod check time out!\n");
				return RT_ERROR;
			}
		}while(rt_nrf24l01_check(&spi_wireless_device)!=0);
		rt_kprintf("rt_nrf24l01_check successful.\n");
					
		rt_nrf24l01_init(&spi_wireless_device,4,80);
			
		/*
		//�������ģ���Ƿ����
		if(rt_nrf24l01_check(&spi_wireless_device)==0)
		{
			
			rt_nrf24l01_init(&spi_wireless_device,4,80);
			rt_kprintf("rt_nrf24l01_check successful.\n");	
			

		}else
		{
			rt_kprintf("can not found rt_nrf24l01.\n");
			return RT_ERROR;
		}*/
    }
	
	
	{
		//������ʱ��,10ms���һ��
		//spi_wireless_device.timer=rt_timer_create("rf", rf_check_timeout, RT_NULL,10, RT_TIMER_FLAG_PERIODIC);
		//������open�ӿڵ�ʱ��������ʱ��

		spi_wireless_device.thread_tid = rt_thread_create("rfPoll",
						  rf_thread_entry,
                          RT_NULL,
						  512,
                          7,
                          20);
		//if (spi_wireless_device.thread_tid != RT_NULL) 
		//	rt_thread_startup(spi_wireless_device.thread_tid);
	}
    /* register device */
    spi_wireless_device.wireless_device.type    = RT_Device_Class_SPIDevice;
    spi_wireless_device.wireless_device.init    = nrf24l01_wireless_init;
    spi_wireless_device.wireless_device.open    = nrf24l01_wireless_open;
    spi_wireless_device.wireless_device.close   = nrf24l01_wireless_close;
    spi_wireless_device.wireless_device.read    = nrf24l01_wireless_read;
    spi_wireless_device.wireless_device.write   = nrf24l01_wireless_write;
    spi_wireless_device.wireless_device.control = nrf24l01_wireless_control;
    /* no private */
    spi_wireless_device.wireless_device.tx_complete = RT_NULL;

    rt_device_register(&spi_wireless_device.wireless_device, wireless_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
//INIT_BOARD_EXPORT();
