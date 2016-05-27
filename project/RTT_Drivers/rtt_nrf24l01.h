/*
 * File      : 
 * 
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-01-xx     zzzzz      the first version
 */

#ifndef __RTT_NRF24L01_H__
#define __RTT_NRF24L01_H__

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <drivers/spi.h>
#include "driverlib/ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"



//==========================NRF24L01============================================
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width
#define TX_PLOAD_WIDTH  32  	// 32 TX payload
#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload
//=========================NRF24L01�Ĵ���ָ��===================================
#define RF_READ_REG     0x00  	// ���Ĵ���ָ��
#define RF_WRITE_REG    0x20 	// д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP1            0xFF  	// ����
//========================SPI(nRF24L01)�Ĵ�����ַ===============================
#define NRF_READ_REG    0x00  // Define read command to register
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
//��״̬
#define RX_DR           0x40  //���ݽ���RX FIFO���վ����ж�
#define TX_DS           0x20  //���ݷ���TX FIFO���ͳɹ��ж�      
#define MAX_RT          0x10  //����ʹ����ж�      
#define RX_P_NO         0x0e  //���յ������ݵ�ͨ��
#define TX_FULL         0x01  //����TX FIFO��
#define R_RX_PL_WID   	0x60

struct nrf_hardware
{
	rt_uint32_t						ce_port_periph;
	rt_uint32_t						ce_port_base;
	rt_uint32_t						ce_port;
	rt_uint32_t						iqr_port_periph;
	rt_uint32_t						iqr_port_base;
	rt_uint32_t						iqr_port;
};
struct nrf_addr_data
{
	rt_uint8_t							tx_addr[5];
	rt_uint8_t							rx_addr0[5];
	rt_uint8_t							rx_addr1[5];
	rt_uint8_t							rx_addr2[5];
	rt_uint8_t							rx_addr3[5];
	rt_uint8_t							rx_addr4[5];
};
struct nrf_rx_data
{
	rt_uint8_t rx_buffer[33];
	rt_uint32_t read_index, save_index;
};
struct spi_wireless_device
{
    struct rt_device                wireless_device;
    struct rt_spi_device *          rt_spi_device;
    struct rt_mutex                 lock;
	/*nrf24xx pritave data*/
	rt_uint8_t						initNum;
	rt_thread_t                     thread_tid;
	struct nrf_hardware				hardware;
	struct nrf_addr_data			addrdata;
	struct nrf_rx_data              rxdata;
};

extern rt_err_t nrf24_init(const char * wireless_device_name,
                            const char * spi_device_name);


#endif // __RTT_NRF24L01_H__
