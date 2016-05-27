#include <rtthread.h>
#include "nRF24L01.h"
//=============================RF24l01״̬=====================================
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]  = {0xa0,0xa1,0xa2,0xa3,0xa4};// ·�ɽڵ��ַ
uint8_t  RX_ADDRESS1[TX_ADR_WIDTH] = {0xa0,0xa1,0xa2,0xa3,0xa4};//д��ַ�Ĵ�������д���ֽڣ���˺��漸���ֽ���ͬ
uint8_t  RX_ADDRESS2[TX_ADR_WIDTH] = {0xa1,0xa1,0xa2,0xa3,0xa4};


//uint8_t  RX_ADDRESS3[TX_ADR_WIDTH] = {0xe2,0xe1,0xe2,0xe3,0xe4};
//uint8_t  RX_ADDRESS4[TX_ADR_WIDTH] = {0xe3,0xe1,0xe2,0xe3,0xe4};
//uint8_t  RX_ADDRESS5[TX_ADR_WIDTH] = {0xe4,0xe1,0xe2,0xe3,0xe4};

uint8_t RX_Number;
uint8_t status_in,status_out;
bool RX_Data_Ready,TX_Data_Success,TX_Data_Failed=1;

void SSI2SetToSPIInit(void)
{
	//SPI���ö˿�����
	ROM_GPIOPinConfigure(GPIO_PB4_SSI2CLK);//SCK
	ROM_GPIOPinTypeGPIOOutput(RF_CSN_BASE, RF_CSN_PORT);//ʹ�����ģ��SPIƬѡCSN
    ROM_GPIOPinConfigure(GPIO_PB6_SSI2RX);//MISO
    ROM_GPIOPinConfigure(GPIO_PB7_SSI2TX);//MOSI
	ROM_GPIOPinTypeSSI(RF_GPIO_PORT_BASE, RF_SSI_TX | RF_SSI_RX | RF_SSI_CLK);
	GPIOPadConfigSet(RF_GPIO_PORT_BASE, RF_SSI_RX, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(RF_GPIO_PORT_BASE, RF_SSI_CLK | RF_SSI_TX | RF_SSI_FSS,GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	ROM_SSIConfigSetExpClk(RF_SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 8000000, 8);
	ROM_SSIEnable(RF_SSI_BASE);
}


//==============================================================================
//������uint SPI_RW(uint uuint8_t)
//���ܣ�NRF24L01��SPIдʱ��
//******************************************************************************
uint8_t SPI_RW(uint8_t data)
{
	uint32_t temp=0;
	ROM_SSIDataPut(RF_SSI_BASE,data);
	ROM_SSIDataGet(RF_SSI_BASE, &temp);
	return (uint8_t)temp;
}
//****************************************************************************************************
//������uuint8_t SPI_Read(uuint8_t reg)
//���ܣ�NRF24L01��SPIʱ��
//****************************************************************************************************
uint8_t SPI_Read(uint8_t reg)
{
	uint8_t reg_val;
	RF24L01_CSN_0;           // CSN low, initialize SPI communication...
	//��д�Ĵ�����ַ���������ǼĴ����ĵ�ַ�������������Ĳ��ǼĴ��������ֵ
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	RF24L01_CSN_1;         // CSN high, terminate SPI communication
	return(reg_val);       // return register value
}
//****************************************************************************************************/
//���ܣ�NRF24L01��д�Ĵ�������
//****************************************************************************************************/
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status1;
	RF24L01_CSN_0;              // CSN low, init SPI transaction
	status1 = SPI_RW(reg);      // select register
	SPI_RW(value);              // ..and write value to it..
	RF24L01_CSN_1;              // CSN high again
	return(status1);            // return nRF24L01 status uuint8_t
}
//****************************************************************************************************/
//������uint SPI_Read_Buf(uuint8_t reg, uuint8_t *pBuf, uuint8_t uuint8_ts)
//����: ���ڶ����ݣ�reg��Ϊ�Ĵ�����ַ��pBuf��Ϊ���������ݵ�ַ��uuint8_ts���������ݵĸ���
//****************************************************************************************************/
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts)
{
	uint8_t status2,uuint8_t_ctr;
	RF24L01_CSN_0;                    		// Set CSN low, init SPI tranaction
	status2 = SPI_RW(reg);       		// Select register to write to and read status uuint8_t
	for(uuint8_t_ctr=0;uuint8_t_ctr<uint8_ts;uuint8_t_ctr++)
		pBuf[uuint8_t_ctr] = SPI_RW(0);    // 
	RF24L01_CSN_1;                           
	return(status2);                    // return nRF24L01 status uuint8_t
}
//*********************************************************************************************************
//������uint SPI_Write_Buf(uuint8_t reg, uuint8_t *pBuf, uuint8_t uuint8_ts)
//����: ����д���ݣ�Ϊ�Ĵ�����ַ��pBuf��Ϊ��д�����ݵ�ַ��uuint8_ts��д�����ݵĸ���
//*********************************************************************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts)
{
	uint8_t status1,uuint8_t_ctr;
	RF24L01_CSN_0;             //SPIʹ��       
	status1 = SPI_RW(reg);   
	for(uuint8_t_ctr=0; uuint8_t_ctr<uint8_ts; uuint8_t_ctr++) //
		SPI_RW(*pBuf++);
	RF24L01_CSN_1;           //�ر�SPI
	return(status1);    		  // 
}
void RF24L01_Init(void)
{
	ROM_SysCtlPeripheralEnable(RF_SSI_SYSCTL_PERIPH);//����ʹ��
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	
	ROM_GPIOPinTypeGPIOOutput(RF_CE_BASE, RF_CE_PORT);//CE����Ϊ���ģʽ
	ROM_GPIOPinTypeGPIOInput(RF_IQR_BASE,RF_IQR_PORT);
	SSI2SetToSPIInit();//��ʼ��SSI0ΪSPI����
}
void RF24L01_ON(void)
{
	RF24L01_CE_0 ;// Chip Enable

	RF24L01_CSN_1;// SPI disable 
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x3f);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x3f);   // ʹ�ܽ���ͨ��0
	SPI_RW_Reg(RF_WRITE_REG + SETUP_RETR, 0x05);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40);         // ѡ����Ƶͨ��0x40
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	RF24L01_CE_1;

}
void RF24L01_OFF(void)
{
	RF24L01_CE_0;
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	RF24L01_CE_1;
	SysCtlDelay(5*(SysCtlClockGet()/ 3000));
	RF24L01_CE_0;
}

/**************************************************
������RX_Mode()

������
    �����������nRF24L01Ϊ����ģʽ���ȴ����շ����豸�����ݰ�

����ģʽ����ѡ�������ͨ������ѡ��ͨ��0~1ʱ����ֱд��ַ
��ѡ��ͨ��2~5ʱ������дͨ��1�ĵ�ַ����Ϊ2~5ͨ����ַֻ��һ���ֽ�
*************************************************/
void RX_Mode(void)
{
	
	RF24L01_CE_0;;	  
	SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);//дRX�ڵ��ַ
	
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40); // Select RF channel 40
	SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�ȣ�32�ֽڳ���
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x0f);
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0x0f);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ

	RF24L01_CE_1; //CEΪ��,�������ģʽ 
	/*
	RF24L01_CE_0;
	SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
    //SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P1, RX_ADDRESS1, TX_ADR_WIDTH); 
    //SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P2, RX_ADDRESS2, TX_ADR_WIDTH);
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40); // Select RF channel 40
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�ȣ�32�ֽڳ���
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P1, TX_PLOAD_WIDTH);//32�ֽڳ���
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P2, TX_PLOAD_WIDTH);//32�ֽڳ��� 
	SPI_RW_Reg(FLUSH_RX,0);
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x07);
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0x0f);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //��״̬�Ĵ���
	RF24L01_CE_1;                                 // ����CE���������豸
*/
}
/**************************************************/

/**************************************************
������TX_Mode()

������
    �����������nRF24L01Ϊ����ģʽ����CE=1��������10us����
	130us���������䣬���ݷ��ͽ����󣬷���ģ���Զ�ת�����
	ģʽ�ȴ�Ӧ���źš�

����ģʽ��Ӧ�����ͨ������ѡ��0ͨ����ѡ������ͨ�����޷����ܵ�Ӧ��λ
**************************************************/
void TX_Mode(void)
{
	RF24L01_CE_0;    
  	SPI_Write_Buf(RF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	SPI_Write_Buf(RF_WRITE_REG+RX_ADDR_P0,TX_ADDRESS,RX_ADR_WIDTH); //����RX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	SPI_RW_Reg(RF_WRITE_REG + EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ�� 
  	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	SPI_RW_Reg(RF_WRITE_REG + SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	SPI_RW_Reg(RF_WRITE_REG + RF_CH,40);       //����RFͨ��Ϊ40
	SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0,RX_PLOAD_WIDTH);//���ý������ݳ���
  	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	SPI_RW_Reg(RF_WRITE_REG + CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	RF24L01_CE_1;
}
//******************************************************************************************************/
//������unsigned uint8_t nRF24L01_RxPacket(unsigned uint8_t* rx_buf)
//���ܣ����ݶ�ȡ�����rx_buf���ջ�������
//******************************************************************************************************/
uint8_t nRF24L01_RxPacket(uint8_t* rx_buf)
{
	uint8_t status;
	status=SPI_Read(NRFRegSTATUS);
	if(status&RX_DR)
	{
		RF24L01_CE_0;
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		RF24L01_CE_1;		 //�ø�CE���������ݷ���
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //��״̬�Ĵ���
		return (RX_Number+1);     //���յ��򷵻�����ͨ�����
	}
	else
	{
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //��״̬�Ĵ���
		return 0;//���򷵻�0����ʾδ���ܵ��µ�����
	}
}


uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
 	uint8_t pin;
	RF24L01_CE_0;
  	SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	RF24L01_CE_1;//��������	
	
	do
	{
		pin=GPIOPinRead(RF_IQR_BASE,RF_IQR_PORT);
	}while(pin&GPIO_PIN_2);//�ȴ��������,���жϿ�
	
	sta=SPI_Read(NRFRegSTATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_RT)//�ﵽ����ط�����
	{
		SPI_RW_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_RT; 
	}
	if(sta&TX_DS)//�������
	{
		return TX_DS;
	}
	return 0xff;//����ԭ����ʧ��
}
//***********************************************************************************************************
//������void nRF24L01_TxPacket(uint8_t * tx_buf)
//���ܣ����� tx_buf������
//**********************************************************************************************************/
bool NRF_TxPacket(uint8_t * tx_buf,uint8_t length)
{
	uint8_t status;
	status=SPI_Read(NRFRegSTATUS);
	if(status&MAX_RT)
	{
		//��������ط�����
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,MAX_RT);   //��״̬�Ĵ���
		return false;
	}
    RF24L01_CE_0;			//StandBy Iģʽ	
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, length); 			 // װ������	
    RF24L01_CE_1;		 //�ø�CE���������ݷ���
	status=SPI_Read(NRFRegSTATUS);
	if(status&TX_DS)
	{
		//���ͳɹ�
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,TX_DS);   //��״̬�Ĵ���
		return true;
	}
	else
	{
		rt_kprintf("FAILED:status=%x\n",status);
		return false;
	}
}

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	
	SPI_Write_Buf(RF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	
	SPI_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	
	for(i=0;i<5;i++)
	{
		//rt_kprintf("buf[%d]=%x\n",i,buf[i]);
		if(buf[i]!=0XA5)
			break;	 							   
	}
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 
void NRF24L01_Init()
{
	RF24L01_Init();
	//RF24L01_ON();
	if(0 == NRF24L01_Check())
	{
		TX_Mode();
		//RX_Mode();
		rt_kprintf("NRF24L01+ initialization successful\n");
		
	}
	else
	{
		rt_kprintf("NRF24L01+ initialization failed\n");
	}
}
	

void rt_hw_rf_init(void)
{
	NRF24L01_Init();
}

#ifdef RT_USING_FINSH
#include <finsh.h>
#include <stdio.h>
//static rt_uint8_t adc_inited = 0;
static void rf_send(uint8_t *buff)
{
	NRF24L01_TxPacket(buff);
}
FINSH_FUNCTION_EXPORT(rf_send, seed data by RF2.4G.)
#endif
