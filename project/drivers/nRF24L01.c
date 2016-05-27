#include <rtthread.h>
#include "nRF24L01.h"
//=============================RF24l01状态=====================================
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]  = {0xa0,0xa1,0xa2,0xa3,0xa4};// 路由节点地址
uint8_t  RX_ADDRESS1[TX_ADR_WIDTH] = {0xa0,0xa1,0xa2,0xa3,0xa4};//写地址寄存器是先写低字节，因此后面几个字节相同
uint8_t  RX_ADDRESS2[TX_ADR_WIDTH] = {0xa1,0xa1,0xa2,0xa3,0xa4};


//uint8_t  RX_ADDRESS3[TX_ADR_WIDTH] = {0xe2,0xe1,0xe2,0xe3,0xe4};
//uint8_t  RX_ADDRESS4[TX_ADR_WIDTH] = {0xe3,0xe1,0xe2,0xe3,0xe4};
//uint8_t  RX_ADDRESS5[TX_ADR_WIDTH] = {0xe4,0xe1,0xe2,0xe3,0xe4};

uint8_t RX_Number;
uint8_t status_in,status_out;
bool RX_Data_Ready,TX_Data_Success,TX_Data_Failed=1;

void SSI2SetToSPIInit(void)
{
	//SPI配置端口配置
	ROM_GPIOPinConfigure(GPIO_PB4_SSI2CLK);//SCK
	ROM_GPIOPinTypeGPIOOutput(RF_CSN_BASE, RF_CSN_PORT);//使用软件模拟SPI片选CSN
    ROM_GPIOPinConfigure(GPIO_PB6_SSI2RX);//MISO
    ROM_GPIOPinConfigure(GPIO_PB7_SSI2TX);//MOSI
	ROM_GPIOPinTypeSSI(RF_GPIO_PORT_BASE, RF_SSI_TX | RF_SSI_RX | RF_SSI_CLK);
	GPIOPadConfigSet(RF_GPIO_PORT_BASE, RF_SSI_RX, GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(RF_GPIO_PORT_BASE, RF_SSI_CLK | RF_SSI_TX | RF_SSI_FSS,GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	ROM_SSIConfigSetExpClk(RF_SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 8000000, 8);
	ROM_SSIEnable(RF_SSI_BASE);
}


//==============================================================================
//函数：uint SPI_RW(uint uuint8_t)
//功能：NRF24L01的SPI写时序
//******************************************************************************
uint8_t SPI_RW(uint8_t data)
{
	uint32_t temp=0;
	ROM_SSIDataPut(RF_SSI_BASE,data);
	ROM_SSIDataGet(RF_SSI_BASE, &temp);
	return (uint8_t)temp;
}
//****************************************************************************************************
//函数：uuint8_t SPI_Read(uuint8_t reg)
//功能：NRF24L01的SPI时序
//****************************************************************************************************
uint8_t SPI_Read(uint8_t reg)
{
	uint8_t reg_val;
	RF24L01_CSN_0;           // CSN low, initialize SPI communication...
	//先写寄存器地址，读到的是寄存器的地址，接下来读到的才是寄存器里面的值
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	RF24L01_CSN_1;         // CSN high, terminate SPI communication
	return(reg_val);       // return register value
}
//****************************************************************************************************/
//功能：NRF24L01读写寄存器函数
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
//函数：uint SPI_Read_Buf(uuint8_t reg, uuint8_t *pBuf, uuint8_t uuint8_ts)
//功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uuint8_ts：读出数据的个数
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
//函数：uint SPI_Write_Buf(uuint8_t reg, uuint8_t *pBuf, uuint8_t uuint8_ts)
//功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uuint8_ts：写入数据的个数
//*********************************************************************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts)
{
	uint8_t status1,uuint8_t_ctr;
	RF24L01_CSN_0;             //SPI使能       
	status1 = SPI_RW(reg);   
	for(uuint8_t_ctr=0; uuint8_t_ctr<uint8_ts; uuint8_t_ctr++) //
		SPI_RW(*pBuf++);
	RF24L01_CSN_1;           //关闭SPI
	return(status1);    		  // 
}
void RF24L01_Init(void)
{
	ROM_SysCtlPeripheralEnable(RF_SSI_SYSCTL_PERIPH);//外设使能
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	
	ROM_GPIOPinTypeGPIOOutput(RF_CE_BASE, RF_CE_PORT);//CE配置为输出模式
	ROM_GPIOPinTypeGPIOInput(RF_IQR_BASE,RF_IQR_PORT);
	SSI2SetToSPIInit();//初始化SSI0为SPI功能
}
void RF24L01_ON(void)
{
	RF24L01_CE_0 ;// Chip Enable

	RF24L01_CSN_1;// SPI disable 
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x3f);       // 使能接收通道0自动应答
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x3f);   // 使能接收通道0
	SPI_RW_Reg(RF_WRITE_REG + SETUP_RETR, 0x05);  // 自动重发延时等待250us+86us，自动重发10次
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40);         // 选择射频通道0x40
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	RF24L01_CE_1;

}
void RF24L01_OFF(void)
{
	RF24L01_CE_0;
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0);              // CRC使能，16位CRC校验，上电，接收模式
	RF24L01_CE_1;
	SysCtlDelay(5*(SysCtlClockGet()/ 3000));
	RF24L01_CE_0;
}

/**************************************************
函数：RX_Mode()

描述：
    这个函数设置nRF24L01为接收模式，等待接收发送设备的数据包

接收模式可以选择任意的通道，当选择通道0~1时可以直写地址
当选择通道2~5时还必须写通道1的地址，因为2~5通道地址只有一个字节
*************************************************/
void RX_Mode(void)
{
	
	RF24L01_CE_0;;	  
	SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);//写RX节点地址
	
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40); // Select RF channel 40
	SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// 接收通道0选择和发送通道相同有效数据宽度，32字节长度
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x0f);
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0x0f);      // CRC使能，16位CRC校验，上电，接收模式

	RF24L01_CE_1; //CE为高,进入接收模式 
	/*
	RF24L01_CE_0;
	SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
    //SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P1, RX_ADDRESS1, TX_ADR_WIDTH); 
    //SPI_Write_Buf(RF_WRITE_REG + RX_ADDR_P2, RX_ADDRESS2, TX_ADR_WIDTH);
	SPI_RW_Reg(RF_WRITE_REG + EN_AA, 0x01); // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR, 0x01); // Enable Pipe0
	SPI_RW_Reg(RF_WRITE_REG + RF_CH, 40); // Select RF channel 40
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);// 接收通道0选择和发送通道相同有效数据宽度，32字节长度
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P1, TX_PLOAD_WIDTH);//32字节长度
    SPI_RW_Reg(RF_WRITE_REG + RX_PW_P2, TX_PLOAD_WIDTH);//32字节长度 
	SPI_RW_Reg(FLUSH_RX,0);
	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP, 0x07);
	SPI_RW_Reg(RF_WRITE_REG + CONFIG, 0x0f);      // CRC使能，16位CRC校验，上电，接收模式
	SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //清状态寄存器
	RF24L01_CE_1;                                 // 拉高CE启动接收设备
*/
}
/**************************************************/

/**************************************************
函数：TX_Mode()

描述：
    这个函数设置nRF24L01为发送模式，（CE=1持续至少10us），
	130us后启动发射，数据发送结束后，发送模块自动转入接收
	模式等待应答信号。

发送模式的应答接收通道必须选择0通道，选择其它通道将无法接受到应答位
**************************************************/
void TX_Mode(void)
{
	RF24L01_CE_0;    
  	SPI_Write_Buf(RF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	SPI_Write_Buf(RF_WRITE_REG+RX_ADDR_P0,TX_ADDRESS,RX_ADR_WIDTH); //设置RX节点地址,主要为了使能ACK	  
  	SPI_RW_Reg(RF_WRITE_REG + EN_AA,0x01);     //使能通道0的自动应答 
  	SPI_RW_Reg(RF_WRITE_REG + EN_RXADDR,0x01); //使能通道0的接收地址  
  	SPI_RW_Reg(RF_WRITE_REG + SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	SPI_RW_Reg(RF_WRITE_REG + RF_CH,40);       //设置RF通道为40
	SPI_RW_Reg(RF_WRITE_REG + RX_PW_P0,RX_PLOAD_WIDTH);//设置接收数据长度
  	SPI_RW_Reg(RF_WRITE_REG + RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	SPI_RW_Reg(RF_WRITE_REG + CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	RF24L01_CE_1;
}
//******************************************************************************************************/
//函数：unsigned uint8_t nRF24L01_RxPacket(unsigned uint8_t* rx_buf)
//功能：数据读取后放如rx_buf接收缓冲区中
//******************************************************************************************************/
uint8_t nRF24L01_RxPacket(uint8_t* rx_buf)
{
	uint8_t status;
	status=SPI_Read(NRFRegSTATUS);
	if(status&RX_DR)
	{
		RF24L01_CE_0;
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
		RF24L01_CE_1;		 //置高CE，激发数据发送
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //清状态寄存器
		return (RX_Number+1);     //接收到则返回数据通道编号
	}
	else
	{
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,0x70);   //清状态寄存器
		return 0;//否则返回0，表示未接受到新的数据
	}
}


uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
 	uint8_t pin;
	RF24L01_CE_0;
  	SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	RF24L01_CE_1;//启动发送	
	
	do
	{
		pin=GPIOPinRead(RF_IQR_BASE,RF_IQR_PORT);
	}while(pin&GPIO_PIN_2);//等待发送完成,读中断口
	
	sta=SPI_Read(NRFRegSTATUS);  //读取状态寄存器的值	   
	SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_RT)//达到最大重发次数
	{
		SPI_RW_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_RT; 
	}
	if(sta&TX_DS)//发送完成
	{
		return TX_DS;
	}
	return 0xff;//其他原因发送失败
}
//***********************************************************************************************************
//函数：void nRF24L01_TxPacket(uint8_t * tx_buf)
//功能：发送 tx_buf中数据
//**********************************************************************************************************/
bool NRF_TxPacket(uint8_t * tx_buf,uint8_t length)
{
	uint8_t status;
	status=SPI_Read(NRFRegSTATUS);
	if(status&MAX_RT)
	{
		//超过最大重发次数
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,MAX_RT);   //清状态寄存器
		return false;
	}
    RF24L01_CE_0;			//StandBy I模式	
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, length); 			 // 装载数据	
    RF24L01_CE_1;		 //置高CE，激发数据发送
	status=SPI_Read(NRFRegSTATUS);
	if(status&TX_DS)
	{
		//发送成功
		SPI_RW_Reg(RF_WRITE_REG+NRFRegSTATUS,TX_DS);   //清状态寄存器
		return true;
	}
	else
	{
		rt_kprintf("FAILED:status=%x\n",status);
		return false;
	}
}

//检测24L01是否存在
//返回值:0，成功;1，失败	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	
	SPI_Write_Buf(RF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	
	SPI_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	
	for(i=0;i<5;i++)
	{
		//rt_kprintf("buf[%d]=%x\n",i,buf[i]);
		if(buf[i]!=0XA5)
			break;	 							   
	}
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
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
