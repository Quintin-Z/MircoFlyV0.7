#include "drv_eeprom.h"

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <driverlib/rom.h>
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"
void eerprom_init(void)
{
	uint32_t status;
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	status = EEPROMInit();
	
	if(status == EEPROM_INIT_OK)
	{
		rt_kprintf("eeprom init OK.\n");//eeprom初始化成功
	}
	else if(status == EEPROM_INIT_RETRY)
	{
		rt_kprintf("eeprom init RETRY.\n");//前一个eeprom之前操作被打断
	}
	else if(status == EEPROM_INIT_ERROR)
	{
		rt_kprintf("eeprom init ERROR.\n");//eeprom初始化失败
	}
	status = EEPROMSizeGet();
	rt_kprintf("EEPROMSize = %d byte.\n",status);
	
	
	//eepromBuff[0]=0x12345678;
	//eepromAddress = EEPROMAddrFromBlock(i);
	//EEPROMProgram(eepromBuff, eepromAddress, 4);
	

	
	
}

int8_t eeprom_read_byte(uint32_t *pfeepromBuff,uint32_t startAddress,uint32_t byteCount)
{
	//判读传入的参数合法
	
	
	EEPROMRead(pfeepromBuff,startAddress,byteCount);
	return -1;
}

int8_t eeprom_read_block(uint32_t *pfeepromBuff,uint8_t startBlock,uint8_t blockCount)
{
	uint8_t i;
	uint32_t eepromAddress;
	if(blockCount<=0)
	{
		return 0;
	}
	else
	{
		for(i=startBlock; i<(startBlock+blockCount); i++)
		{
			eepromAddress = EEPROMAddrFromBlock(i);
			EEPROMRead(pfeepromBuff,eepromAddress,64);
			pfeepromBuff +=16;
			
		}
		return -1;
	}
}

uint32_t eeprom_write_byte(uint32_t *pfeepromBuff,uint32_t startAddress,uint32_t byteCount)
{
	return EEPROMProgram(pfeepromBuff,startAddress,byteCount);
}


#ifdef RT_USING_FINSH
#include <finsh.h>
static void read_eeprom(uint8_t startBlock,uint8_t blockCount) 
{
	uint8_t i,j;
	uint32_t eepromAddress;
	uint32_t eepromBuff[16];
	if(blockCount>0)
	{
		for(i=startBlock; i<(startBlock+blockCount); i++)
		{
			eepromAddress = EEPROMAddrFromBlock(i);
			EEPROMRead(eepromBuff,eepromAddress,64);
			for(j=0;j<16;j++)
			{
				rt_kprintf("Block:%2d->Word:%2d:0x%8x\n",i,j,eepromBuff[j]);
			}
		}
		
	}
}
FINSH_FUNCTION_EXPORT(read_eeprom, read eeprom data by read_eeprom(uint8_t startBlock,uint8_t blockCount).)
#endif
