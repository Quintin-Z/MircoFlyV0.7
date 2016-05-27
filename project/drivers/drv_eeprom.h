#ifndef __DRV_EEPROM_H__
#define __DRV_EEPROM_H__
#include <stdbool.h>
#include <stdint.h>
void eerprom_init(void);
int8_t eeprom_read_byte(uint32_t *pfeepromBuff,uint32_t startAddress,uint32_t byteCount);
int8_t eeprom_read_block(uint32_t *pfeepromBuff,uint8_t startBlock,uint8_t blockCount);
uint32_t eeprom_write_byte(uint32_t *pfeepromBuff,uint32_t startAddress,uint32_t byteCount);
#endif
