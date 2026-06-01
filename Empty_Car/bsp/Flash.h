#ifndef __FLASH_H__
#define __FLASH_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* W25Q128 常用指令表 */
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

/* 片选(CS)引脚控制宏定义 (基于你的 SysConfig 设置) */
#define FLASH_CS_LOW()      DL_GPIO_clearPins(FLASH_CS_PORT, FLASH_CS_PIN_1_PIN)
#define FLASH_CS_HIGH()     DL_GPIO_setPins(FLASH_CS_PORT, FLASH_CS_PIN_1_PIN)

/* 函数声明 */
void Flash_Init(void);
uint8_t Flash_SPI_SwapByte(uint8_t txData);
uint16_t Flash_ReadID(void);
void Flash_WriteEnable(void);
void Flash_WaitBusy(void);
void Flash_EraseSector(uint32_t SectorAddr);
void Flash_WritePage(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToWrite);
void Flash_ReadData(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToRead);

#endif /* __FLASH_H__ */