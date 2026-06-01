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

#define FLASH_TEST_ADDR         0x000F0000UL
#define FLASH_TEST_LEN          16U

#define FLASH_TEST_OK           0U
#define FLASH_TEST_ID_FAIL      1U
#define FLASH_TEST_ERASE_FAIL   2U
#define FLASH_TEST_WRITE_FAIL   3U
#define FLASH_TEST_ERASE_TIMEOUT 4U
#define FLASH_TEST_READ1_TIMEOUT 5U
#define FLASH_TEST_WRITE_TIMEOUT 6U
#define FLASH_TEST_READ2_TIMEOUT 7U
#define FLASH_TEST_WEL_FAIL      8U

/* 片选(CS)引脚控制宏定义 (基于你的 SysConfig 设置) */
#define FLASH_CS_LOW()      DL_GPIO_clearPins(FLASH_CS_PORT, FLASH_CS_PIN_1_PIN)
#define FLASH_CS_HIGH()     DL_GPIO_setPins(FLASH_CS_PORT, FLASH_CS_PIN_1_PIN)

/* 函数声明 */
void Flash_Init(void);
uint8_t Flash_SPI_SwapByte(uint8_t txData);
uint16_t Flash_ReadID(void);
uint8_t Flash_ReadStatus(void);
void Flash_WriteEnable(void);
uint8_t Flash_WaitBusy(void);
uint8_t Flash_EraseSector(uint32_t SectorAddr);
uint8_t Flash_WritePage(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToWrite);
uint8_t Flash_ReadData(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToRead);
uint8_t Flash_Test(uint16_t *idOut);

#endif /* __FLASH_H__ */
