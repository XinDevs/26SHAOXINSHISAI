#include "Flash.h"

/**
 * @brief  初始化 Flash (主要拉高CS引脚，SPI底层已由SysConfig初始化)
 */
void Flash_Init(void)
{
    /* 默认拉高片选，取消选中 */
    FLASH_CS_HIGH();
}

/**
 * @brief  SPI底层收发一个字节
 * @param  txData: 要发送的数据
 * @retval 接收到的数据
 */
uint8_t Flash_SPI_SwapByte(uint8_t txData)
{
    /* 等待 TX FIFO 有空间 */
    while (DL_SPI_isTXFIFOFull(SPI_FLASH_INST)) {
        ;
    }
    /* 发送 8 bit 数据 */
    DL_SPI_transmitData8(SPI_FLASH_INST, txData);

    /* 等待 RX FIFO 接收到数据 (全双工通信，发一个必收一个) */
    while (DL_SPI_isRXFIFOEmpty(SPI_FLASH_INST)) {
        ;
    }
    /* 返回接收到的数据 */
    return DL_SPI_receiveData8(SPI_FLASH_INST);
}

/**
 * @brief  读取Flash芯片的厂商和设备ID (W25Q128应返回 0xEF17)
 * @retval 16位ID (高8位厂商ID，低8位设备ID)
 */
uint16_t Flash_ReadID(void)
{
    uint16_t id = 0;
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_ManufactDeviceID); // 0x90
    Flash_SPI_SwapByte(0x00);
    Flash_SPI_SwapByte(0x00);
    Flash_SPI_SwapByte(0x00);
    
    id |= (Flash_SPI_SwapByte(0xFF) << 8);     // 读取厂商ID (0xEF)
    id |= Flash_SPI_SwapByte(0xFF);            // 读取设备ID (0x17)
    FLASH_CS_HIGH();
    
    return id;
}

/**
 * @brief  写使能 (Flash在擦除和写入前必须调用)
 */
void Flash_WriteEnable(void)
{
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_WriteEnable);      // 0x06
    FLASH_CS_HIGH();
}

/**
 * @brief  等待Flash内部操作完成 (读取状态寄存器最低位 BUSY)
 */
void Flash_WaitBusy(void)
{
    uint8_t status = 0;
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_ReadStatusReg);    // 0x05
    do {
        status = Flash_SPI_SwapByte(0xFF);
    } while ((status & 0x01) == 0x01);         // 只要最低位是1，说明还在忙
    FLASH_CS_HIGH();
}

/**
 * @brief  擦除指定扇区 (4KB)
 * @param  SectorAddr: 扇区起始地址
 */
void Flash_EraseSector(uint32_t SectorAddr)
{
    Flash_WriteEnable();  // 擦除前必须写使能
    Flash_WaitBusy();     // 确保设备空闲
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_SectorErase);      // 0x20
    Flash_SPI_SwapByte((SectorAddr & 0xFF0000) >> 16); // 发送24位地址
    Flash_SPI_SwapByte((SectorAddr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(SectorAddr & 0xFF);
    FLASH_CS_HIGH();
    
    Flash_WaitBusy();     // 等待擦除完成
}

/**
 * @brief  页编程：向Flash写入数据 (最多256字节，不能跨页写入)
 * @param  Addr: 写入起始地址
 * @param  pBuffer: 数据指针
 * @param  NumByteToWrite: 写入字节数 (1 ~ 256)
 */
void Flash_WritePage(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
    uint16_t i;
    
    Flash_WriteEnable();  // 写入前写使能
    Flash_WaitBusy();     // 确保设备空闲
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_PageProgram);      // 0x02
    Flash_SPI_SwapByte((Addr & 0xFF0000) >> 16);
    Flash_SPI_SwapByte((Addr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(Addr & 0xFF);
    
    for (i = 0; i < NumByteToWrite; i++) {
        Flash_SPI_SwapByte(pBuffer[i]);
    }
    FLASH_CS_HIGH();
    
    Flash_WaitBusy();     // 等待写入完成
}

/**
 * @brief  从Flash读取数据 (没有跨页限制，可以一直读)
 * @param  Addr: 读取起始地址
 * @param  pBuffer: 数据存储缓冲区指针
 * @param  NumByteToRead: 要读取的字节数
 */
void Flash_ReadData(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToRead)
{
    uint16_t i;
    
    Flash_WaitBusy();     // 确保设备空闲
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_ReadData);         // 0x03
    Flash_SPI_SwapByte((Addr & 0xFF0000) >> 16);
    Flash_SPI_SwapByte((Addr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(Addr & 0xFF);
    
    for (i = 0; i < NumByteToRead; i++) {
        pBuffer[i] = Flash_SPI_SwapByte(0xFF); // 发送空字节换取真实数据
    }
    FLASH_CS_HIGH();
}