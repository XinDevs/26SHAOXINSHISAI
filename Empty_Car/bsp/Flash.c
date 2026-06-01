#include "Flash.h"

#define FLASH_SPI_TIMEOUT       10000UL
#define FLASH_BUSY_TIMEOUT      500000UL
#define FLASH_ERASE_TIMEOUT     8000000UL

/**
 * @brief  初始化 Flash (主要拉高CS引脚，SPI底层已由SysConfig初始化)
 */
void Flash_Init(void)
{
    DL_SPI_disable(SPI_FLASH_INST);
    DL_SPI_setFrameFormat(SPI_FLASH_INST, DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0);
    DL_SPI_setChipSelect(SPI_FLASH_INST, DL_SPI_CHIP_SELECT_NONE);
    DL_SPI_setBitRateSerialClockDivider(SPI_FLASH_INST, 39U);
    DL_SPI_enable(SPI_FLASH_INST);

    while (!DL_SPI_isRXFIFOEmpty(SPI_FLASH_INST)) {
        (void)DL_SPI_receiveData8(SPI_FLASH_INST);
    }

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
    uint32_t timeout;

    /* 等待 TX FIFO 有空间 */
    timeout = FLASH_SPI_TIMEOUT;
    while (DL_SPI_isTXFIFOFull(SPI_FLASH_INST)) {
        if (timeout-- == 0UL) {
            return 0xFFU;
        }
    }
    /* 发送 8 bit 数据 */
    DL_SPI_transmitData8(SPI_FLASH_INST, txData);

    /* 等待 RX FIFO 接收到数据 (全双工通信，发一个必收一个) */
    timeout = FLASH_SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_FLASH_INST)) {
        if (timeout-- == 0UL) {
            return 0xFFU;
        }
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

uint8_t Flash_ReadStatus(void)
{
    uint8_t status;

    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_ReadStatusReg);    // 0x05
    status = Flash_SPI_SwapByte(0xFF);
    FLASH_CS_HIGH();

    return status;
}

/**
 * @brief  等待Flash内部操作完成 (读取状态寄存器最低位 BUSY)
 */
static uint8_t Flash_WaitBusyWithTimeout(uint32_t timeout)
{
    uint8_t status = 0;

    do {
        status = Flash_ReadStatus();
        if (timeout-- == 0UL) {
            return 0U;
        }
    } while ((status & 0x01) == 0x01);         // 只要最低位是1，说明还在忙

    return 1U;
}

uint8_t Flash_WaitBusy(void)
{
    return Flash_WaitBusyWithTimeout(FLASH_BUSY_TIMEOUT);
}

/**
 * @brief  擦除指定扇区 (4KB)
 * @param  SectorAddr: 扇区起始地址
 */
uint8_t Flash_EraseSector(uint32_t SectorAddr)
{
    Flash_WriteEnable();  // 擦除前必须写使能
    if (Flash_WaitBusy() == 0U) {
        return 0U;
    }
    if ((Flash_ReadStatus() & 0x02U) == 0U) {
        return 2U;
    }
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_SectorErase);      // 0x20
    Flash_SPI_SwapByte((SectorAddr & 0xFF0000) >> 16); // 发送24位地址
    Flash_SPI_SwapByte((SectorAddr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(SectorAddr & 0xFF);
    FLASH_CS_HIGH();
    
    return Flash_WaitBusyWithTimeout(FLASH_ERASE_TIMEOUT);     // 等待擦除完成
}

/**
 * @brief  页编程：向Flash写入数据 (最多256字节，不能跨页写入)
 * @param  Addr: 写入起始地址
 * @param  pBuffer: 数据指针
 * @param  NumByteToWrite: 写入字节数 (1 ~ 256)
 */
uint8_t Flash_WritePage(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
    uint16_t i;
    
    Flash_WriteEnable();  // 写入前写使能
    if (Flash_WaitBusy() == 0U) {
        return 0U;
    }
    if ((Flash_ReadStatus() & 0x02U) == 0U) {
        return 2U;
    }
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_PageProgram);      // 0x02
    Flash_SPI_SwapByte((Addr & 0xFF0000) >> 16);
    Flash_SPI_SwapByte((Addr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(Addr & 0xFF);
    
    for (i = 0; i < NumByteToWrite; i++) {
        Flash_SPI_SwapByte(pBuffer[i]);
    }
    FLASH_CS_HIGH();
    
    return Flash_WaitBusy();     // 等待写入完成
}

/**
 * @brief  从Flash读取数据 (没有跨页限制，可以一直读)
 * @param  Addr: 读取起始地址
 * @param  pBuffer: 数据存储缓冲区指针
 * @param  NumByteToRead: 要读取的字节数
 */
uint8_t Flash_ReadData(uint32_t Addr, uint8_t *pBuffer, uint16_t NumByteToRead)
{
    uint16_t i;
    
    if (Flash_WaitBusy() == 0U) {
        return 0U;
    }
    
    FLASH_CS_LOW();
    Flash_SPI_SwapByte(W25X_ReadData);         // 0x03
    Flash_SPI_SwapByte((Addr & 0xFF0000) >> 16);
    Flash_SPI_SwapByte((Addr & 0xFF00) >> 8);
    Flash_SPI_SwapByte(Addr & 0xFF);
    
    for (i = 0; i < NumByteToRead; i++) {
        pBuffer[i] = Flash_SPI_SwapByte(0xFF); // 发送空字节换取真实数据
    }
    FLASH_CS_HIGH();
    return 1U;
}

/**
 * @brief  Flash读写自检
 * @note   会擦除 FLASH_TEST_ADDR 所在4KB扇区，仅用于测试。
 * @param  idOut 输出Flash ID，可传NULL
 * @retval FLASH_TEST_OK/FLASH_TEST_xxx
 */
uint8_t Flash_Test(uint16_t *idOut)
{
    uint8_t writeBuf[FLASH_TEST_LEN] = {
        0x26U, 0x5AU, 0xA5U, 0x01U,
        0x02U, 0x03U, 0x04U, 0x05U,
        0x10U, 0x20U, 0x30U, 0x40U,
        0x55U, 0xAAU, 0xC3U, 0x3CU
    };
    uint8_t readBuf[FLASH_TEST_LEN];
    uint16_t id;
    uint16_t i;

    Flash_Init();
    id = Flash_ReadID();
    if (idOut != 0) {
        *idOut = id;
    }

    if ((id == 0x0000U) || (id == 0xFFFFU) || ((id & 0xFF00U) != 0xEF00U)) {
        return FLASH_TEST_ID_FAIL;
    }

    {
        uint8_t eraseRet = Flash_EraseSector(FLASH_TEST_ADDR);
        if (eraseRet == 2U) {
            return FLASH_TEST_WEL_FAIL;
        }
        if (eraseRet == 0U) {
            return FLASH_TEST_ERASE_TIMEOUT;
        }
    }
    if (Flash_ReadData(FLASH_TEST_ADDR, readBuf, FLASH_TEST_LEN) == 0U) {
        return FLASH_TEST_READ1_TIMEOUT;
    }
    for (i = 0U; i < FLASH_TEST_LEN; i++) {
        if (readBuf[i] != 0xFFU) {
            return FLASH_TEST_ERASE_FAIL;
        }
    }

    {
        uint8_t writeRet = Flash_WritePage(FLASH_TEST_ADDR, writeBuf, FLASH_TEST_LEN);
        if (writeRet == 2U) {
            return FLASH_TEST_WEL_FAIL;
        }
        if (writeRet == 0U) {
            return FLASH_TEST_WRITE_TIMEOUT;
        }
    }
    if (Flash_ReadData(FLASH_TEST_ADDR, readBuf, FLASH_TEST_LEN) == 0U) {
        return FLASH_TEST_READ2_TIMEOUT;
    }
    for (i = 0U; i < FLASH_TEST_LEN; i++) {
        if (readBuf[i] != writeBuf[i]) {
            return FLASH_TEST_WRITE_FAIL;
        }
    }

    return FLASH_TEST_OK;
}
