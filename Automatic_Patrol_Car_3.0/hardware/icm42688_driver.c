/**
 * @file    icm42688_driver.c
 * @brief   ICM-42688-P 六轴IMU驱动（SPI接口）
 * @details 提供ICM-42688加速度计与陀螺仪的初始化、数据读取及零偏校准功能。
 *          SPI通信，支持加速度±4g、陀螺仪±2000dps量程配置。
 */
#include "icm42688_driver.h"
#include "delay.h"
#include "MahonyAHRS.h"

float icm42688_acc_x = 0.0f;
float icm42688_acc_y = 0.0f;
float icm42688_acc_z = 0.0f;
float icm42688_gyro_x = 0.0f;
float icm42688_gyro_y = 0.0f;
float icm42688_gyro_z = 0.0f;
uint8_t icm42688_id = 0U;

static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;
static float acc_offset_x = 0.0f;
static float acc_offset_y = 0.0f;
static float acc_offset_z = 0.0f;

static float yaw_zero_offset = 0.0f;

static const float ACC_SENSITIVITY = 16.0f / 32768.0f;
static const float GYRO_SENSITIVITY = 2000.0f / 32768.0f;



/**
 * @brief  清空 SPI 接收 FIFO
 * @note   避免历史残留数据影响当前事务。
 */
static void ICM_SPI_FlushRXFIFO(void)
{
    while (!DL_SPI_isRXFIFOEmpty(ICM42688_SPI_INST)) {
        (void)DL_SPI_receiveData8(ICM42688_SPI_INST);
    }
}

/**
 * @brief  SPI 单字节收发
 * @param  tx_data 待发送字节
 * @retval 同步收到的字节
 */
static uint8_t ICM_SPI_ReadWriteByte(uint8_t tx_data)
{
    DL_SPI_transmitDataBlocking8(ICM42688_SPI_INST, tx_data);
    return DL_SPI_receiveDataBlocking8(ICM42688_SPI_INST);
}

/**
 * @brief  写 ICM42688 寄存器
 * @param  reg 寄存器地址
 * @param  value 写入值
 */
static void ICM_WriteReg(uint8_t reg, uint8_t value)
{
    ICM_SPI_FlushRXFIFO();
    ICM42688_CS_LOW();
    (void)ICM_SPI_ReadWriteByte(reg & 0x7FU);
    (void)ICM_SPI_ReadWriteByte(value);
    ICM42688_CS_HIGH();
}

/**
 * @brief  连续读取 ICM42688 寄存器
 * @param  reg 起始寄存器地址
 * @param  buf 输出缓冲区
 * @param  len 读取长度
 */
static void ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    ICM_SPI_FlushRXFIFO();
    ICM42688_CS_LOW();
    (void)ICM_SPI_ReadWriteByte(reg | 0x80U);
    for (i = 0U; i < len; i++) {
        buf[i] = ICM_SPI_ReadWriteByte(0xFFU);
    }
    ICM42688_CS_HIGH();
}

/**
 * @brief  片选引脚初始化
 */
static void ICM_CS_Init(void)
{
#ifdef ICM42688_CS_IOMUX
    DL_GPIO_initDigitalOutput(ICM42688_CS_IOMUX);
#endif
    DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_PIN);
    DL_GPIO_enableOutput(ICM42688_CS_PORT, ICM42688_CS_PIN);
    ICM42688_CS_HIGH();
}

/**
 * @brief  SPI 重新配置为软件片选模式
 */
static void ICM_SPI_ReconfigForSoftCS(void)
{
    DL_SPI_disable(ICM42688_SPI_INST);
    DL_SPI_setFrameFormat(ICM42688_SPI_INST, DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0);
    DL_SPI_setChipSelect(ICM42688_SPI_INST, DL_SPI_CHIP_SELECT_NONE);
    DL_SPI_setBitRateSerialClockDivider(ICM42688_SPI_INST, ICM42688_SPI_SCR);
    DL_SPI_enable(ICM42688_SPI_INST);
}

/**
 * @brief  初始化 ICM42688
 * @retval WHO_AM_I 寄存器读到的设备 ID
 * @note   完成复位、量程/滤波配置和工作模式配置。
 */
uint8_t Init_ICM42688(void)
{
    uint8_t retry;

    ICM_CS_Init();
    ICM_SPI_ReconfigForSoftCS();

    ICM42688_CS_HIGH();
    delay_ms(50U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_WriteReg(ICM42688_DEVICE_CONFIG, 0x01U);
    delay_ms(50U);

    retry = 10U;
    while (retry--) {
        ICM_ReadRegs(ICM42688_WHO_AM_I, &icm42688_id, 1U);
        if (icm42688_id == 0x47U) {
            break;
        }
        delay_ms(10U);
    }

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_WriteReg(ICM42688_GYRO_CONFIG0, 0x06U);
    ICM_WriteReg(ICM42688_ACCEL_CONFIG0, 0x06U);
    ICM_WriteReg(ICM42688_PWR_MGMT0, 0x0FU);
    delay_ms(30U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_WriteReg(ICM42688_GYRO_CONFIG1, 0x56U);
    ICM_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, 0x11U);
    ICM_WriteReg(ICM42688_ACCEL_CONFIG1, 0x0DU);

    ICM_WriteReg(ICM42688_INT_CONFIG0, 0x00U);
    ICM_WriteReg(ICM42688_INT_SOURCE0, 0x08U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x01U);
    ICM_WriteReg(0x0BU, 0xA0U);
    ICM_WriteReg(0x0CU, 0x0CU);
    ICM_WriteReg(0x0DU, 0x90U);
    ICM_WriteReg(0x0EU, 0x80U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x02U);
    ICM_WriteReg(0x03U, 0x18U);
    ICM_WriteReg(0x04U, 0x90U);
    ICM_WriteReg(0x05U, 0x80U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_WriteReg(ICM42688_GYRO_CONFIG1, 0x1AU);
    ICM_WriteReg(ICM42688_ACCEL_CONFIG1, 0x15U);
    ICM_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, 0x66U);

    ICM_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
    ICM_WriteReg(ICM42688_PWR_MGMT0, 0x0FU);

    delay_ms(50U);
    return icm42688_id;
}

/**
 * @brief  读取并更新加速度数据
 * @note   单位为 g，包含零偏补偿。
 */
void Get_Acc_ICM42688(void)
{
    uint8_t buf[6];

    ICM_ReadRegs(ICM42688_ACCEL_DATA_X1, buf, 6U);
    icm42688_acc_x = (int16_t)((buf[0] << 8) | buf[1]) * ACC_SENSITIVITY;
    icm42688_acc_y = (int16_t)((buf[2] << 8) | buf[3]) * ACC_SENSITIVITY;
    icm42688_acc_z = (int16_t)((buf[4] << 8) | buf[5]) * ACC_SENSITIVITY;

    icm42688_acc_x -= acc_offset_x;
    icm42688_acc_y -= acc_offset_y;
    icm42688_acc_z -= acc_offset_z;
}

/**
 * @brief  读取并更新陀螺仪数据
 * @note   单位为 dps，包含零偏补偿。
 */
void Get_Gyro_ICM42688(void)
{
    uint8_t buf[6];

    ICM_ReadRegs(ICM42688_GYRO_DATA_X1, buf, 6U);
    icm42688_gyro_x = (int16_t)((buf[0] << 8) | buf[1]) * GYRO_SENSITIVITY;
    icm42688_gyro_y = (int16_t)((buf[2] << 8) | buf[3]) * GYRO_SENSITIVITY;
    icm42688_gyro_z = (int16_t)((buf[4] << 8) | buf[5]) * GYRO_SENSITIVITY;

    icm42688_gyro_x -= gyro_offset_x;
    icm42688_gyro_y -= gyro_offset_y;
    icm42688_gyro_z -= gyro_offset_z;
}

/**
 * @brief  陀螺仪零偏校准
 * @note   采样多次并取平均值作为静态零偏。
 */
void IMU_Calibrate(void)
{
    const uint16_t samples = 500U;
    uint16_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (i = 0U; i < samples; i++) {
        Get_Gyro_ICM42688();
        sum_x += icm42688_gyro_x;
        sum_y += icm42688_gyro_y;
        sum_z += icm42688_gyro_z;
        delay_ms(2U);
    }

    gyro_offset_x = sum_x / (float)samples;
    gyro_offset_y = sum_y / (float)samples;
    gyro_offset_z = sum_z / (float)samples;
}

/**
 * @brief  获取带零点偏移的Yaw角(度)
 * @retval Yaw角(度), 范围 -180 ~ +180
 */
float ICM42688_GetYawZeroedDeg(void)
{
    float yaw = getYaw();
    float diff = yaw - yaw_zero_offset;

    /* 将差值归一化到 -180 ~ +180 */
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}

/**
 * @brief  将当前Yaw角设为零点
 */
void ICM42688_ResetYawZero(void)
{
    yaw_zero_offset = getYaw();
}
