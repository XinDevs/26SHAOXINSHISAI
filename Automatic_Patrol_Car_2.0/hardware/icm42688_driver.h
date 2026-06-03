/**
 * @file    icm42688_driver.h
 * @brief   ICM-42688-P 六轴IMU驱动 — 头文件
 * @details 声明ICM-42688初始化、加速度/陀螺仪数据读取及校准接口，
 *          以及SPI引脚配置宏和寄存器地址定义。
 */
#ifndef ICODE_ICM42688_DRIVER_H_
#define ICODE_ICM42688_DRIVER_H_

#include <stdint.h>

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * 用户可按硬件改这两个宏
 * ========================= */
#ifndef ICM42688_SPI_INST
#define ICM42688_SPI_INST              SPI_ICM_INST
#endif

#ifndef ICM42688_CS_PORT
#if defined(ICM42688P_CS_PORT)
#define ICM42688_CS_PORT               ICM42688P_CS_PORT
#elif defined(ICM42688P_PORT)
#define ICM42688_CS_PORT               ICM42688P_PORT
#else
#define ICM42688_CS_PORT               GPIOA
#endif
#endif

#ifndef ICM42688_CS_PIN
#if defined(ICM42688P_CS_PIN)
#define ICM42688_CS_PIN                ICM42688P_CS_PIN
#else
#define ICM42688_CS_PIN                DL_GPIO_PIN_7
#endif
#endif

/* 可选：若 SysConfig 里创建了 ICM42688P/CS，自动使用其 IOMUX 宏 */
#ifndef ICM42688_CS_IOMUX
#if defined(ICM42688P_CS_IOMUX)
#define ICM42688_CS_IOMUX              ICM42688P_CS_IOMUX
#endif
#endif

/* SPI 位率分频系数 SCR：
 * f_spi = f_spi_clk / (2 * (1 + SCR))
 * 80MHz 下 SCR=39 -> 1MHz
 */
#ifndef ICM42688_SPI_SCR
#define ICM42688_SPI_SCR               (39U)
#endif

#define ICM42688_CS_LOW()              DL_GPIO_clearPins(ICM42688_CS_PORT, ICM42688_CS_PIN)
#define ICM42688_CS_HIGH()             DL_GPIO_setPins(ICM42688_CS_PORT, ICM42688_CS_PIN)

extern float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z;
extern float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z;
extern uint8_t icm42688_id;

/**
 * @brief  初始化 ICM42688
 * @retval WHO_AM_I 设备 ID
 */
uint8_t Init_ICM42688(void);
/**
 * @brief  读取并更新加速度数据
 */
void Get_Acc_ICM42688(void);
/**
 * @brief  读取并更新陀螺仪数据
 */
void Get_Gyro_ICM42688(void);
/**
 * @brief  执行 IMU 零偏校准
 */
void IMU_Calibrate(void);
/**
 * @brief  获取带零点偏移的Yaw角(度)
 * @retval Yaw角(度), 范围 -180 ~ +180
 */
float ICM42688_GetYawZeroedDeg(void);
/**
 * @brief  将当前Yaw角设为零点
 */
void ICM42688_ResetYawZero(void);

/* Bank 0 常用寄存器 */
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_REG_BANK_SEL              0x76

#ifdef __cplusplus
}
#endif

#endif /* ICODE_ICM42688_DRIVER_H_ */
