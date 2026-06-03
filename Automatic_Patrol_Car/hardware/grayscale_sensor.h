/**
 * @file    grayscale_sensor.h
 * @brief   8 路灰度传感器头文件
 * @details 声明灰度模块寄存器宏、状态码和对外 API。
 *          I2C 地址: 0x4C。
 */

#ifndef ICODE_GRAYSCALE_SENSOR_H_
#define ICODE_GRAYSCALE_SENSOR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 寄存器及命令宏定义 ================= */
#define GW_GRAY_PING                  0xAA
#define GW_GRAY_PING_OK               0x66
#define GW_GRAY_DIGITAL_MODE          0xDD
#define GW_GRAY_ANALOG_BASE_          0xB0
#define GW_GRAY_ANALOG_SINGLE_BASE    0xB1
#define GW_GRAY_CHANNEL_ENABLE        0xCE
#define GW_GRAY_NORMALIZE_ENABLE      0xCF
#define GW_GRAY_HYSTERESIS_GRAYB      0xD0
#define GW_GRAY_HYSTERESIS_GRAYW      0xD1
#define GW_GRAY_SOFTWARE_ADDRESS      0xAD
#define GW_GRAY_ERROR_INFO            0xDE
#define GW_GRAY_SOFTWARE_RESET        0xC0
#define GW_GRAY_FIRMWARE_VERSION      0xC1

/* ================= 传感器 I2C 地址 ================= */
#define GW_GRAY_ADDR_SENSOR_LEFT      0x4C   // 左模块地址(0b1001100)
#define GW_GRAY_ADDR_SENSOR_RIGHT     0x4D   // 右模块地址(0b1001101)
#define GW_GRAY_ADDR_SENSOR           0x4C   // 兼容旧代码

/* ================= 运行状态宏 ================= */
#define GW_GRAY_OK                    0x00
#define GW_GRAY_ERROR                 0x01
#define GW_GRAY_PING_FAIL             0x02
#define GW_GRAY_I2C_ERROR             0x03
#define GW_GRAY_BUSY                  0x04

/* 全局数组：存储8路灰度状态 */
#define GW_GRAY_CHANNEL_COUNT         16U
#define GW_GRAY_MODULE_CHANNEL_COUNT  8U

extern uint8_t sensor[GW_GRAY_CHANNEL_COUNT];  /* sensor[0]=0号(最左), sensor[15]=15号(最右) */

/* ================== 用户 API 声明 ================== */
/**
 * @brief  读取 Ping 寄存器确认设备应答
 * @param  addr 从机地址
 * @retval GW_GRAY_OK: 应答正确
 * @retval GW_GRAY_PING_FAIL: 应答异常
 */
unsigned char Ping(uint8_t addr);
/**
 * @brief  获取数字模式 8bit 结果
 * @param  addr 从机地址
 * @retval 数字模式原始字节
 */
unsigned char IIC_Get_Digtal(uint8_t addr);
/**
 * @brief  获取数字模式 8bit 结果(带状态返回)
 * @param  addr 从机地址
 * @param  digitalValue 输出字节指针
 * @retval GW_GRAY_OK: 读取成功
 * @retval GW_GRAY_ERROR/GW_GRAY_I2C_ERROR: 失败
 */
unsigned char IIC_Get_Digtal_Ex(uint8_t addr, uint8_t *digitalValue);
/**
 * @brief  读取模拟模式连续数据
 * @param  addr 从机地址
 * @param  Result 输出缓冲区
 * @param  len 读取长度
 * @retval I2C 操作结果码
 */
unsigned char IIC_Get_Anolog(uint8_t addr, unsigned char *Result, unsigned char len);
/**
 * @brief  读取单通道模拟值
 * @param  addr 从机地址
 * @param  Channel 通道号(1~8)
 * @retval 通道模拟值
 */
unsigned char IIC_Get_Single_Anolog(uint8_t addr, unsigned char Channel);
/**
 * @brief  使能/配置模拟归一化
 * @param  addr 从机地址
 * @param  Normalize_channel 归一化通道掩码
 * @retval I2C 操作结果码
 */
unsigned char IIC_Anolog_Normalize(uint8_t addr, uint8_t Normalize_channel);
/**
 * @brief  读取偏移值(接口预留)
 * @retval 偏移值
 */
unsigned short IIC_Get_Offset(void);
/**
 * @brief  配置通道使能掩码
 * @param  addr 从机地址
 * @param  ChannelEnable 通道掩码
 * @retval I2C 操作结果码
 */
unsigned char IIC_Set_Channel_Enable(uint8_t addr, uint8_t ChannelEnable);
/**
 * @brief  读取模块错误信息寄存器
 * @param  addr 从机地址
 * @retval 错误信息字节
 */
unsigned char IIC_Get_Error_Info(uint8_t addr);
/**
 * @brief  触发模块软件复位
 * @param  addr 从机地址
 * @retval I2C 操作结果码
 */
unsigned char IIC_Software_Reset(uint8_t addr);
/**
 * @brief  读取固件版本号
 * @param  addr 从机地址
 * @retval 固件版本
 */
unsigned char IIC_Get_Firmware_Version(uint8_t addr);

/**
 * @brief  将 8bit 灰度数字量展开到 sensor 数组
 * @param  data 8bit 原始位图
 */
void grayscale_byte_to_sensor_array(uint8_t data);
/* 将左右两模块各8bit数字量合并展开到16路sensor数组 */
void grayscale_dual_byte_to_sensor_array(uint8_t leftData, uint8_t rightData);
/* 同步读取左右两模块数字量并更新sensor数组（阻塞） */
unsigned char grayscale_update_sensor_array(void);
/* 异步读取灰度传感器，按指定间隔分步完成左右模块通信 */
unsigned char grayscale_update_sensor_array_async(uint32_t nowMs);
/* 获取异步读取状态：0=空闲，1=进行中 */
unsigned char grayscale_get_async_status(void);
/**
 * @brief  探测指定地址的灰度模块是否在线
 * @param  addr 从机地址
 * @retval GW_GRAY_OK: 在线
 * @retval 其他: 不在线或通信失败
 */
unsigned char IIC_Probe(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* ICODE_GRAYSCALE_SENSOR_H_ */
