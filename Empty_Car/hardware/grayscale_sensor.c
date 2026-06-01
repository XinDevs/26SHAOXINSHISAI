/**
 * @file    grayscale_sensor.c
 * @brief   8 路灰度传感器驱动
 * @details 提供灰度模块 I2C 读写封装、设备探测、数字/模拟量读取，
 *          以及 8bit 数字量展开到 8 路传感数组的工具函数。
 */

#include "grayscale_sensor.h"
#include "delay.h"

#define GW_GRAY_I2C_TIMEOUT_CNT    (200000U)

uint8_t sensor[GW_GRAY_CHANNEL_COUNT] = {0};  /* sensor[0]=1号(最左), sensor[15]=16号(最右) */

/* ============== 内部私有底层 I2C 实现 ===================*/

/**
 * @brief  I2C 异常恢复
 * @note   在超时或错误后复位控制器与 FIFO。
 */
static void grayscale_i2c_recover(void)
{
    DL_I2C_disableController(I2C_GRAYSCALE_INST);
    DL_I2C_resetControllerTransfer(I2C_GRAYSCALE_INST);
    DL_I2C_flushControllerTXFIFO(I2C_GRAYSCALE_INST);
    DL_I2C_flushControllerRXFIFO(I2C_GRAYSCALE_INST);
    SYSCFG_DL_I2C_GRAYSCALE_init();
}

/**
 * @brief  等待 I2C 控制器空闲
 * @retval GW_GRAY_OK: 空闲可用
 * @retval GW_GRAY_I2C_ERROR: 超时并已执行恢复
 */
static unsigned char grayscale_wait_controller_idle(void)
{
    uint32_t timeout = GW_GRAY_I2C_TIMEOUT_CNT;
    while (((DL_I2C_getControllerStatus(I2C_GRAYSCALE_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) && (timeout-- > 0U));
    if (timeout > 0U) return GW_GRAY_OK;
    
    grayscale_i2c_recover();
    return GW_GRAY_I2C_ERROR;
}

/**
 * @brief  I2C 连续写
 * @param  slave_addr 从机地址
 * @param  buf 写入数据缓冲
 * @param  len 写入字节数
 * @retval GW_GRAY_OK: 写入成功
 * @retval GW_GRAY_I2C_ERROR: 总线错误或超时
 * @retval GW_GRAY_ERROR: 参数非法
 */
static unsigned char grayscale_i2c_tx(uint8_t slave_addr, const uint8_t *buf, uint8_t len)
{
    uint32_t timeout, status;
    uint8_t i;

    if ((buf == 0) || (len == 0U)) return GW_GRAY_ERROR;
    if (grayscale_wait_controller_idle() != GW_GRAY_OK) return GW_GRAY_I2C_ERROR;

    DL_I2C_resetControllerTransfer(I2C_GRAYSCALE_INST);
    DL_I2C_flushControllerTXFIFO(I2C_GRAYSCALE_INST);
    DL_I2C_startControllerTransfer(I2C_GRAYSCALE_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    for (i = 0U; i < len; i++) {
        timeout = GW_GRAY_I2C_TIMEOUT_CNT;
        while ((DL_I2C_getControllerTXFIFOCounter(I2C_GRAYSCALE_INST) == 0U) && (timeout-- > 0U)) {
            status = DL_I2C_getControllerStatus(I2C_GRAYSCALE_INST);
            if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                grayscale_i2c_recover();
                return GW_GRAY_I2C_ERROR;
            }
        }
        if (timeout == 0U) { grayscale_i2c_recover(); return GW_GRAY_I2C_ERROR; }
        
        DL_I2C_transmitControllerData(I2C_GRAYSCALE_INST, buf[i]);
    }

    if (grayscale_wait_controller_idle() != GW_GRAY_OK) return GW_GRAY_I2C_ERROR;
    
    status = DL_I2C_getControllerStatus(I2C_GRAYSCALE_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) { grayscale_i2c_recover(); return GW_GRAY_I2C_ERROR; }
    
    return GW_GRAY_OK;
}

/**
 * @brief  I2C 复合读事务(先写寄存器地址, 再读数据)
 * @param  slave_addr 从机地址
 * @param  reg_addr 寄存器地址
 * @param  data 读出缓冲区
 * @param  len 读取长度
 * @retval GW_GRAY_OK: 读取成功
 * @retval GW_GRAY_I2C_ERROR: 总线错误或超时
 */
static unsigned char grayscale_i2c_write_then_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    // 先单独写寄存器地址 (带 STOP)
    uint8_t tx_buf[1] = {reg_addr};
    if (grayscale_i2c_tx(slave_addr, tx_buf, 1) != GW_GRAY_OK) {
        return GW_GRAY_I2C_ERROR;
    }
    
    // 给从机一点点反应时间 (有些性能差的从机需要)
    delay_ms(1); 
    
    // 再单独发起读操作
    if (grayscale_wait_controller_idle() != GW_GRAY_OK) return GW_GRAY_I2C_ERROR;
    
    DL_I2C_resetControllerTransfer(I2C_GRAYSCALE_INST);
    DL_I2C_flushControllerRXFIFO(I2C_GRAYSCALE_INST);
    DL_I2C_startControllerTransfer(I2C_GRAYSCALE_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    
    uint32_t timeout;
    for (uint8_t i = 0; i < len; i++) {
        timeout = GW_GRAY_I2C_TIMEOUT_CNT;
        while ((DL_I2C_getControllerRXFIFOCounter(I2C_GRAYSCALE_INST) == 0U) && (timeout-- > 0U));
        if (timeout == 0U) { grayscale_i2c_recover(); return GW_GRAY_I2C_ERROR; }
        data[i] = DL_I2C_receiveControllerData(I2C_GRAYSCALE_INST);
    }
    
    return GW_GRAY_OK;
}

/**
 * @brief  读取单字节寄存器
 */
static unsigned char IIC_Read_Byte(unsigned char Salve_Address, unsigned char Reg_Address, unsigned char *data) {
    return grayscale_i2c_write_then_read(Salve_Address, Reg_Address, data, 1U);
}

/**
 * @brief  连续读取多个寄存器字节
 */
static unsigned char IIC_Read_Bytes(unsigned char Salve_Address, unsigned char Reg_Address, unsigned char *Result, unsigned char len) {
    return grayscale_i2c_write_then_read(Salve_Address, Reg_Address, Result, len);
}

/**
 * @brief  写入单字节寄存器
 */
static unsigned char IIC_Write_Byte(unsigned char Salve_Address, unsigned char Reg_Address, unsigned char data) {
    uint8_t tx_buf[2] = {Reg_Address, data};
    return grayscale_i2c_tx(Salve_Address, tx_buf, 2U);
}


/* ============== 用户逻辑功能接口 ===================*/

/**
 * @brief  探测指定地址的灰度模块是否在线
 * @param  addr 从机地址
 * @retval GW_GRAY_OK: 在线
 * @retval 其他: 不在线或通信失败
 */
unsigned char IIC_Probe(uint8_t addr)
{
    uint8_t cmd = GW_GRAY_DIGITAL_MODE;
    return grayscale_i2c_tx(addr, &cmd, 1U);
}

/**
 * @brief  读取 Ping 寄存器确认设备应答
 * @param  addr 从机地址
 * @retval GW_GRAY_OK: 应答正确
 * @retval GW_GRAY_PING_FAIL: 应答异常
 */
unsigned char Ping(uint8_t addr)
{
    unsigned char dat = 0U;
    if (IIC_Read_Byte(addr, GW_GRAY_PING, &dat) == GW_GRAY_OK) {
        if (dat == GW_GRAY_PING_OK) return GW_GRAY_OK;
    }
    return GW_GRAY_PING_FAIL;
}

/**
 * @brief  获取数字模式 8bit 结果
 * @param  addr 从机地址
 * @retval 数字模式原始字节
 */
unsigned char IIC_Get_Digtal(uint8_t addr)
{
    unsigned char dat = 0U;
    (void) IIC_Read_Byte(addr, GW_GRAY_DIGITAL_MODE, &dat);
    return dat;
}

/**
 * @brief  获取数字模式 8bit 结果(带状态返回)
 * @param  addr 从机地址
 * @param  digitalValue 输出字节指针
 * @retval GW_GRAY_OK: 读取成功
 * @retval GW_GRAY_ERROR/GW_GRAY_I2C_ERROR: 失败
 */
unsigned char IIC_Get_Digtal_Ex(uint8_t addr, uint8_t *digitalValue)
{
    unsigned char dat = 0U;
    if (digitalValue == 0) return GW_GRAY_ERROR;
    
    if (IIC_Read_Byte(addr, GW_GRAY_DIGITAL_MODE, &dat) == GW_GRAY_OK) {
        *digitalValue = dat;
        return GW_GRAY_OK;
    }
    *digitalValue = 0U;
    return GW_GRAY_I2C_ERROR;
}

/**
 * @brief  读取模拟模式连续数据
 * @param  addr 从机地址
 * @param  Result 输出缓冲区
 * @param  len 读取长度
 * @retval I2C 操作结果码
 */
unsigned char IIC_Get_Anolog(uint8_t addr, unsigned char *Result, unsigned char len)
{
    if (IIC_Write_Byte(addr, GW_GRAY_ANALOG_BASE_, 0x00U) != GW_GRAY_OK) return GW_GRAY_ERROR;
    delay_ms(1U);
    return IIC_Read_Bytes(addr, GW_GRAY_ANALOG_BASE_, Result, len);
}

/**
 * @brief  读取单通道模拟值
 * @param  addr 从机地址
 * @param  Channel 通道号(1~8)
 * @retval 通道模拟值
 */
unsigned char IIC_Get_Single_Anolog(uint8_t addr, unsigned char Channel)
{
    unsigned char dat = 0U;
    if ((Channel < 1U) || (Channel > 8U)) return 0U;
    (void) IIC_Read_Byte(addr, (unsigned char)(GW_GRAY_ANALOG_SINGLE_BASE + (Channel - 1U)), &dat);
    return dat;
}

/**
 * @brief  使能/配置模拟归一化
 * @param  addr 从机地址
 * @param  Normalize_channel 归一化通道掩码
 * @retval I2C 操作结果码
 */
unsigned char IIC_Anolog_Normalize(uint8_t addr, uint8_t Normalize_channel) {
    return IIC_Write_Byte(addr, GW_GRAY_NORMALIZE_ENABLE, Normalize_channel);
}

/**
 * @brief  配置通道使能掩码
 * @param  addr 从机地址
 * @param  ChannelEnable 通道掩码
 * @retval I2C 操作结果码
 */
unsigned char IIC_Set_Channel_Enable(uint8_t addr, uint8_t ChannelEnable) {
    return IIC_Write_Byte(addr, GW_GRAY_CHANNEL_ENABLE, ChannelEnable);
}

/**
 * @brief  读取模块错误信息寄存器
 * @param  addr 从机地址
 * @retval 错误信息字节
 */
unsigned char IIC_Get_Error_Info(uint8_t addr) {
    unsigned char error_info = 0U;
    (void) IIC_Read_Byte(addr, GW_GRAY_ERROR_INFO, &error_info);
    return error_info;
}

/**
 * @brief  触发模块软件复位
 * @param  addr 从机地址
 * @retval I2C 操作结果码
 */
unsigned char IIC_Software_Reset(uint8_t addr) {
    return IIC_Write_Byte(addr, GW_GRAY_SOFTWARE_RESET, 0x00U);
}

/**
 * @brief  将 8bit 灰度数字量提取左/右两路到 sensor 数组
 * @param  data 8bit 原始位图
 * @note   取反处理：硬件原始值 0=黑线, 1=白地
 *         取反后 sensor[i]=1 表示检测到黑线, sensor[i]=0 表示白地
 *         sensor[0] = bit3(左), sensor[1] = bit4(右)
 */
void grayscale_byte_to_sensor_array(uint8_t data)
{
    uint8_t i;

    for (i = 0U; i < GW_GRAY_MODULE_CHANNEL_COUNT; i++) {
        sensor[i] = ((data & (1U << i)) == 0U) ? 1U : 0U;
    }
}

void grayscale_dual_byte_to_sensor_array(uint8_t leftData, uint8_t rightData)
{
    uint8_t i;

    for (i = 0U; i < GW_GRAY_MODULE_CHANNEL_COUNT; i++) {
        sensor[i] = ((leftData & (1U << i)) == 0U) ? 1U : 0U;
        sensor[i + GW_GRAY_MODULE_CHANNEL_COUNT] =
            ((rightData & (1U << i)) == 0U) ? 1U : 0U;
    }
}

unsigned char grayscale_update_sensor_array(void)
{
    uint8_t grayLeft = 0U;
    uint8_t grayRight = 0U;

    if (IIC_Get_Digtal_Ex(GW_GRAY_ADDR_SENSOR_LEFT, &grayLeft) != GW_GRAY_OK) {
        return GW_GRAY_I2C_ERROR;
    }

    if (IIC_Get_Digtal_Ex(GW_GRAY_ADDR_SENSOR_RIGHT, &grayRight) != GW_GRAY_OK) {
        return GW_GRAY_I2C_ERROR;
    }

    grayscale_dual_byte_to_sensor_array(grayLeft, grayRight);
    return GW_GRAY_OK;
}
