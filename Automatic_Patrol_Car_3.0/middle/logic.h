#ifndef __LOGIC_H
#define __LOGIC_H

#include <stdint.h>
#include <stdbool.h>

// ================= 1. 系统常量与枚举 =================
#define NODE_O  0
#define NODE_N1 1
#define NODE_N2 2
#define NODE_N3 3
#define NODE_N4 4
#define NODE_N5 5

#define TURN_LEFT  1
#define TURN_RIGHT 0

#define MODE_EXPLORE 0
#define MODE_GO_HOME 1

#define COLOR_NONE  0
#define COLOR_RED   1
#define COLOR_GREEN 2

#define SHAPE_NONE   0
#define SHAPE_CIRCLE 1
#define SHAPE_SQUARE 2

// 标识字母枚举 (用于映射和存储)
typedef enum {
    LETTER_NONE = 0,
    LETTER_A, LETTER_B, LETTER_C, LETTER_D,
    LETTER_E, LETTER_F, LETTER_G, LETTER_H,
    LETTER_I, LETTER_J, LETTER_K, LETTER_L
} LetterType;

// 视觉数据结构体
typedef struct {
    uint8_t color;  // 颜色：COLOR_RED, COLOR_GREEN, COLOR_NONE
    uint8_t shape;  // 形状：SHAPE_CIRCLE, SHAPE_SQUARE, SHAPE_NONE
} MaixCam_Data;

// ================= 2. 全局状态变量暴露 =================
// 暴露给主函数，方便在 OLED 上显示系统当前状态
extern int sys_state;
extern int last_node;
extern int curr_node;
extern int found_count;
extern int step;
extern int target_N; // 可在 main 函数启动前通过按键修改
extern LetterType map_forward_letter[6][6];

// ================= 3. 核心逻辑函数 =================
/**
 * @brief  初始化地图与视场映射阵列，在 main() 开始时调用一次
 */
void Init_Map(void);

/**
 * @brief  路口处理核心中断/事件函数。
 * @note   控制组的同学请在“灰度传感器同时压到黑线（检测到路口）”时调用此函数！
 */
void On_Crossroad_Detected(void);

// ================= 4. 需要底层同学实现的外部接口 =================
// 提示：以下函数在 logic.c 中被调用，控制组的同学需要在自己的驱动文件里实现它们！

extern void Motor_Stop(void);             // 刹车停稳 (短暂刹车)
extern void Motor_Brake_And_Lock(void);   // 任务结束，电机抱死死循环
extern void Delay_ms(uint32_t ms);        // 毫秒延时

extern MaixCam_Data Get_Vision_Data(void); // 从串口读取最新的MaixCam识别结果

extern void Buzzer_Beep(uint32_t ms);               // 蜂鸣器长鸣
extern void Buzzer_Beep_Intermittent(uint32_t ms);  // 蜂鸣器断续鸣叫
extern void Buzzer_Off(void);                       // 关闭蜂鸣器

extern void LED_Red_On(void);             // 亮红灯
extern void LED_Green_On(void);           // 亮绿灯
extern void LED_Off(void);                // 关指示灯

/**
 * @brief 执行物理转弯动作
 * @param dir: TURN_LEFT 或 TURN_RIGHT
 * @note 控制组注意：执行转弯时必须是”原地自旋”，屏蔽一段灰度后，重新压中黑线才退出该函数！
 */
extern void Execute_Physical_Turn(int dir);

// ================= 5. 非阻塞决策接口 (供 main.c 状态机调用) =================

/**
 * @brief  设置摄像头识别结果缓存
 * @param  resultCode MaixCam 识别结果码 (SERIAL_MAIXCAM_RESULT_*)
 * @note   main.c 收到 MaixCam 数据后调用，供 Logic_Get_Turn_Direction 使用
 */
void Logic_SetVisionCache(uint8_t resultCode);

/**
 * @brief  非阻塞路口决策：根据摄像头颜色 + 拓扑地图决定转向方向
 * @param  cameraColor 摄像头识别的颜色 (COLOR_RED / COLOR_GREEN / COLOR_NONE)
 * @retval TURN_LEFT(0) 或 TURN_RIGHT(1): 正常转向方向
 * @retval -1: 已到达终点(O点)，任务应结束
 * @note   此函数同时更新图论状态 (edge_visited, last_node, curr_node 等)
 */
int Logic_Get_Turn_Direction(uint8_t cameraColor);

/**
 * @brief  重置 logic 全局状态 (任务开始/重新开始时调用)
 */
void Logic_Reset(void);

#endif /* __LOGIC_H */