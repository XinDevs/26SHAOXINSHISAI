/**
 * @file    menu.h
 * @brief   OLED 树状菜单系统
 * @details 基于 4 个按键的层级菜单导航，支持节点菜单和叶子动作。
 *          K1=上移，K3=下移，K2=确认，K4=返回。
 */
#ifndef MENU_H_
#define MENU_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 菜单项类型 ===== */
typedef enum {
    MENU_NODE = 0,  /* 包含子菜单的节点 */
    MENU_LEAF       /* 可执行动作的叶子项 */
} MenuItemType_t;

/* ===== 回调函数类型 ===== */
typedef void (*MenuAction_cb)(void);

/* ===== 菜单项结构 ===== */
typedef struct {
    const char      *name;          /* 显示名称 */
    MenuItemType_t   type;          /* NODE 或 LEAF */
    MenuAction_cb    action;        /* LEAF 的回调，NODE 为 NULL */
    uint8_t          childStart;    /* NODE: 子菜单在数组中的起始索引 */
    uint8_t          childCount;    /* NODE: 子菜单项数量 */
} MenuItem_t;

/* ===== 系统运行模式 ===== */
typedef enum {
    SYS_MENU = 0,      /* 菜单导航模式 */
    SYS_TASK_RUN,      /* 任务运行模式 */
    SYS_PATROL_INFO,   /* 巡检信息页面 */
    SYS_MONITOR,       /* 实时监测模式 */
    SYS_PID_EDIT,      /* PID 调参模式 */
    SYS_FLASH_TEST,    /* Flash 测试页面 */
    SYS_GRAY_TEST,     /* 灰度测试页面 */
    SYS_CAMERA_TEST,   /* 摄像头测试页面 */
    SYS_OLED_CN_TEST,  /* OLED 中文测试页面 */
    SYS_BUZZER_LED_TEST /* 蜂鸣器和 LED 测试页面 */
} SystemMode_t;

/* ===== PID 编辑页面 ===== */
typedef enum {
    PID_EDIT_LEFT = 0,
    PID_EDIT_RIGHT,
    PID_EDIT_GRAY,
    PID_EDIT_YAW
} PIDEditPage_t;

/* ===== API ===== */

/**
 * @brief  初始化菜单系统
 */
void Menu_Init(void);

/**
 * @brief  按键处理，主循环调用
 * @param  key 按键值: 1=K1, 2=K2, 3=K3, 4=K4
 */
void Menu_ProcessKey(uint8_t key);

/**
 * @brief  菜单渲染，主循环按 100ms 周期调用
 */
void Menu_Render(void);

/**
 * @brief  获取当前系统模式
 */
SystemMode_t Menu_GetMode(void);

/**
 * @brief  从任意运行状态退回主菜单
 */
void Menu_ExitToMenu(void);

/**
 * @brief  获取当前 PID 编辑页面
 */
PIDEditPage_t Menu_GetPIDEditPage(void);

/**
 * @brief  设置菜单模式，供回调函数切换状态
 */
void Menu_SetMode(SystemMode_t mode);

/**
 * @brief  设置 PID 编辑子页面
 */
void Menu_SetPIDEditPage(PIDEditPage_t page);

#ifdef __cplusplus
}
#endif

#endif /* MENU_H_ */
