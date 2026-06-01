/**
 * @file    menu.h
 * @brief   OLED 树状菜单系统
 * @details 基于4按键(K1=下移,K3=上移,K2=确认,K4=返回)的层级菜单导航。
 *          支持节点(展开子菜单)和叶子(执行回调)两种菜单项类型。
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
    MENU_LEAF       /* 可执行的叶子动作 */
} MenuItemType_t;

/* ===== 回调函数类型 ===== */
typedef void (*MenuAction_cb)(void);

/* ===== 菜单项结构 ===== */
typedef struct {
    const char      *name;          /* 显示名称(建议≤16字符) */
    MenuItemType_t   type;          /* NODE 或 LEAF */
    MenuAction_cb    action;        /* LEAF时的回调, NODE时为NULL */
    uint8_t          childStart;    /* NODE: 子菜单在数组中的起始索引 */
    uint8_t          childCount;    /* NODE: 子菜单项数 */
} MenuItem_t;

/* ===== 系统运行模式 ===== */
typedef enum {
    SYS_MENU = 0,   /* 菜单导航模式 */
    SYS_TASK_RUN,   /* 任务运行模式 */
    SYS_MONITOR,    /* 实时监测模式 */
    SYS_PID_EDIT,   /* PID 调参模式 */
    SYS_GIMBAL_SETZERO /* 步进电机回零点设置模式 */
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
 * @brief  菜单系统初始化
 */
void Menu_Init(void);

/**
 * @brief  按键处理(主循环调用)
 * @param  key 按键值(1=K1,2=K2,3=K3,4=K4)
 */
void Menu_ProcessKey(uint8_t key);

/**
 * @brief  菜单渲染(主循环调用, 100ms周期)
 */
void Menu_Render(void);

/**
 * @brief  获取当前系统模式
 */
SystemMode_t Menu_GetMode(void);

/**
 * @brief  从任意运行态退回菜单
 */
void Menu_ExitToMenu(void);

/**
 * @brief  获取当前PID编辑页面
 */
PIDEditPage_t Menu_GetPIDEditPage(void);

/**
 * @brief  设置菜单模式(供回调函数切换到运行态)
 */
void Menu_SetMode(SystemMode_t mode);

/**
 * @brief  设置PID编辑子页面
 */
void Menu_SetPIDEditPage(PIDEditPage_t page);

#ifdef __cplusplus
}
#endif

#endif /* MENU_H_ */
