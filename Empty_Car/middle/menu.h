/**
 * @file    menu.h
 * @brief   OLED 鏍戠姸鑿滃崟绯荤粺
 * @details 鍩轰簬4鎸夐敭(K1=涓嬬Щ,K3=涓婄Щ,K2=纭,K4=杩斿洖)鐨勫眰绾ц彍鍗曞鑸€? *          鏀寔鑺傜偣(灞曞紑瀛愯彍鍗?鍜屽彾瀛?鎵ц鍥炶皟)涓ょ鑿滃崟椤圭被鍨嬨€? */
#ifndef MENU_H_
#define MENU_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 鑿滃崟椤圭被鍨?===== */
typedef enum {
    MENU_NODE = 0,  /* 鍖呭惈瀛愯彍鍗曠殑鑺傜偣 */
    MENU_LEAF       /* 鍙墽琛岀殑鍙跺瓙鍔ㄤ綔 */
} MenuItemType_t;

/* ===== 鍥炶皟鍑芥暟绫诲瀷 ===== */
typedef void (*MenuAction_cb)(void);

/* ===== 鑿滃崟椤圭粨鏋?===== */
typedef struct {
    const char      *name;          /* 鏄剧ず鍚嶇О(寤鸿鈮?6瀛楃) */
    MenuItemType_t   type;          /* NODE 鎴?LEAF */
    MenuAction_cb    action;        /* LEAF鏃剁殑鍥炶皟, NODE鏃朵负NULL */
    uint8_t          childStart;    /* NODE: 瀛愯彍鍗曞湪鏁扮粍涓殑璧峰绱㈠紩 */
    uint8_t          childCount;    /* NODE: 瀛愯彍鍗曢」鏁?*/
} MenuItem_t;

/* ===== 绯荤粺杩愯妯″紡 ===== */
typedef enum {
    SYS_MENU = 0,   /* 鑿滃崟瀵艰埅妯″紡 */
    SYS_TASK_RUN,   /* 浠诲姟杩愯妯″紡 */
    SYS_MONITOR,    /* 瀹炴椂鐩戞祴妯″紡 */
    SYS_PID_EDIT,   /* PID 璋冨弬妯″紡 */
    SYS_FLASH_TEST,   /* Flash test result */
    SYS_GRAY_TEST,    /* Gray test */
    SYS_CAMERA_TEST,  /* Camera test */
    SYS_OLED_CN_TEST  /* OLED Chinese test */
} SystemMode_t;

/* ===== PID 缂栬緫椤甸潰 ===== */
typedef enum {
    PID_EDIT_LEFT = 0,
    PID_EDIT_RIGHT,
    PID_EDIT_GRAY,
    PID_EDIT_YAW
} PIDEditPage_t;

/* ===== API ===== */

/**
 * @brief  鑿滃崟绯荤粺鍒濆鍖? */
void Menu_Init(void);

/**
 * @brief  鎸夐敭澶勭悊(涓诲惊鐜皟鐢?
 * @param  key 鎸夐敭鍊?1=K1,2=K2,3=K3,4=K4)
 */
void Menu_ProcessKey(uint8_t key);

/**
 * @brief  鑿滃崟娓叉煋(涓诲惊鐜皟鐢? 100ms鍛ㄦ湡)
 */
void Menu_Render(void);

/**
 * @brief  鑾峰彇褰撳墠绯荤粺妯″紡
 */
SystemMode_t Menu_GetMode(void);

/**
 * @brief  浠庝换鎰忚繍琛屾€侀€€鍥炶彍鍗? */
void Menu_ExitToMenu(void);

/**
 * @brief  鑾峰彇褰撳墠PID缂栬緫椤甸潰
 */
PIDEditPage_t Menu_GetPIDEditPage(void);

/**
 * @brief  璁剧疆鑿滃崟妯″紡(渚涘洖璋冨嚱鏁板垏鎹㈠埌杩愯鎬?
 */
void Menu_SetMode(SystemMode_t mode);

/**
 * @brief  璁剧疆PID缂栬緫瀛愰〉闈? */
void Menu_SetPIDEditPage(PIDEditPage_t page);

#ifdef __cplusplus
}
#endif

#endif /* MENU_H_ */


