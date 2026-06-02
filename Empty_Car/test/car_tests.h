#ifndef CAR_TESTS_H_
#define CAR_TESTS_H_

#include <stdint.h>

typedef struct {
    uint16_t flashId;
    uint8_t flashResult;
    uint8_t grayLeftRaw;
    uint8_t grayRightRaw;
    uint8_t grayStatus;
} CarTestState_t;

void CarTest_FlashRun(CarTestState_t *state);
void CarTest_RenderFlash(const CarTestState_t *state);
void CarTest_RenderGray(CarTestState_t *state);
void CarTest_CameraEnter(void);
void CarTest_CameraLook(void);
void CarTest_CameraExit(void);
void CarTest_RenderCamera(void);
void CarTest_RenderOledChinese(void);
void CarTest_RenderBuzzerLed(void);

#endif /* CAR_TESTS_H_ */
