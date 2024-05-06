#ifndef INC_ROBOTARM_H_
#define INC_ROBOTARM_H_

#include "main.h"
#include "Log.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void onUartDataReceived(char ch);
EXTERNC void onGpioExt(uint16_t pin);
EXTERNC void onZeroDetected(int index);
EXTERNC void onControllerInterrupt();

EXTERNC void setup(TIM_HandleTypeDef *htim);
EXTERNC void loop();

#endif /* INC_ROBOTARM_H_ */
