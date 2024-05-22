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
EXTERNC void onSpiDataReceived();
EXTERNC void onSpiDataError();
EXTERNC void onGpioExt(uint16_t pin);
EXTERNC void onZeroDetected(int index);
EXTERNC void onControllerInterrupt();

EXTERNC void setup(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi);
EXTERNC void loop();

#endif /* INC_ROBOTARM_H_ */
