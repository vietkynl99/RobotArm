#ifndef INC_ROBOTARM_H_
#define INC_ROBOTARM_H_

#include "main.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void print(const char *data);
EXTERNC void println(const char *data);

EXTERNC void uartRxEvent(char ch);

EXTERNC void setup(UART_HandleTypeDef *huart);
EXTERNC void loop();

#endif /* INC_ROBOTARM_H_ */
