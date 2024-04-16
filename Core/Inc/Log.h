#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void setupHandler(UART_HandleTypeDef *huart);

EXTERNC void print(const char *data);
EXTERNC void println(const char *data);

#endif /* INC_LOG_H_ */
