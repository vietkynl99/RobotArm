#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "main.h"
#include "usbd_cdc_if.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void setUartLogHandler(UART_HandleTypeDef *huart);

EXTERNC void uartPrint(const char *pFormat, ...);
EXTERNC void uartPrintln(const char *pFormat, ...);
EXTERNC void usbPrint(const char *pFormat, ...);
EXTERNC void usbPrintln(const char *pFormat, ...);

#endif /* INC_LOG_H_ */
