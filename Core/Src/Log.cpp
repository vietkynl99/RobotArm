#include "Log.h"
#include <stdarg.h>
#include "string.h"

#define BUFFER_SIZE 256

UART_HandleTypeDef *mHuart = nullptr;
char buffer[BUFFER_SIZE];

void setUartLogHandler(UART_HandleTypeDef *huart)
{
    mHuart = huart;
}

void uartPrint(const char *pFormat, ...)
{
    if (mHuart)
    {
        va_list pVlist;
        va_start(pVlist, pFormat);
        vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
        va_end(pVlist);

        HAL_UART_Transmit(mHuart, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}

void uartPrintln(const char *pFormat, ...)
{
    if (mHuart)
    {
        va_list pVlist;
        va_start(pVlist, pFormat);
        vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
        va_end(pVlist);

        if (strlen(buffer) + 2 < BUFFER_SIZE)
        {
            strcat(buffer, "\r\n");
        }
        HAL_UART_Transmit(mHuart, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}

void usbPut(const char *str)
{
    for (int i = 0; i < 3; i++)
    {
        if (CDC_Transmit_FS((uint8_t *)str, strlen(str)) == USBD_OK)
        {
            return;
        }
        HAL_Delay(1);
    }
}

void usbPrint(const char *pFormat, ...)
{
    va_list pVlist;
    va_start(pVlist, pFormat);
    vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
    va_end(pVlist);

    usbPut(buffer);
}

void usbPrintln(const char *pFormat, ...)
{
    va_list pVlist;
    va_start(pVlist, pFormat);
    vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
    va_end(pVlist);

    if (strlen(buffer) + 2 < BUFFER_SIZE)
    {
        strcat(buffer, "\r\n");
    }
    usbPut(buffer);
}