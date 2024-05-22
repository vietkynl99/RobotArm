#include "Log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 256

UART_HandleTypeDef *mHuart = NULL;
char buffer[BUFFER_SIZE];

void setUartHandler(UART_HandleTypeDef *huart)
{
    mHuart = huart;
}

void print(const char *pFormat, ...)
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

void println(const char *pFormat, ...)
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