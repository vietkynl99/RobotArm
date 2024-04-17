#include "Log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 256

UART_HandleTypeDef *mCmdHuart = NULL, *mLogHuart = NULL;
char buffer[BUFFER_SIZE];

void setCmdUartHandler(UART_HandleTypeDef *huart)
{
    mCmdHuart = huart;
}

void setLogUartHandler(UART_HandleTypeDef *huart)
{
    mLogHuart = huart;
}

void print(const char *pFormat, ...)
{
    if (mCmdHuart)
    {
        va_list pVlist;
        va_start(pVlist, pFormat);
        vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
        va_end(pVlist);

        HAL_UART_Transmit(mCmdHuart, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}

void println(const char *pFormat, ...)
{
    if (mCmdHuart)
    {
        va_list pVlist;
        va_start(pVlist, pFormat);
        vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
        va_end(pVlist);

        if (strlen(buffer) + 2 < BUFFER_SIZE)
        {
            strcat(buffer, "\r\n");
        }
        HAL_UART_Transmit(mCmdHuart, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}

void printlnLog(const char *pFormat, ...)
{
    if (mLogHuart)
    {
        va_list pVlist;
        va_start(pVlist, pFormat);
        vsnprintf(buffer, sizeof(buffer) - 1, pFormat, pVlist);
        va_end(pVlist);

        if (strlen(buffer) + 2 < BUFFER_SIZE)
        {
            strcat(buffer, "\r\n");
        }
        HAL_UART_Transmit(mLogHuart, (uint8_t *)buffer, strlen(buffer), 1000);
    }
}