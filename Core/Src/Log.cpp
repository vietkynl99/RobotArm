#include "Log.h"
// #include "stdio.h"
#include "string.h"

UART_HandleTypeDef *mHuart = nullptr;

void setupHandler(UART_HandleTypeDef *huart)
{
    mHuart = huart;
}

void print(const char *data)
{
    if (mHuart && strlen(data) > 0)
    {
        HAL_UART_Transmit(mHuart, (uint8_t *)data, strlen(data), 1000);
    }
}

void println(const char *data)
{
    print(data);
    print("\r\n");
}