#include "RobotArm.h"
#include "CommandLine.h"

#include <string>
#include <cstring>

UART_HandleTypeDef *mHuart = nullptr;

using namespace std;

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

void blinkLed(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, int count)
{
    for (int i = 0; i < count; i++)
    {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        HAL_Delay(60);
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
        HAL_Delay(60);
    }
}

void uartRxEvent(char ch)
{
    CommandLine::onCharacterReceived(ch);
}

void setup(UART_HandleTypeDef *huart)
{
    mHuart = huart;
    println("Robot Arm");

    CommandLine::init();

    blinkLed(LED_GPIO_Port, LED_Pin, 3);

}

void loop()
{
}
