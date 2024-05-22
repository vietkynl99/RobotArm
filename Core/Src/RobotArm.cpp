#include "RobotArm.h"
#include "CommandLine.h"
#include "Servo.h"
#include "DeviceController.h"

#include <string>
#include <cstring>

using namespace std;

#define SERVO_LOG_INTERVAL_MS (50)
#define SERVO_NUMS (6)

DeviceController *mDeviceController;
Servo *mServo[SERVO_NUMS];

void onUartDataReceived(char ch)
{
    CommandLine::onCharacterReceived(ch);
}

void onSpiDataReceived()
{
    if (mDeviceController)
    {
        mDeviceController->onDataReceived();
    }
}

void onSpiDataError()
{
    if (mDeviceController)
    {
        mDeviceController->onDataError();
    }
}

void onGpioExt(uint16_t pin)
{
    if (!mServo[5])
    {
        return;
    }
    if (pin == M1_E1_Pin)
    {
        mServo[0]->onEncoderEvent(!HAL_GPIO_ReadPin(M1_E2_GPIO_Port, M1_E2_Pin));
    }
    else if (pin == M2_E1_Pin)
    {
        mServo[1]->onEncoderEvent(!HAL_GPIO_ReadPin(M2_E2_GPIO_Port, M2_E2_Pin));
    }
    else if (pin == M3_E1_Pin)
    {
        mServo[2]->onEncoderEvent(!HAL_GPIO_ReadPin(M3_E2_GPIO_Port, M3_E2_Pin));
    }
    else if (pin == M4_E1_Pin)
    {
        mServo[3]->onEncoderEvent(!HAL_GPIO_ReadPin(M4_E2_GPIO_Port, M4_E2_Pin));
    }
    else if (pin == M5_E1_Pin)
    {
        mServo[4]->onEncoderEvent(!HAL_GPIO_ReadPin(M5_E2_GPIO_Port, M5_E2_Pin));
    }
    else if (pin == M6_E1_Pin)
    {
        mServo[5]->onEncoderEvent(!HAL_GPIO_ReadPin(M6_E2_GPIO_Port, M6_E2_Pin));
    }
}

void onZeroDetected(int index)
{
    // println("onZeroDetected: %d", index);
    if (index >= 0 && index < SERVO_NUMS)
    {
        mServo[index]->onZeroDectected();
    }
}

void onControllerInterrupt()
{
    if (!mServo[5])
    {
        return;
    }
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        mServo[i]->run();
    }
}

bool onCommandReboot(string params)
{
    if (!params.empty())
    {
        return false;
    }
    println("Rebooting...");
    HAL_NVIC_SystemReset();
    return true;
}

bool onCommandPosition(string params)
{
    if (params.empty())
    {
        println("Current position: %.6f", (float)mServo[0]->getCurrentPosition());
        return true;
    }
    else
    {
        float setpoint = 0;
        if (sscanf(params.c_str(), "%f", &setpoint) == 1)
        {
            mServo[0]->requestPosition(setpoint);
            return true;
        }
    }
    return false;
}

bool onCommandResetServo(string params)
{
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        mServo[i]->reset();
    }
    return true;
}

bool onCommandTune(string params)
{
    float kp = 0, ki = 0, kd = 0;
    int ret = sscanf(params.c_str(), "%f %f %f", &kp, &ki, &kd);
    if (ret >= 1 && ret <= 3)
    {
        println("PID tune: kp=%.2f ki=%.2f kd=%.2f", kp, ki, kd);
        PidParams params{kp, ki, kd};
        mServo[0]->tune(params);
        return true;
    }
    else
    {
        return false;
    }
}

bool onCommandEnable(string params)
{
    if (params == "on")
    {
        println("Servo is enabled");
        for (int i = 0; i < SERVO_NUMS; i++)
        {
            mServo[i]->setEnable(true);
        }
        return true;
    }
    else if (params == "off")
    {
        println("Servo is disabled");
        for (int i = 0; i < SERVO_NUMS; i++)
        {
            mServo[i]->setEnable(false);
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool onCommandZeroDetect(string params)
{
    if (!params.empty())
    {
        return false;
    }
    mServo[0]->zeroDetect();
    return true;
}

void setup(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    mDeviceController = new DeviceController(hspi);
    mServo[0] = new Servo(htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[1] = new Servo(htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[2] = new Servo(htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[3] = new Servo(htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[4] = new Servo(htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[5] = new Servo(htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);

    println("");
    println("*** Robot Arm ***");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("position", onCommandPosition, "position [value]\t: rotate the servo to position\r\nservo-position\t: get current position");
    CommandLine::install("reset", onCommandResetServo, "reset\t: reset servo data");
    CommandLine::install("tune", onCommandTune, "tune [kp] [ki] [kd]\t: set pid controller params");
    CommandLine::install("enable", onCommandEnable, "enable [on/off]\t: turn on/off servo");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect\t: Zero dectection");
}

void loop()
{
    mDeviceController->run();
}
