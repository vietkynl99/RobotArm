#include "RobotArm.h"
#include "CommandLine.h"
#include "DeviceController.h"

#include <string>
#include <cstring>

using namespace std;

DeviceController *mController;

void onUartDataReceived(char ch)
{
    CommandLine::onCharacterReceived(ch);
}

void onSpiDataReceived()
{
    if (mController)
    {
        mController->onDataReceived();
    }
}

void onSpiDataError()
{
    if (mController)
    {
        mController->onDataError();
    }
}

void onGpioExt(uint16_t pin)
{
    if (mController)
    {
        mController->onEncoderEvent(pin);
    }
}

void onZeroDetected(int index)
{
    if (mController)
    {
        mController->onZeroDetected(index);
    }
}

void onControllerInterrupt()
{
    if (mController)
    {
        mController->onControllerInterrupt();
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
    // if (params.empty())
    // {
    //     println("Current position: %.6f", (float)mServo[0]->getCurrentPosition());
    //     return true;
    // }
    // else
    // {
    //     float setpoint = 0;
    //     if (sscanf(params.c_str(), "%f", &setpoint) == 1)
    //     {
    //         mServo[0]->requestPosition(setpoint);
    //         return true;
    //     }
    // }
    return false;
}

bool onCommandResetServo(string params)
{
    // for (int i = 0; i < SERVO_NUMS; i++)
    // {
    //     mServo[i]->reset();
    // }
    return true;
}

bool onCommandTune(string params)
{
    // float kp = 0, ki = 0, kd = 0;
    // int ret = sscanf(params.c_str(), "%f %f %f", &kp, &ki, &kd);
    // if (ret >= 1 && ret <= 3)
    // {
    //     println("PID tune: kp=%.2f ki=%.2f kd=%.2f", kp, ki, kd);
    //     PidParams params{kp, ki, kd};
    //     mServo[0]->tune(params);
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
	return false;
}

bool onCommandEnable(string params)
{
    // if (params == "on")
    // {
    //     println("Servo is enabled");
    //     for (int i = 0; i < SERVO_NUMS; i++)
    //     {
    //         mServo[i]->setEnable(true);
    //     }
    //     return true;
    // }
    // else if (params == "off")
    // {
    //     println("Servo is disabled");
    //     for (int i = 0; i < SERVO_NUMS; i++)
    //     {
    //         mServo[i]->setEnable(false);
    //     }
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
	return false;
}

bool onCommandZeroDetect(string params)
{
    // if (!params.empty())
    // {
    //     return false;
    // }
    // mServo[0]->zeroDetect();
    return true;
}

void setup(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    println("");
    println("*** Robot Arm ***");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("position", onCommandPosition, "position [value]\t: rotate the servo to position\r\nservo-position\t: get current position");
    CommandLine::install("reset", onCommandResetServo, "reset\t: reset servo data");
    CommandLine::install("tune", onCommandTune, "tune [kp] [ki] [kd]\t: set pid controller params");
    CommandLine::install("enable", onCommandEnable, "enable [on/off]\t: turn on/off servo");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect\t: Zero dectection");

    mController = new DeviceController(htim1, htim2, htim3, hspi);
}

void loop()
{
    mController->run();
}
