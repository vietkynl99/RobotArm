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
    if (params.empty())
    {
        println("Current position: %.6f", mController->getCurrentPosition(0));
        return true;
    }
    else
    {
        float setpoint = 0;
        if (sscanf(params.c_str(), "%f", &setpoint) == 1)
        {
            mController->requestPosition(0, setpoint);
            return true;
        }
    }
    return false;
}

bool onCommandZeroDetect(string params)
{
    if (!params.empty())
    {
        return false;
    }
    mController->startZeroDetection(0);
    return true;
}

void setup(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    println("");
    println("*** Robot Arm ***");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("position", onCommandPosition, "position [value]\t: rotate the servo to position\r\nservo-position\t: get current position");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect\t: Zero dectection");

    mController = new DeviceController(htim1, htim2, htim3, hspi);
}

void loop()
{
    mController->run();
}
