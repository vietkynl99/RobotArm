#include "BuildEnv.h"
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
    if (!mController->isMonitoring())
    {
        println("You need to set debug index first.");
        return true;
    }
    if (params.empty())
    {
        println("Current position: %.6f", mController->getCurrentPosition(mController->getMonitorIndex()));
        return true;
    }
    else
    {
        float setpoint = 0;
        if (sscanf(params.c_str(), "%f", &setpoint) == 1)
        {
            mController->requestPosition(mController->getMonitorIndex(), setpoint);
            return true;
        }
    }
    return false;
}

bool onCommandZeroDetect(string params)
{
    if (!mController->isMonitoring())
    {
        println("You need to set debug index first.");
        return true;
    }
    if (!params.empty())
    {
        return false;
    }
    mController->startZeroDetection(mController->getMonitorIndex());
    return true;
}

bool onCommandTune(string params)
{
    int index = 0;
    float kp, ki, kd;
    if (!params.empty() && sscanf(params.c_str(), "%d %f %f %f", &index, &kp, &ki, &kd) == 4 && index >= 0 && index <= SERVO_NUMS)
    {
        mController->tune(index, PidParams{kp, ki, kd});
        return true;
    }
    return false;
}

bool onCommandForceOutput(string params)
{
    int index = 0, pwm = 0;
    if (!params.empty() && sscanf(params.c_str(), "%d %d", &index, &pwm) == 2 &&
        index >= 0 && index <= SERVO_NUMS &&
        pwm >= 0 && pwm <= SERVO_PWM_RESOLUTION)
    {
        mController->forceOutput(index, pwm);
        return true;
    }
    return false;
}

bool onCommandDisable(string params)
{
    int index = 0;
    if (!params.empty() && sscanf(params.c_str(), "%d", &index) == 1 && index >= 0 && index <= SERVO_NUMS)
    {
        mController->disableServo(index);
        return true;
    }
    return false;
}

bool onCommandDebug(string params)
{
    int index = 0;
    if (!params.empty() && sscanf(params.c_str(), "%d", &index) == 1 && index >= 0 && index <= SERVO_NUMS)
    {
        mController->debugMotor(index);
        return true;
    }
    return false;
}

bool onCommandMonitor(string params)
{
    int index = 0;
    if (!params.empty() && sscanf(params.c_str(), "%d", &index) == 1 && index >= -1 && index <= SERVO_NUMS)
    {
        if (index == -1)
        {
            mController->stopMonitor();
        }
        else
        {
            mController->startMonitor(index);
        }
        return true;
    }
    return false;
}

void setup(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    println("");
    println("*** Robot Arm - ver " GIT_VERSION " ***");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("position", onCommandPosition, "position [value]\t: rotate the servo to position\r\nservo-position\t: get current position");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect\t: Zero dectection");
    CommandLine::install("tune", onCommandTune, "tune [index] [kp] [ki] [kd] \t: tune PID params");
    CommandLine::install("disable", onCommandDisable, "disable [index]\t: disable servo at index");
    CommandLine::install("forceOutput", onCommandForceOutput, "forceOutput [index] [pwm]\t: Force servo to run with pwm value. Servo needs to be disabled first.");
    CommandLine::install("debug", onCommandDebug, "debug [index]\t: debug servo at index");
    CommandLine::install("monitor", onCommandMonitor, "monitor -1\t: stop monitor\r\nmonitor [index]\t: monitor servo position at index");

    mController = new DeviceController(htim1, htim2, htim3, hspi);
}

void loop()
{
    mController->run();
}
