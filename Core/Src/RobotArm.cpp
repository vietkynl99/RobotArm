#include "BuildEnv.h"
#include "RobotArm.h"
#include "CommandLine.h"
#include "DeviceController.h"
#include "SoftI2c.h"
#include "MCP23017.h"

#include <string>
#include <cstring>

using namespace std;

SoftI2c *mI2c;
MCP23017 *mIOExpander;
DeviceController *mController;

int parseIndex(string params, int min = 0, int max = SERVO_NUMS - 1)
{
    int index;
    if (!params.empty() && sscanf(params.c_str(), "%d", &index) == 1 && index >= min && index <= max)
    {
        return index;
    }
    return -1;
}

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
    if (pin == IO_EXPANDER_INT_Pin)
    {
        if (mIOExpander)
        {
            mIOExpander->interrupt();
        }
    }
    else
    {
        if (mController)
        {
            mController->onGpioExt(pin);
        }
    }
}

void onExpanderGpioExt(MCP23017_Pin pin)
{
    if (mController)
    {
        mController->onExpanderGpioExt(pin);
    }
}

void onTimerInterrupt()
{
    if (mController)
    {
        mController->onTimerInterrupt();
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
    int index = 0, position = 0;
    if (!params.empty() && sscanf(params.c_str(), "%d %d", &index, &position) == 2 && index >= 0 && index < SERVO_NUMS)
    {
        mController->requestPosition(index, position);
        return true;
    }
    else if (!params.empty() && sscanf(params.c_str(), "%d", &index) == 1 && index >= 0 && index < SERVO_NUMS)
    {
        println("Current position: %.6f", mController->getCurrentPosition(index));
        return true;
    }
    return false;
}

bool onCommandZeroDetect(string params)
{
    if (!mIOExpander->isRunning())
    {
        println("IO expander error");
        return false;
    }
    int index = parseIndex(params);
    if (index != -1)
    {
        mController->startZeroDetection(index);
        return true;
    }
    return false;
}

bool onCommandTune(string params)
{
    int index = 0;
    float kp, ki, kd;
    if (!params.empty() && sscanf(params.c_str(), "%d %f %f %f", &index, &kp, &ki, &kd) == 4 && index >= 0 && index < SERVO_NUMS)
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
        index >= 0 && index < SERVO_NUMS &&
        pwm >= -SERVO_PWM_RESOLUTION && pwm <= SERVO_PWM_RESOLUTION)
    {
        mController->forceOutput(index, pwm);
        return true;
    }
    return false;
}

bool onCommandEnable(string params)
{
    int index = parseIndex(params);
    if (index != -1)
    {
        mController->enableServo(index);
        return true;
    }
    return false;
}

bool onCommandDisable(string params)
{
    int index = parseIndex(params);
    if (index != -1)
    {
        mController->disableServo(index);
        return true;
    }
    return false;
}

bool onCommandReset(string params)
{
    int index = parseIndex(params);
    if (index != -1)
    {
        mController->reset(index);
        return true;
    }
    return false;
}

bool onCommandDebug(string params)
{
    int index = parseIndex(params);
    if (index != -1)
    {
        mController->debugMotor(index);
        return true;
    }
    return false;
}

bool onCommandMonitor(string params)
{
    int index = parseIndex(params, -1, SERVO_NUMS - 1);
    if (index != -1)
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
    CommandLine::install("position", onCommandPosition, "position [index] [value]\t: rotate the servo to position\r\nposition [index]\t: get current position");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect [index]\t: Zero dectection");
    CommandLine::install("tune", onCommandTune, "tune [index] [kp] [ki] [kd] \t: tune PID params");
    CommandLine::install("enable", onCommandEnable, "enable [index]\t: enable servo at index");
    CommandLine::install("disable", onCommandDisable, "disable [index]\t: disable servo at index");
    CommandLine::install("reset", onCommandReset, "reset [index]\t: reset servo at index");
    CommandLine::install("forceOutput", onCommandForceOutput, "forceOutput [index] [pwm]\t: Force servo to run with pwm value");
    CommandLine::install("debug", onCommandDebug, "debug [index]\t: debug servo at index");
    CommandLine::install("monitor", onCommandMonitor, "monitor -1\t: stop monitor\r\nmonitor [index]\t: monitor servo position at index");

    mI2c = new SoftI2c(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin, SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin);
    if (!mI2c->begin())
    {
        println("Failed to initialize I2C");
    }
    else
    {
        mIOExpander = new MCP23017(mI2c, MCP23017_ADDRESS, onExpanderGpioExt);
        for (int i = 0; i < 5; i++)
        {
            if (mIOExpander->init())
            {
                println("IO expander initialized");
                break;
            }
            println("Failed to initialize IO expander. Retrying...");
            HAL_Delay(100);
        }
    }

    mController = new DeviceController(htim1, htim2, htim3, hspi);
}

void loop()
{
    if (mIOExpander)
    {
        mIOExpander->run();
    }
    mController->run();
}
