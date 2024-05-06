#include "RobotArm.h"
#include "CommandLine.h"
#include "Servo.h"

#include <string>
#include <cstring>

using namespace std;

#define SERVO_LOG_INTERVAL_MS (50)

Servo *mServo;
bool mLogEnabled = false;

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

void onUartDataReceived(char ch)
{
    CommandLine::onCharacterReceived(ch);
}

void onGpioExt(uint16_t pin)
{
    if (pin == M1_E1_Pin)
    {
        if (mServo)
        {
            mServo->onEncoderEvent(!HAL_GPIO_ReadPin(M1_E2_GPIO_Port, M1_E2_Pin));
        }
    }
}

void onZeroDetected(int index)
{
    if (index == 0)
    {
        mServo->onZeroDectected();
    }
}

void onControllerInterrupt()
{
    if (mServo)
    {
        mServo->run();
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
        println("Current position: %.6f", (float)mServo->getCurrentPosition());
        return true;
    }
    else
    {
        float setpoint = 0;
        if (sscanf(params.c_str(), "%f", &setpoint) == 1)
        {
            mServo->requestPosition(setpoint);
            return true;
        }
    }
    return false;
}

bool onCommandResetServo(string params)
{
    if (mServo)
    {
        mServo->reset();
    }
    return true;
}

bool onCommandPlot(string params)
{
    if (params == "on")
    {
        if (!mLogEnabled)
        {
            mLogEnabled = true;
            printlnLog("RequestedPosition,CurrentPosition,ControlValue");
        }
        return true;
    }
    else if (params == "off")
    {
        mLogEnabled = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool onCommandTune(string params)
{
    float kp = 0, ki = 0, kd = 0;
    int ret = sscanf(params.c_str(), "%f %f %f", &kp, &ki, &kd);
    if (ret >= 1 && ret <= 3)
    {
        println("PID tune: kp=%.2f ki=%.2f kd=%.2f", kp, ki, kd);
        PidParams params{kp, ki, kd};
        mServo->tune(params);
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
        mServo->setEnable(true);
        return true;
    }
    else if (params == "off")
    {
        println("Servo is disabled");
        mServo->setEnable(false);
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
    mServo->zeroDetect();
    return true;
}

void setup(TIM_HandleTypeDef *htim)
{
    mServo = new Servo(htim, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775 * 4, true, -160, 170, 180, 30, 0);

    println("");
    println("*** Robot Arm ***");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("plot", onCommandPlot, "plot [on/off]\t: turn on/off servo motor plotter");
    CommandLine::install("position", onCommandPosition, "position [value]\t: rotate the servo to position\r\nservo-position\t: get current position");
    CommandLine::install("reset", onCommandResetServo, "reset\t: reset servo data");
    CommandLine::install("tune", onCommandTune, "tune [kp] [ki] [kd]\t: set pid controller params");
    CommandLine::install("enable", onCommandEnable, "enable [on/off]\t: turn on/off servo");
    CommandLine::install("zero-detect", onCommandZeroDetect, "zero-detect\t: Zero dectection");

    blinkLed(LED_GPIO_Port, LED_Pin, 3);
}

void loop()
{
    static uint32_t timeTick = 0;

    if (mLogEnabled && HAL_GetTick() > timeTick)
    {
        timeTick = HAL_GetTick() + SERVO_LOG_INTERVAL_MS;
        printlnLog("%.2f %.2f %.2f", mServo->getRequestedPosition(), mServo->getCurrentPosition(), 100 * mServo->getControlValue() / SERVO_PWM_RESOLUTION);
    }
}
