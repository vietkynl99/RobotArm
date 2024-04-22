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

bool onCommandGetCurrentPosition(string params)
{
    if (mServo)
    {
        println("Current position: %.6f", (float)mServo->getCurrentPosition());
    }
    return true;
}

bool onCommandSetpoint(string params)
{
    if (mServo)
    {
        long value = stoi(params);
        println("Set setpoint to %d", value);
        mServo->requestPosition(value);
    }
    return true;
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

void setup(TIM_HandleTypeDef *htim)
{
    mServo = new Servo(htim, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, 30, 0);

    println("");
    println("*****************");
    println("*** Robot Arm ***");
    println("*****************");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("plot", onCommandPlot, "plot [on/off]\t: turn on/off servo motor plotter");
    CommandLine::install("servo-position", onCommandGetCurrentPosition);
    CommandLine::install("servo-setpoint", onCommandSetpoint);
    CommandLine::install("servo-reset", onCommandResetServo);
    CommandLine::install("servo-tune", onCommandTune, "servo-tune [kp] [ki] [kd]\t: set pid controller params");

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
