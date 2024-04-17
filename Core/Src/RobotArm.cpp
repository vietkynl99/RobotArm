#include "RobotArm.h"
#include "CommandLine.h"
#include "Servo.h"

#include <string>
#include <cstring>

using namespace std;

#define MOTOR_SAMPLE_TIME_S (1E-3)   // must matched with timer interrupt
#define MOTOR_PWM_RESOLUTION (999) // must matched with timer pwm generator
#define MOTOR_ENCODER_RESOLUTION (2) // pulse per revolution
#define MOTOR_LOG_INTERVAL_MS (50)

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
    if (pin == MOTOR1_E1_Pin)
    {
        if (mServo)
        {
            mServo->onEncoderEvent(true);
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
        println("Current position: %.2f", (float)mServo->getCurrentPosition());
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

bool onCommandTest(string params)
{
    if (mServo)
    {
        mServo->reset();

        long value = stoi(params);
        println("Set setpoint to %d", value);
        mServo->requestPosition(value);
    }
    return true;
}

bool onCommandLog(string params)
{
    if (params == "on")
    {
        mLogEnabled = true;
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

void setup(TIM_HandleTypeDef *htim)
{
    mServo = new Servo(htim, TIM_CHANNEL_1, TIM_CHANNEL_2, MOTOR_SAMPLE_TIME_S, MOTOR_ENCODER_RESOLUTION, MOTOR_PWM_RESOLUTION);

    println("");
    println("*****************");
    println("*** Robot Arm ***");
    println("*****************");
    CommandLine::init();
    CommandLine::install("reboot", onCommandReboot, "reboot\t: reboot device");
    CommandLine::install("log", onCommandLog, "log [on/off]\t: turn on/off servo motor log");
    CommandLine::install("servo-position", onCommandGetCurrentPosition);
    CommandLine::install("servo-setpoint", onCommandSetpoint);
    CommandLine::install("servo-reset", onCommandResetServo);
    CommandLine::install("servo-test", onCommandTest);

    blinkLed(LED_GPIO_Port, LED_Pin, 3);
}

void loop()
{
    static uint32_t timeTick = 0;

    if (mLogEnabled && HAL_GetTick() > timeTick)
    {
        timeTick = HAL_GetTick() + MOTOR_LOG_INTERVAL_MS;
        printlnLog("%.2f %.2f %.2f", mServo->getRequestedPosition(), mServo->getCurrentPosition(), mServo->getControlValue() / MOTOR_PWM_RESOLUTION);
    }
}
