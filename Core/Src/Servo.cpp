#include "Servo.h"
#include <stdio.h>

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double gearRatio, double kp, double ki, double kd)
{
    double resolution = 360 / gearRatio;

    mOutputTimer = outputTimer;
    mOutputTimerCh1 = outputTimerCh1;
    mOutputTimerCh2 = outputTimerCh2;
    mEncoderResolution = resolution / SERVO_ENCODER_RESOLUTION;

    PidParams params{kp / resolution, ki / resolution, kd / resolution};
    mPidController = new PidController(&mError, &mOutput, SERVO_SAMPLE_TIME_S, params, -SERVO_PWM_RESOLUTION, SERVO_PWM_RESOLUTION);

    reset();
}

Servo::~Servo()
{
    if (mPidController)
    {
        delete mPidController;
        mPidController = nullptr;
    }
    if (mOutputTimer)
    {
        delete mOutputTimer;
        mOutputTimer = nullptr;
    }
}

void Servo::onEncoderEvent(bool direction)
{
    if (direction)
    {
        mEncoderPulse++;
    }
    else
    {
        mEncoderPulse--;
    }
}

void Servo::run()
{
    mError = mSetpoint - getCurrentPosition();
    mPidController->run();

    double fixedOutput = mOutput;
    if (fixedOutput > -SERVO_FIXED_PWN_IN && fixedOutput < SERVO_FIXED_PWN_IN)
    {
        fixedOutput = map(fixedOutput, -SERVO_FIXED_PWN_IN, SERVO_FIXED_PWN_IN, -SERVO_FIXED_PWN_OUT, SERVO_FIXED_PWN_OUT);
    }
    else if (fixedOutput > 0)
    {
        fixedOutput = map(fixedOutput, SERVO_FIXED_PWN_IN, SERVO_PWM_RESOLUTION, SERVO_FIXED_PWN_OUT, SERVO_PWM_RESOLUTION);
    }
    else
    {
        fixedOutput = map(fixedOutput, -SERVO_PWM_RESOLUTION, -SERVO_FIXED_PWN_IN, -SERVO_PWM_RESOLUTION, -SERVO_FIXED_PWN_OUT);
    }

    if (fixedOutput > 0)
    {
        __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh1, fixedOutput);
        __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh1, 0);
        __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh2, -fixedOutput);
    }
}

void Servo::reset()
{
    mEncoderPulse = 0;
    mSetpoint = 0;
    mOutput = 0;
    mError = 0;
    mFeedback = 0;

    mPidController->reset();
}

double Servo::map(double input, double inMin, double inMax, double outMin, double outMax)
{
    double result = outMin + (input - inMin) * (outMax - outMin) / (inMax - inMin);
    if (result < outMin)
    {
        return outMin;
    }
    if (result > outMax)
    {
        return outMax;
    }
    return result;
}

// Unit: degree
void Servo::requestPosition(double postion)
{
    mSetpoint = postion;
}

// Unit: degree
double Servo::getRequestedPosition()
{
    return mSetpoint;
}

// Unit: degree
double Servo::getCurrentPosition()
{
    return mEncoderPulse * mEncoderResolution;
}

// Range: [-MOTOR_PWM_RESOLUTION, MOTOR_PWM_RESOLUTION]
double Servo::getControlValue()
{
    return mOutput;
}