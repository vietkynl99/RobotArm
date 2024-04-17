#include "Servo.h"

#include <stdio.h>

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double sampleTime, uint16_t pulsePerRev)
{
    // The motor rotates 100 revolutions and the shaft will rotate 360 ​​degrees.
    // double gearRatio = 360.0 / 100;
    double gearRatio = 1.0 / 100;

    mOutputTimer = outputTimer;
    mOutputTimerCh1 = outputTimerCh1;
    mOutputTimerCh2 = outputTimerCh2;
    mEncoderResolution = gearRatio / pulsePerRev;

    double kp = 100;
    double ki = 0;
    double kd = 0;
    PidParams params{kp / gearRatio, ki / gearRatio, kd / gearRatio};
    mPidController = new PidController(&mError, &mOutput, sampleTime, params, 0, 999);

    reset();
}

Servo::~Servo()
{
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
    __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh1, mOutput);
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

void Servo::requestPosition(double postion)
{
    mSetpoint = postion;
}

double Servo::getRequestedPosition()
{
    return mSetpoint;
}

double Servo::getCurrentPosition()
{
    return mEncoderPulse * mEncoderResolution;
}
