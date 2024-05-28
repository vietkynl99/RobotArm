#include "Servo.h"
#include <stdio.h>

#define ZERO_DETECTION_SPEED_PERCENT (0.25)

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double gearRatio, double minPosition, double maxPosition, double zeroPosition, double kp, double ki, double kd)
{
    mEnabled = true;
    mIsZeroDetecting = false;
    mZeroChecked = false;
    mMinPosition = minPosition;
    mMaxPosition = maxPosition;
    mZeroPosition = zeroPosition;

    mResolution = 360 / gearRatio;

    mOutputTimer = outputTimer;
    mOutputTimerCh1 = outputTimerCh1;
    mOutputTimerCh2 = outputTimerCh2;
    mEncoderResolution = mResolution / SERVO_ENCODER_RESOLUTION;

    PidParams params{kp / mResolution, ki / mResolution, kd / mResolution};
    mPidController = new PidController(&mError, &mOutput, SERVO_SAMPLE_TIME_S, params, -SERVO_PWM_RESOLUTION, SERVO_PWM_RESOLUTION);

    reset();

    HAL_TIM_PWM_Start(mOutputTimer, mOutputTimerCh1);
	HAL_TIM_PWM_Start(mOutputTimer, mOutputTimerCh2);
    setOutput(0);
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

void Servo::onZeroDectected()
{
    if (mIsZeroDetecting)
    {
        println("Stop zero detection");
        mIsZeroDetecting = false;
        mZeroChecked = true;
        reset(mZeroPosition);
    }
}

void Servo::setEnable(bool enabled)
{
    mEnabled = enabled;
}

void Servo::tune(PidParams params)
{
    params.kp /= mResolution;
    params.ki /= mResolution;
    params.kd /= mResolution;
    mPidController->tune(params);
}

void Servo::run()
{
    if (mEnabled && !mIsZeroDetecting)
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

        setOutput(fixedOutput);
    }
}

void Servo::reset(double position)
{
    mEncoderPulse = position / mEncoderResolution;
    mSetpoint = position;
    mOutput = 0;
    mError = 0;

    mPidController->reset();
}

void Servo::zeroDetect()
{
    if (mEnabled && !mIsZeroDetecting)
    {
        println("Zero dectect is running...");
        mIsZeroDetecting = true;
        mZeroChecked = false;
        setOutput(SERVO_PWM_RESOLUTION * ZERO_DETECTION_SPEED_PERCENT);
    }
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

void Servo::setOutput(int value)
{
    if (mEnabled)
    {
        if (value > 0)
        {
            __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh1, value);
            __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh2, 0);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh1, 0);
            __HAL_TIM_SET_COMPARE(mOutputTimer, mOutputTimerCh2, -value);
        }
    }
}

// Unit: degree
bool Servo::requestPosition(double postion)
{
    if (!mZeroChecked)
    {
        println("Need to check zero position");
        return false;
    }
    if (postion > mMaxPosition || postion < mMinPosition)
    {
        println("%.2f out of range [%.2f; %.2f]", postion, mMinPosition, mMaxPosition);
        return false;
    }
    mSetpoint = postion;
    return true;
}

double Servo::getMinPostion()
{
    return mMinPosition;
}

double Servo::getMaxPostion()
{
    return mMaxPosition;
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