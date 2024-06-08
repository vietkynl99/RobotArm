#include "Servo.h"
#include <stdio.h>
#include <cstdlib>

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double gearRatio, double minPosition, double maxPosition, double zeroPosition, double kp, double ki, double kd)
{
    mState = SERVO_STATE_DISABLED;
    mSpeed = 0;
#if SERVO_ENABLE_ERR_DETECTION
    mTick = 0;
#endif
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
    if (mState == SERVO_STATE_ZERO_DETECTION)
    {
        println("Stop zero detection");
        setState(SERVO_STATE_POSITION);
        reset(mZeroPosition);
    }
}

void Servo::setState(ServoState state)
{
    mState = state;
    if (mState != SERVO_STATE_ZERO_DETECTION && mState != SERVO_STATE_POSITION)
    {
        setOutput(0);
    }
}

int Servo::getState()
{
    return mState;
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
    if (mState == SERVO_STATE_ERROR || mState == SERVO_STATE_DISABLED)
    {
        return;
    }

    if (mState == SERVO_STATE_ZERO_DETECTION)
    {
        mSetpoint = mSpeed * (HAL_GetTick() - mOriginTime);
    }
    mError = mSetpoint - getCurrentPosition();

#if SERVO_ENABLE_ERR_DETECTION
    mTick = (mTick + 1) % 100;
    if (!mTick)
    {
        if (abs(mError) > abs(mPrevError) && abs(mError) > 10)
        {
            if (mInvalidCount < 0xFF)
            {
                mInvalidCount++;
            }
            if (mInvalidCount > 5)
            {
                println("Servo direction error");
                setState(SERVO_STATE_ERROR);
                mPrevError = mError;
                return;
            }
        }
        else
        {
            mInvalidCount = 0;
        }

        mPrevError = mError;
    }
#endif
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

void Servo::reset(double position)
{
    mEncoderPulse = position / mEncoderResolution;
    mSetpoint = position;
    mOutput = 0;
    mError = 0;

#if SERVO_ENABLE_ERR_DETECTION
    mPrevError = 0;
    mInvalidCount = 0;
#endif

    mPidController->reset();
}

bool Servo::zeroDetect()
{
    if (mState == SERVO_STATE_ZERO_DETECTION)
    {
        return false;
    }
    setState(SERVO_STATE_ZERO_DETECTION);
    requestSpeed(SERVO_ZERO_DETECTION_SPEED);
    return true;
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

// Unit: degree
bool Servo::requestPosition(double postion)
{
    if (mState != SERVO_STATE_POSITION)
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

// Unit: rpm
bool Servo::requestSpeed(double rpm)
{
    if (mState != SERVO_STATE_ZERO_DETECTION)
    {
        println("Is not zero detection state");
        return false;
    }
    mEncoderPulse = 0;
    // rpm to deg/ms
    mSpeed = rpm * 0.006;
    mOriginTime = HAL_GetTick();
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