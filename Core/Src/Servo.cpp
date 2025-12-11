#include "Servo.h"
#include <stdio.h>
#include <cstdlib>

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2,
             GPIO_TypeDef *e1GPIO, uint16_t e1Pin, GPIO_TypeDef *e2GPIO, uint16_t e2Pin,
             GearBox gearBox, PositionLimit positionLimit, PidParams params)
{
    mE1GPIO = e1GPIO;
    mE1Pin = e1Pin;
    mE2GPIO = e2GPIO;
    mE2Pin = e2Pin;
#if SERVO_ENABLE_ERR_DETECTION
    mTick = 0;
#endif
    mOutputTimer = outputTimer;
    mOutputTimerCh1 = outputTimerCh1;
    mOutputTimerCh2 = outputTimerCh2;

    setGearBox(gearBox);
    setPositionLimit(positionLimit);

    mPidController = new PidController(&mError, &mOutput, SERVO_SAMPLE_TIME_S, -SERVO_PWM_RESOLUTION, SERVO_PWM_RESOLUTION);
    tune(params);

    setState(SERVO_STATE_DISABLED);
    setMode(SERVO_MODE_POSITION);

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

void Servo::onEncoderEvent()
{
    if (HAL_GPIO_ReadPin(mE2GPIO, mE2Pin))
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
    if (mState == SERVO_STATE_RUNNING && mMode == SERVO_MODE_SPEED)
    {
        println("Stop zero detection");
        setState(SERVO_STATE_RUNNING);
        setMode(SERVO_MODE_POSITION);
        reset(mPositionLimit.zero);
    }
}

void Servo::setState(ServoState state)
{
    if (mState != state)
    {
        mState = state;
        if (mState != SERVO_STATE_RUNNING)
        {
            setOutput(0);
            mOutput = 0;
            mError = 0;
        }
    }
}

void Servo::setMode(ServoMode mode)
{
    if (mMode != mode)
    {
        mMode = mode;
        setOutput(0);
    }
}

ServoState Servo::getState()
{
    return mState;
}

ServoMode Servo::getMode()
{
    return mMode;
}

GearBox Servo::getGearBox()
{
    return mGearBox;
}

PositionLimit Servo::getPositionLimit()
{
    return mPositionLimit;
}

PidParams Servo::getPidParams()
{
    return mPidParams;
}

void Servo::setGearBox(GearBox gearBox)
{
    mGearBox = gearBox;
    mResolution = 360 / gearBox.ratio;
    mEncoderResolution = mResolution / gearBox.encoderResolution;
}

void Servo::setPositionLimit(PositionLimit positionLimit)
{
    mPositionLimit = positionLimit;
}

void Servo::tune(PidParams params)
{
    mPidParams = params;
    params.kp /= mResolution;
    params.ki /= mResolution;
    params.kd /= mResolution;
    mPidController->tune(params);
}

void Servo::run()
{
    if (mState != SERVO_STATE_RUNNING)
    {
        return;
    }

    if (mMode == SERVO_MODE_SPEED)
    {
        mSetpoint = mSpeed * (HAL_GetTick() - mOriginTime);
    }
    mError = mSetpoint - getCurrentPosition();

#if SERVO_ENABLE_ERR_DETECTION
    mTick = (mTick + 1) % 100;
    if (!mTick)
    {
        if (abs(mError) > 10 && (abs(mError) > abs(mPrevError) || abs(mError - abs(mPrevError)) < 0.5))
        {
            mInvalidCount++;
            if (mInvalidCount > 5)
            {
                println("Servo error");
                setState(SERVO_STATE_ERROR);
                mPrevError = mError;
                printData();
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
    setOutput(mOutput);
}

void Servo::reset(float position)
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

    setState(SERVO_STATE_DISABLED);
}

bool Servo::zeroDetect()
{
    setState(SERVO_STATE_RUNNING);
    setMode(SERVO_MODE_SPEED);
    requestSpeed(SERVO_ZERO_DETECTION_SPEED);
    return true;
}

const char *Servo::toString(ServoState value)
{
    switch (value)
    {
    case SERVO_STATE_ERROR:
        return "ERROR";
    case SERVO_STATE_DISABLED:
        return "DISABLED";
    case SERVO_STATE_RUNNING:
        return "RUNNING";
    default:
        return "UNKNOWN";
    }
}

const char *Servo::toString(ServoMode value)
{
    switch (value)
    {
    case SERVO_MODE_SPEED:
        return "SPEED";
    case SERVO_MODE_POSITION:
        return "POSITION";
    default:
        return "UNKNOWN";
    }
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
bool Servo::requestPosition(float postion)
{
    if (mState != SERVO_STATE_RUNNING)
    {
        println("Servo is not running");
        return false;
    }
    if (mMode != SERVO_MODE_POSITION)
    {
        println("Is not position mode");
        return false;
    }
    if (postion > mPositionLimit.max || postion < mPositionLimit.min)
    {
        println("%.2f out of range [%.2f; %.2f]", postion, mPositionLimit.min, mPositionLimit.max);
        return false;
    }
    mSetpoint = postion;
    return true;
}

// Unit: rpm
bool Servo::requestSpeed(float rpm)
{
    if (mState != SERVO_STATE_RUNNING)
    {
        println("Servo is not running");
        return false;
    }
    if (mMode != SERVO_MODE_SPEED)
    {
        println("Is not speed mode");
        return false;
    }
    mEncoderPulse = 0;
    // rpm to deg/ms
    mSpeed = rpm * 0.006;
    mOriginTime = HAL_GetTick();
    return true;
}

uint16_t Servo::getE1Pin()
{
    return mE1Pin;
}

uint16_t Servo::getE2Pin()
{
    return mE2Pin;
}

int Servo::getEncoderPluse()
{
    return mEncoderPulse;
}

// Unit: degree
float Servo::getRequestedPosition()
{
    return mSetpoint;
}

// Unit: degree
float Servo::getCurrentPosition()
{
    return mEncoderPulse * mEncoderResolution;
}

// Range: [-100; 100]
float Servo::getControlValue()
{
    return 100 * mOutput / SERVO_PWM_RESOLUTION;
}

void Servo::printData()
{
    println("state %s, mode %s, GearBox{%.2f, %d}, Limit{%.2f, %.2f, %.2f}, PID{%.2f, %.2f, %.2f, %.2f %.2f}, S%.2f, F%.2f, V%.2f",
            toString(getState()),
            toString(getMode()),
            mGearBox.ratio,
            mGearBox.encoderResolution,
            mPositionLimit.min,
            mPositionLimit.max,
            mPositionLimit.zero,
            mPidParams.kp,
            mPidParams.ki,
            mPidParams.kd,
            mPidParams.boostedInput,
            mPidParams.boostedOutput,
            getRequestedPosition(),
            getCurrentPosition(),
            getControlValue());
}