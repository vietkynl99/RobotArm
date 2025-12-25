#include "Servo.h"
#include <stdio.h>
#include <cstdlib>

Servo::Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2,
             GearBox gearBox, PositionLimit positionLimit, PidParams params)
{
#if SERVO_ENABLE_ERR_DETECTION
    mTick = 0;
#endif
    mSpeedTickTime = 0;
    mOutputTimer = outputTimer;
    mOutputTimerCh1 = outputTimerCh1;
    mOutputTimerCh2 = outputTimerCh2;

    setGearBox(gearBox);
    setPositionLimit(positionLimit);

    mPidController = new PidController(&mError, &mOutput, SERVO_SAMPLE_TIME_S, -SERVO_PWM_RESOLUTION, SERVO_PWM_RESOLUTION);
    tune(params);

    mZeroDetectionState = ZERO_DETECTION_STATE_IDLE;
    setState(SERVO_STATE_DISABLED);
    setZeroDetectionState(ZERO_DETECTION_STATE_IDLE);

    HAL_TIM_PWM_Start(mOutputTimer, mOutputTimerCh1);
    HAL_TIM_PWM_Start(mOutputTimer, mOutputTimerCh2);
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

void Servo::onEvent(ServoEvent event)
{
    switch (event)
    {
    case SERVO_EVENT_ENCODER_INC:
        mEncoderPulse++;
        break;
    case SERVO_EVENT_ENCODER_DEC:
        mEncoderPulse--;
        break;
    case SERVO_EVENT_ZERO_DETECTED:
        onZeroDectected();
        break;
    default:
        break;
    }
}

void Servo::onZeroDectected()
{
    if (mState != SERVO_STATE_ZERO_DETECTING)
    {
        return;
    }
    println("zero dectected");
    if (mZeroDetectionState == ZERO_DETECTION_STATE_FORWARD1)
    {
        setZeroDetectionState((ZeroDetectionState)(mZeroDetectionState + 1));
    }
    else if (mZeroDetectionState == ZERO_DETECTION_STATE_FORWARD2)
    {
        println("Stop zero detection");
        reset(mPositionLimit.zero);
        setState(SERVO_STATE_RUNNING);
        setZeroDetectionState(ZERO_DETECTION_STATE_IDLE);
    }
    // else
    // {
    //     println("unexpected zero detected");
    //     setState(SERVO_STATE_ERROR);
    // }
}

void Servo::setState(ServoState state)
{
    if (mState != state)
    {
        mState = state;
        if (state == SERVO_STATE_ERROR)
        {
            println("Servo enter error state");
            printData();
        }
        if (mState != SERVO_STATE_RUNNING && mState != SERVO_STATE_ZERO_DETECTING)
        {
            setOutput(0);
            mOutput = 0;
            mError = 0;
        }
    }
}

void Servo::setZeroDetectionState(ZeroDetectionState state)
{
    if (mZeroDetectionState != state)
    {
        mZeroDetectionState = state;
        switch (state)
        {
        case ZERO_DETECTION_STATE_IDLE:
        {
            break;
        }
        case ZERO_DETECTION_STATE_BACKWARD1:
        {
            requestSpeed(-SERVO_ZERO_DETECTION_SPEED, 2000);
            break;
        }
        case ZERO_DETECTION_STATE_FORWARD1:
        {
            requestSpeed(SERVO_ZERO_DETECTION_SPEED);
            break;
        }
        case ZERO_DETECTION_STATE_BACKWARD2:
        {
            requestSpeed(-SERVO_ZERO_DETECTION_SPEED, 1000);
            break;
        }
        case ZERO_DETECTION_STATE_FORWARD2:
        {
            requestSpeed(1);
            break;
        }
        default:
            break;
        }
    }
}

ServoState Servo::getState()
{
    return mState;
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

void Servo::timerInterrupt()
{
    if (mState != SERVO_STATE_ZERO_DETECTING && mState != SERVO_STATE_RUNNING)
    {
        return;
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

void Servo::run()
{
    if (mState == SERVO_STATE_ZERO_DETECTING)
    {
        if (mTimeoutTime > 0 && HAL_GetTick() - mOriginTimeTick > (uint32_t)mTimeoutTime)
        {
            // timeout
            if (mZeroDetectionState == ZERO_DETECTION_STATE_BACKWARD1 || mZeroDetectionState == ZERO_DETECTION_STATE_BACKWARD2)
            {
                setZeroDetectionState((ZeroDetectionState)(mZeroDetectionState + 1));
            }
            else
            {
                println("Speed request timeout???");
                setState(SERVO_STATE_ERROR);
                return;
            }
        }
        else
        {
            mSetpoint = mSpeed * (HAL_GetTick() - mOriginTimeTick);
        }
    }

    // Caculate speed
    if (HAL_GetTick() - mSpeedTickTime > SERVO_SPEED_DETECTION_INTERVAL)
    {
        mSpeedTickTime = HAL_GetTick();
        mEncoderPulseCount = mEncoderPulse - mPrevEncoderPulse;
        mPrevEncoderPulse = mEncoderPulse;
    }
}

void Servo::reset(float position)
{
    mEncoderPulse = position / mEncoderResolution;
    mPrevEncoderPulse = mEncoderPulse;
    mSetpoint = position;
    mOutput = 0;
    mError = 0;

#if SERVO_ENABLE_ERR_DETECTION
    mPrevError = 0;
    mInvalidCount = 0;
#endif

    mPidController->reset();

    setOutput(0);
    setState(SERVO_STATE_DISABLED);
}

bool Servo::zeroDetect()
{
    if (mState != SERVO_STATE_ZERO_DETECTING && mZeroDetectionState == ZERO_DETECTION_STATE_IDLE)
    {
        println("Start zero detection");
        setState(SERVO_STATE_ZERO_DETECTING);
        setZeroDetectionState(ZERO_DETECTION_STATE_BACKWARD1);
        return true;
    }
    else
    {
        println("Already in zero detection");
        return false;
    }
}

const char *Servo::toString(ServoState value)
{
    switch (value)
    {
    case SERVO_STATE_ERROR:
        return "ERROR";
    case SERVO_STATE_DISABLED:
        return "DISABLED";
    case SERVO_STATE_ZERO_DETECTING:
        return "ZERO_DETECTING";
    case SERVO_STATE_RUNNING:
        return "RUNNING";
    default:
        return "UNKNOWN";
    }
}

void Servo::setOutput(int value)
{
    mOutput = value;
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
    if (postion > mPositionLimit.max || postion < mPositionLimit.min)
    {
        println("%.2f out of range [%.2f; %.2f]", postion, mPositionLimit.min, mPositionLimit.max);
        return false;
    }
    mSetpoint = postion;
    return true;
}

// Unit: rpm
bool Servo::requestSpeed(float rpm, int timeout)
{
    if (mState != SERVO_STATE_ZERO_DETECTING)
    {
        println("Servo is not in zero detecting state");
        return false;
    }
    mEncoderPulse = 0;
    mPrevEncoderPulse = 0;
    mSpeed = rpm * 0.006; // rpm to deg/ms
    mOriginTimeTick = HAL_GetTick();
    mTimeoutTime = timeout;
    return true;
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

// Unit: deg/s
float Servo::getCurrentSpeed()
{
    return mEncoderPulseCount * mEncoderResolution * 1000 / SERVO_SPEED_DETECTION_INTERVAL;
}

// Unit: rpm
float Servo::getCurrentSpeedRpm()
{
    return mEncoderPulseCount * mEncoderResolution * 166.67f / SERVO_SPEED_DETECTION_INTERVAL;
}

// Range: [-100; 100]
float Servo::getControlValue()
{
    return 100 * mOutput / SERVO_PWM_RESOLUTION;
}

void Servo::printData()
{
    println("state %s, zeroDetectState %d, GearBox{%.2f, %d}, Limit{%.2f, %.2f, %.2f}, PID{%.2f, %.2f, %.2f}, S%.2f, F%.2f, V%.2f, %.2frpm",
            toString(getState()),
            mZeroDetectionState,
            mGearBox.ratio,
            mGearBox.encoderResolution,
            mPositionLimit.min,
            mPositionLimit.max,
            mPositionLimit.zero,
            mPidParams.kp,
            mPidParams.ki,
            mPidParams.kd,
            getRequestedPosition(),
            getCurrentPosition(),
            getControlValue(),
            getCurrentSpeedRpm());
}