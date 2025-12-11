#include "PidController.h"

PidController::PidController(float *inputPtr, float *outputPtr, float sampleTime, PidParams params, float lowLimit, float highLimit)
{
    mSampleTime = sampleTime;
    mPidParams = params;

    mInputPtr = inputPtr;
    mOutputPtr = outputPtr;

    mLowLimit = lowLimit;
    mHighLimit = highLimit;

    mPrevInput = 0;

    reset();
}

PidController::PidController(float *inputPtr, float *outputPtr, float sampleTime, float lowLimit, float highLimit)
{
    mSampleTime = sampleTime;

    mInputPtr = inputPtr;
    mOutputPtr = outputPtr;

    mLowLimit = lowLimit;
    mHighLimit = highLimit;

    mPrevInput = 0;

    reset();
}

PidController::~PidController()
{
    if (mInputPtr)
    {
        delete mInputPtr;
        mInputPtr = nullptr;
    }
    if (mOutputPtr)
    {
        delete mOutputPtr;
        mOutputPtr = nullptr;
    }
}

void PidController::reset()
{
    mIntegral = 0;
}

void PidController::tune(PidParams params)
{
    mPidParams = params;
    mBoostedLowInput = params.boostedInput * mLowLimit;
    mBoostedHighInput = params.boostedInput * mHighLimit;
    mBoostedLowOutput = params.boostedOutput * mLowLimit;
    mBoostedHighOutput = params.boostedOutput * mHighLimit;
}

void PidController::run()
{
    *mOutputPtr = mPidParams.kp * *mInputPtr;
    if (mPidParams.ki)
    {
        mIntegral += *mInputPtr * mSampleTime;
        if (mIntegral < mLowLimit)
        {
            mIntegral = mLowLimit;
        }
        if (mIntegral > mHighLimit)
        {
            mIntegral = mHighLimit;
        }
        *mOutputPtr += mPidParams.ki * mIntegral;
    }
    if (mPidParams.kd)
    {
        *mOutputPtr += mPidParams.kd * (*mInputPtr - mPrevInput) / mSampleTime;
        mPrevInput = *mInputPtr;
    }

    if (*mOutputPtr >= mBoostedLowInput && *mOutputPtr <= mBoostedHighInput)
    {
        *mOutputPtr = map(*mOutputPtr, mBoostedLowInput, mBoostedHighInput, mBoostedLowOutput, mBoostedHighOutput);
    }
    else if (*mOutputPtr > mBoostedHighInput)
    {
        *mOutputPtr = map(*mOutputPtr, mBoostedHighInput, mHighLimit, mBoostedHighOutput, mHighLimit);
    }
    else
    {
        *mOutputPtr = map(*mOutputPtr, mLowLimit, mBoostedLowInput, mLowLimit, mBoostedLowInput);
    }
}

float PidController::map(float input, float inMin, float inMax, float outMin, float outMax)
{
    float result = inMax == inMin ? input : outMin + (input - inMin) * (outMax - outMin) / (inMax - inMin);
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