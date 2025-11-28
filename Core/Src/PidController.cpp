#include "PidController.h"

PidController::PidController(double *inputPtr, double *outputPtr, double sampleTime, PidParams params, double lowLimit, double highLimit)
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

PidController::PidController(double *inputPtr, double *outputPtr, double sampleTime, double lowLimit, double highLimit)
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

    if (*mOutputPtr < mLowLimit)
    {
        *mOutputPtr = mLowLimit;
    }
    if (*mOutputPtr > mHighLimit)
    {
        *mOutputPtr = mHighLimit;
    }
}