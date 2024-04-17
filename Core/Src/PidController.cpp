#include "PidController.h"

#define PID_KD_ENABLE 0

PidController::PidController(double *inputPtr, double *outputPtr, double sampleTime, PidParams params, double lowLimit, double highLimit)
{
    mSampleTime = sampleTime;
    mPidParams = params;

    mInputPtr = inputPtr;
    mOutputPtr = outputPtr;

    mLowLimit = lowLimit;
    mHighLimit = highLimit;

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
    mPrevInput = 0;
    mIntegral = 0;
}

void PidController::tune(PidParams params)
{
    mPidParams = params;
}

void PidController::run()
{
    mIntegral += *mInputPtr * mSampleTime;
#if PID_KD_ENABLE
    *mOutputPtr = mPidParams.kp * *mInputPtr + mPidParams.ki * mIntegral + mPidParams.kd * (*mInputPtr - mPrevInput) / mSampleTime;
#else
    *mOutputPtr = mPidParams.kp * *mInputPtr + mPidParams.ki * mIntegral;
#endif

    if (*mOutputPtr < mLowLimit)
    {
        *mOutputPtr = mLowLimit;
    }
    if (*mOutputPtr > mHighLimit)
    {
        *mOutputPtr = mHighLimit;
    }

#if PID_KD_ENABLE
    mPrevInput = *mInputPtr;
#endif
}