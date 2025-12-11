#pragma once

#include "SpiTransferType.h"

class PidController
{
private:
    float mSampleTime;
    PidParams mPidParams;
    float mLowLimit;
    float mHighLimit;

    float *mInputPtr;
    float *mOutputPtr;

    float mPrevInput;
    float mIntegral;

public:
    PidController(float *inputPtr, float *outputPtr, float sampleTime, PidParams params, float lowLimit, float highLimit);
    PidController(float *inputPtr, float *outputPtr, float sampleTime, float lowLimit, float highLimit);
    ~PidController();

    void reset();
    void tune(PidParams params);
    void run();

private:
    float map(float input, float inMin, float inMax, float outMin, float outMax);
};