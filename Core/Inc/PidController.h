#pragma once

#include "SpiTransferType.h"

class PidController
{
private:
    double mSampleTime;
    PidParams mPidParams;
    double mLowLimit;
    double mHighLimit;
    double mBoostedLowInput;
    double mBoostedLowOutput;
    double mBoostedHighInput;
    double mBoostedHighOutput;

    double *mInputPtr;
    double *mOutputPtr;

    double mPrevInput;
    double mIntegral;

public:
    PidController(double *inputPtr, double *outputPtr, double sampleTime, PidParams params, double lowLimit, double highLimit);
    PidController(double *inputPtr, double *outputPtr, double sampleTime, double lowLimit, double highLimit);
    ~PidController();

    void reset();
    void tune(PidParams params);
    void run();

private:
    double map(double input, double inMin, double inMax, double outMin, double outMax);
};