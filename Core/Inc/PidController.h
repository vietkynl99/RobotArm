#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

#include "SpiTransferType.h"

class PidController
{
private:
    double mSampleTime;
    PidParams mPidParams;
    double mLowLimit;
    double mHighLimit;

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
};



#endif /* INC_PIDCONTROLLER_H_ */
