#include "DelayTransferFunction.h"

DelayTransferFunction::DelayTransferFunction(double *input, double *output, double sampleTime, double delayTime) : ZTransferFunction(input, output)
{
    // Init values for mNum and mDen
    ZTransferFunction::mNum.push_back(sampleTime);
    
    ZTransferFunction::mDen.push_back(delayTime);
    ZTransferFunction::mDen.push_back(sampleTime - delayTime);

    // Init values for other parameters
    initParams();
}