#ifndef INC_DELAY_TRANSFER_FUNCTION_H_
#define INC_DELAY_TRANSFER_FUNCTION_H_

#include "ZTransferFunction.h"

class DelayTransferFunction : public ZTransferFunction
{
private:
    double mTime;

public:
    DelayTransferFunction(double *input, double *output, double sampleTime, double delayTime);
};
#endif /* INC_DELAY_TRANSFER_FUNCTION_ */