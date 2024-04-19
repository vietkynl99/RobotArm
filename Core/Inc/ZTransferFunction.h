#ifndef INC_ZTRANSFERFUNCTION_H_
#define INC_ZTRANSFERFUNCTION_H_

#include "Vector.h"

class ZTransferFunction
{
protected:
    Vector<double> mNum;
    Vector<double> mDen;

private:
    double *mInputPtr;
    double *mOutputPtr;
    double *mPrevInputList;
    double *mPrevOutputList;
    int mPrevInputListSize;
    int mPrevOutputListSize;
    double *mLimitLow;
    double *mLimitHight;

public:
    ZTransferFunction(double *input, double *output);
    ~ZTransferFunction();

    void setOutputLimitLow(double limit);
    void setOutputLimitHight(double limit);
    void run();

protected:
    void initParams();
};
#endif /* INC_ZTRANSFERFUNCTION_H_ */
