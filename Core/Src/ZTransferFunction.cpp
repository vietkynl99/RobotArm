#include "ZTransferFunction.h"
#include "Log.h"
#include <stdint.h>

ZTransferFunction::ZTransferFunction(double *input, double *output)
{
    mInputPtr = input;
    mOutputPtr = output;

    // Init values for mNum and mDen
    // mNum.push_back(1);
    // mDen.push_back(1);

    // Init values for other parameters
    // initParams();
}

ZTransferFunction::~ZTransferFunction()
{
    if (mPrevInputList != nullptr)
    {
        delete[] mPrevInputList;
        mPrevInputList = nullptr;
    }
    if (mPrevOutputList != nullptr)
    {
        delete[] mPrevOutputList;
        mPrevOutputList = nullptr;
    }
    if (mLimitLow != nullptr)
    {
        delete mLimitLow;
        mLimitLow = nullptr;
    }
    if (mLimitHight != nullptr)
    {
        delete mLimitHight;
        mLimitHight = nullptr;
    }
}

void ZTransferFunction::initParams()
{
    if (mNum.size() < 1 || mDen.size() < 1 || mNum.at(0) == 0 || mDen.at(0) == 0)
    {
        println("Transfer function is invalid");
        return;
    }
    if (mNum.size() > mDen.size())
    {
        println("Transfer function is not stable! Cannot create object");
        return;
    }

    mLimitLow = nullptr;
    mLimitHight = nullptr;

    mPrevInputListSize = mNum.size();
    mPrevOutputListSize = mDen.size() - 1;
    mPrevInputList = new double[mPrevInputListSize];
    mPrevOutputList = new double[mPrevOutputListSize];

    for (int i = 0; i < mPrevInputListSize; i++)
    {
        mPrevInputList[i] = 0;
    }
    for (int i = 0; i < mPrevOutputListSize; i++)
    {
        mPrevOutputList[i] = 0;
    }
}

void ZTransferFunction::setOutputLimitLow(double limit)
{
    if (mLimitLow == nullptr)
    {
        mLimitLow = new double(limit);
    }
    else
    {
        *mLimitLow = limit;
    }
}

void ZTransferFunction::setOutputLimitHight(double limit)
{
    if (mLimitHight == nullptr)
    {
        mLimitHight = new double(limit);
    }
    else
    {
        *mLimitHight = limit;
    }
}

void ZTransferFunction::run()
{
    if (!mInputPtr || !mOutputPtr || !mPrevInputList || !mPrevOutputList)
    {
        return;
    }

    // Save prev input value
    for (int i = mPrevInputListSize - 1; i > 0; i--)
    {
        mPrevInputList[i] = mPrevInputList[i - 1];
    }
    mPrevInputList[0] = *mInputPtr;

    // Save prev output value
    for (int i = mPrevOutputListSize - 1; i > 0; i--)
    {
        mPrevOutputList[i] = mPrevOutputList[i - 1];
    }
    mPrevOutputList[0] = *mOutputPtr;

    // Caculate current output value
    *mOutputPtr = 0;
    for (int i = 0; i < mPrevInputListSize; i++)
    {
        *mOutputPtr += mNum.at(i) * mPrevInputList[i];
    }
    for (int i = 0; i < mPrevOutputListSize; i++)
    {
        *mOutputPtr -= mDen.at(i + 1) * mPrevOutputList[i];
    }
    *mOutputPtr /= mDen.at(0);

    // Output limit reached
    if (mLimitHight != nullptr && *mOutputPtr > *mLimitHight)
    {
        *mOutputPtr = *mLimitHight;
    }
    if (mLimitLow != nullptr && *mOutputPtr < *mLimitLow)
    {
        *mOutputPtr = *mLimitLow;
    }
}