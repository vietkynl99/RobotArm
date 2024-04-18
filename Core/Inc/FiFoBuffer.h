#ifndef INC_FIFOBUFFER_H_
#define INC_FIFOBUFFER_H_

#define FIFO_SIZE_MAX_DEFAULT (64)

template <typename T>
class FiFoBuffer
{
private:
    int mSize;
    int mSizeMax;
    T *mDataArr;
    int mRIndex;
    int mWIndex;

public:
    FiFoBuffer(uint16_t sizeMax = FIFO_SIZE_MAX_DEFAULT)
    {
        mSizeMax = sizeMax > 0 && sizeMax <= FIFO_SIZE_MAX_DEFAULT ? sizeMax : FIFO_SIZE_MAX_DEFAULT;
        mDataArr = new T[mSizeMax];
        mSize = 0;
        mRIndex = 0;
        mWIndex = 0;
    }

    ~FiFoBuffer()
    {
        mSize = 0;
        if (mDataArr)
        {
            delete[] mDataArr;
            mDataArr = nullptr;
        }
    }

    bool push(T value)
    {
        if (mSize >= mSizeMax)
        {
            return false;
        }
        mSize++;
        mWIndex = (mWIndex + 1) % mSizeMax;
        mDataArr[mWIndex] = value;
        return true;
    }

    bool pop(T &value)
    {
        if (mSize <= 0)
        {
            return false;
        }
        mSize--;
        mRIndex = (mRIndex + 1) % mSizeMax;
        value = mDataArr[mRIndex];
        return true;
    }

    int size() const
    {
        return mSize;
    }

    bool get(int index, T &value)
    {
        if (index < 0 || index >= mSize)
        {
            return false;
        }
        value = mDataArr[(mRIndex + index + 1) % mSizeMax];
        return true;
    }
};
#endif /* INC_FIFOBUFFER_H_ */
