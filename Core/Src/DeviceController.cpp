#include "DeviceController.h"
#include "Log.h"
#include <cstring>

#define CONNECTION_TIMEOUT 1000

#define DEBUG_FRAME_DATA (1)

DeviceController::DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
#if DEBUG_FRAME_DATA
    if (sizeof(DataFrame) != SPI_FRAME_SIZE)
    {
        println("Error !!! DataFrame is not a valid size (%d)", sizeof(DataFrame));
    }
#endif

    mServo[0] = new Servo(htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[1] = new Servo(htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[2] = new Servo(htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[3] = new Servo(htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[4] = new Servo(htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, 98.775, -160, 170, 180, 20, 0, 0);
    mServo[5] = new Servo(htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, 98.775, -160, 170, 180, 20, 0, 0);

    memset(&mTxDataFrame, 0, sizeof(DataFrame));
    memset(&mRxDataFrame, 0, sizeof(DataFrame));
    setState(STATE_DISCONNECTED);
    mLastTime = 0;
    mAutoGetData = false;

    mHspi = hspi;
    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.rawData, mRxDataFrame.rawData, SPI_FRAME_SIZE);
}

string DeviceController::getString(const DataFrame &frame)
{
    string str = "";
    for (uint8_t i = 0; i < sizeof(DataFrame); i++)
    {
        if (i != 0)
        {
            str += ".";
        }
        str += to_string(frame.rawData[i]);
    }
    return str;
}

uint8_t DeviceController::calculateChecksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i)
    {
        checksum ^= data[i];
    }
    return checksum;
}

bool DeviceController::verifyChecksum(const uint8_t *data, size_t length, uint8_t checksum)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i)
    {
        sum ^= data[i];
    }
    sum ^= checksum;
    return sum == 0;
}

void DeviceController::setState(DeviceState state)
{
    if (mState != state)
    {
        mState = state;
        if (mState != STATE_CONNECTED)
        {
            createDataFrame(mTxDataFrame, CMD_RESP_DATA_ERROR, nullptr, 0);
        }
        println("Status changed to %d", mState);
    }
}

void DeviceController::createDataFrame(DataFrame &dataFrame, uint8_t command)
{
    dataFrame.start = SPI_DATA_START_BYTE;
    dataFrame.command = command;
    dataFrame.checksum = calculateChecksum(dataFrame.rawData, SPI_FRAME_SIZE - 1);
}

void DeviceController::createDataFrame(DataFrame &dataFrame, uint8_t command, const uint8_t *data, size_t length)
{
    memset(&dataFrame, 0, sizeof(DataFrame));
    if (data && length > 0 && length <= SPI_DATA_SIZE)
    {
        memcpy(dataFrame.data, data, length);
    }
    createDataFrame(dataFrame, command);
}

DeviceState DeviceController::verifyDataFrame(const DataFrame &frame)
{
    if (frame.start == 0)
    {
        for (int i = 0; i < SPI_FRAME_SIZE; i++)
        {
            if (frame.rawData[i] != 0)
            {
#if DEBUG_FRAME_DATA
                println("Data frame error");
#endif
                return STATE_DATA_ERROR;
            }
        }
#if DEBUG_FRAME_DATA
        println("No data error");
#endif
        return STATE_DISCONNECTED;
    }
    else if (frame.start != SPI_DATA_START_BYTE)
    {
#if DEBUG_FRAME_DATA
        println("Start byte error");
#endif
        return STATE_DATA_ERROR;
    }
    if (!verifyChecksum(frame.rawData, SPI_FRAME_SIZE - 1, frame.checksum))
    {
#if DEBUG_FRAME_DATA
        println("Checksum error");
#endif
        return STATE_DATA_ERROR;
    }
    return STATE_CONNECTED;
}

void DeviceController::onEncoderEvent(uint16_t pin)
{
    if (pin == M1_E1_Pin)
    {
        mServo[0]->onEncoderEvent(!HAL_GPIO_ReadPin(M1_E2_GPIO_Port, M1_E2_Pin));
    }
    else if (pin == M2_E1_Pin)
    {
        mServo[1]->onEncoderEvent(!HAL_GPIO_ReadPin(M2_E2_GPIO_Port, M2_E2_Pin));
    }
    else if (pin == M3_E1_Pin)
    {
        mServo[2]->onEncoderEvent(!HAL_GPIO_ReadPin(M3_E2_GPIO_Port, M3_E2_Pin));
    }
    else if (pin == M4_E1_Pin)
    {
        mServo[3]->onEncoderEvent(!HAL_GPIO_ReadPin(M4_E2_GPIO_Port, M4_E2_Pin));
    }
    else if (pin == M5_E1_Pin)
    {
        mServo[4]->onEncoderEvent(!HAL_GPIO_ReadPin(M5_E2_GPIO_Port, M5_E2_Pin));
    }
    else if (pin == M6_E1_Pin)
    {
        mServo[5]->onEncoderEvent(!HAL_GPIO_ReadPin(M6_E2_GPIO_Port, M6_E2_Pin));
    }
}

void DeviceController::onZeroDetected(int index)
{
    // println("onZeroDetected: %d", index);
    if (index >= 0 && index < SERVO_NUMS)
    {
        mServo[index]->onZeroDectected();
    }
}

void DeviceController::onControllerInterrupt()
{
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        mServo[i]->run();
    }
}

void DeviceController::onDataReceived()
{
    mLastTime = HAL_GetTick() + CONNECTION_TIMEOUT;
    DeviceState state = verifyDataFrame(mRxDataFrame);
#if DEBUG_FRAME_DATA
    println("received: %s -> state: %d", getString(mRxDataFrame).c_str(), state);
#endif
    setState(state);

    if (state == STATE_CONNECTED)
    {
        switch (mRxDataFrame.command)
        {
        case CMD_REQ_SET_AUTO_GET_SERVO_DATA:
            mAutoGetData = mRxDataFrame.data[0] != 0;
            break;
        default:
            if (mAutoGetData)
            {
                // for (int i = 0; i < SERVO_NUMS; i++)
                // {
                //     mTxDataFrame.servoPosition[i] = mServo[i]->getCurrentPosition();
                // }
                // createDataFrame(mTxDataFrame, CMD_RESP_SERVO_DATA);
            }
            else if (mTxDataFrame.command != CMD_RESP_PING)
            {
                createDataFrame(mTxDataFrame, CMD_RESP_PING, nullptr, 0);
            }
            break;
        }
    }

    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.rawData, mRxDataFrame.rawData, SPI_FRAME_SIZE);
}

void DeviceController::onDataError()
{
    // println("onDataError");
    setState(STATE_DISCONNECTED);

    HAL_SPI_DeInit(mHspi);
    asm("nop");
    asm("nop");
    __SPI1_FORCE_RESET();
    asm("nop");
    asm("nop");
    __SPI1_RELEASE_RESET();
    asm("nop");
    asm("nop");
    HAL_SPI_Init(mHspi);
    asm("nop");
    asm("nop");
    HAL_SPI_DMAStop(mHspi);
    asm("nop");
    asm("nop");

    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.rawData, mRxDataFrame.rawData, SPI_FRAME_SIZE);
}

void DeviceController::run()
{
    if (mState != STATE_DISCONNECTED && HAL_GetTick() > mLastTime)
    {
        // println("Timeout...");
        mLastTime = HAL_GetTick() + CONNECTION_TIMEOUT;
        setState(STATE_DISCONNECTED);
    }
}