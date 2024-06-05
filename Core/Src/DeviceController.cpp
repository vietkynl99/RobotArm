#include "DeviceController.h"
#include "Log.h"
#include <cstring>

#define CONNECTION_TIMEOUT 1000

#define DEBUG_FRAME_DATA (0)

DeviceController::DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
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
    mSettingsData.autoSend = false;

    mHspi = hspi;
    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.frame, mRxDataFrame.frame, SPI_FRAME_SIZE);
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
        str += to_string(frame.frame[i]);
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
            create(mTxDataFrame, CMD_DATA_ERROR, nullptr, 0, RESP_CODE_ERROR);
        }
        println("Status changed to %d", mState);
    }
}

void DeviceController::create(DataFrame &dataFrame, uint8_t command, uint8_t responseCode)
{
    dataFrame.pack.key1 = SPI_DATA_KEY1;
    dataFrame.pack.key2 = SPI_DATA_KEY2;
    dataFrame.pack.command = command;
    dataFrame.pack.responseCode = responseCode;
    dataFrame.pack.checksum = calculateChecksum(dataFrame.frame, SPI_FRAME_SIZE - 1);
}

void DeviceController::create(DataFrame &dataFrame, uint8_t command, const void *data, size_t length, uint8_t responseCode)
{
    memset(&dataFrame, 0, sizeof(DataFrame));
    if (data && length > 0 && length <= SPI_DATA_SIZE)
    {
        memcpy(dataFrame.pack.data, data, length);
    }
    create(dataFrame, command, responseCode);
}

DeviceState DeviceController::verifyDataFrame(const DataFrame &frame)
{
    if (!frame.pack.key1 || !frame.pack.key2)
    {
        for (int i = 0; i < SPI_FRAME_SIZE; i++)
        {
            if (frame.frame[i] != 0)
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
    else if (frame.pack.key1 != SPI_DATA_KEY1 || frame.pack.key2 != SPI_DATA_KEY2)
    {
#if DEBUG_FRAME_DATA
        println("Key byte error");
#endif
        return STATE_DATA_ERROR;
    }
    if (!verifyChecksum(frame.frame, SPI_FRAME_SIZE - 1, frame.pack.checksum))
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
        bool isSuccess = false;
        bool isHandled = true;
        bool useDefaultReponse = true;

        switch (mRxDataFrame.pack.command)
        {
        case CMD_SYNC_SETTINGS:
        {
            memcpy(&mSettingsData, mRxDataFrame.pack.data, sizeof(SettingsData));
            println("autoSend: %d", mSettingsData.autoSend);
            isSuccess = true;
            break;
        }
        case CMD_START_ZERO_DETECTION:
        {
            int index = mRxDataFrame.pack.data[0];
            isSuccess = startZeroDetection(index);
            break;
        }
        case CMD_SET_POSITION:
        {
            ServoReqData servoReqData;
            memcpy(&servoReqData, mRxDataFrame.pack.data, sizeof(ServoReqData));
            println("servoReqData %d %.2f", servoReqData.index, servoReqData.position);
            isSuccess = requestPosition(servoReqData.index, servoReqData.position);
            break;
        }
        case CMD_GET_SERVO_PARAMS:
        {
            ServoParamsData servoParamsData;
            memcpy(&servoParamsData, mRxDataFrame.pack.data, sizeof(ServoParamsData));
            uint8_t index = servoParamsData.index;
            if (index >= 0 && index < SERVO_NUMS)
            {
                isSuccess = true;
                useDefaultReponse = false;
                memset(&servoParamsData, 0, sizeof(ServoParamsData));
                servoParamsData.index = index;
                servoParamsData.minPosition = mServo[index]->getMinPostion();
                servoParamsData.maxPosition = mServo[index]->getMaxPostion();
                create(mTxDataFrame, mRxDataFrame.pack.command, &servoParamsData, sizeof(ServoParamsData), RESP_CODE_SUCCESS);
            }
            break;
        }
        default:
        {
            isHandled = false;
            break;
        }
        }

        if (!isHandled)
        {
            if (mRxDataFrame.pack.command != CMD_PING)
            {
                println("Unhanled command %d", mRxDataFrame.pack.command);
            }

            if (mSettingsData.autoSend)
            {
                ServoData servoData;
                for (int i = 0; i < SERVO_NUMS; i++)
                {
                    servoData.position[i] = mServo[i]->getCurrentPosition();
                }
                memcpy(mTxDataFrame.pack.data, &servoData, SPI_DATA_SIZE);
                create(mTxDataFrame, CMD_SERVO_DATA, RESP_CODE_SUCCESS);
            }
            else if (mTxDataFrame.pack.command != CMD_PING)
            {
                create(mTxDataFrame, CMD_PING, nullptr, 0, RESP_CODE_SUCCESS);
            }
        }
        else if (useDefaultReponse)
        {
            create(mTxDataFrame, mRxDataFrame.pack.command, isSuccess ? RESP_CODE_SUCCESS : RESP_CODE_ERROR);
        }
    }

#if DEBUG_FRAME_DATA
    println("send: %s", getString(mTxDataFrame).c_str());
#endif
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
}

void DeviceController::run()
{
    if (mState != STATE_DISCONNECTED && HAL_GetTick() > mLastTime)
    {
        // println("Timeout...");
        mLastTime = HAL_GetTick() + CONNECTION_TIMEOUT;
        setState(STATE_DISCONNECTED);
    }

    static uint32_t timeTick = 0;
    static double position = 0;
    if (HAL_GetTick() > timeTick && position != mServo[0]->getCurrentPosition())
    {
        timeTick = HAL_GetTick() + 10;
        position = mServo[0]->getCurrentPosition();
        println("%.2f %.2f %.2f",
                   mServo[0]->getRequestedPosition(),
                   mServo[0]->getCurrentPosition(),
                   100 * mServo[0]->getControlValue() / SERVO_PWM_RESOLUTION);
    }
}

bool DeviceController::startZeroDetection(int index)
{
    if (index < 0 || index >= SERVO_NUMS)
    {
        println("Invalid servo index %d", index);
        return false;
    }
    println("Servo %d start zero detection", index);
    mServo[index]->zeroDetect();
    return true;
}

bool DeviceController::requestPosition(int index, float position)
{
    if (index < 0 || index >= SERVO_NUMS)
    {
        println("Invalid servo index %d", index);
        return false;
    }
    return mServo[index]->requestPosition(position);
}

float DeviceController::getCurrentPosition(int index)
{
    if (index < 0 || index >= SERVO_NUMS)
    {
        println("Invalid servo index %d", index);
        return 0;
    }
    return mServo[index]->getCurrentPosition();
}