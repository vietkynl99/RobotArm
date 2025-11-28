#include "DeviceController.h"
#include "Log.h"
#include <cstring>

#define CONNECTION_TIMEOUT 1000
#define DEBUG_FRAME_DATA (0)

DeviceController::DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    mServo[0] = new Servo(htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, M1_E1_GPIO_Port, M1_E1_Pin, M1_E2_GPIO_Port, M1_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160000, 170000, 180}, PidParams{100, 0, 10});
    mServo[1] = new Servo(htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, M2_E1_GPIO_Port, M2_E1_Pin, M2_E2_GPIO_Port, M2_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0});
    mServo[2] = new Servo(htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, M3_E1_GPIO_Port, M3_E1_Pin, M3_E2_GPIO_Port, M3_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0});
    mServo[3] = new Servo(htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, M4_E1_GPIO_Port, M4_E1_Pin, M4_E2_GPIO_Port, M4_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0});
    mServo[4] = new Servo(htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, M5_E1_GPIO_Port, M5_E1_Pin, M5_E2_GPIO_Port, M5_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0});
    mServo[5] = new Servo(htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, M6_E1_GPIO_Port, M6_E1_Pin, M6_E2_GPIO_Port, M6_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0});

    memset(&mTxDataFrame, 0, sizeof(DataFrame));
    memset(&mRxDataFrame, 0, sizeof(DataFrame));
    setState(STATE_DISCONNECTED);
    mLastTime = 0;
    mSettingsData.autoSend = false;
    mMonitorIndex = -1;

    mHspi = hspi;
    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.frame, mRxDataFrame.frame, SPI_FRAME_SIZE);
}

const char *DeviceController::deviceStateToString(int deviceState)
{
    switch (deviceState)
    {
    case STATE_DISCONNECTED:
        return "Disconnected";
    case STATE_DATA_ERROR:
        return "Data error";
    case STATE_CONNECTED:
        return "Connected";
    default:
        return "Unknown";
    }
}

void DeviceController::forceOutput(int index, int pwmValue)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        if (mServo[index]->getState() != SERVO_STATE_DISABLED)
        {
            println("Servo %d is not disabled");
        }
        else
        {
            println("forceOutput servo %d: %d", index, pwmValue);
            mServo[index]->setOutput(pwmValue);
        }
    }
}

void DeviceController::enableServo(int index)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        if (mServo[index]->getState() != SERVO_STATE_RUNNING)
        {
            println("Enable servo %d", index);
            mServo[index]->setState(SERVO_STATE_RUNNING);
        }
    }
}

void DeviceController::enableServos()
{
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        enableServo(i);
    }
}

void DeviceController::disableServo(int index)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        if (mServo[index]->getState() != SERVO_STATE_DISABLED)
        {
            println("Disable servo %d", index);
            mServo[index]->setState(SERVO_STATE_DISABLED);
        }
    }
}

void DeviceController::disableServos()
{
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        disableServo(i);
    }
}
void DeviceController::reset(int index, double position)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        mServo[index]->reset(position);
    }
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
        println("State changed to %s", deviceStateToString(mState));
        if (mState != STATE_CONNECTED)
        {
            create(mTxDataFrame, CMD_DATA_ERROR, nullptr, 0, RESP_CODE_ERROR);
            disableServos();
        }
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
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        if (pin == mServo[i]->getE1Pin())
        {
            mServo[i]->onEncoderEvent();
            break;
        }
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
            println("Settings are synchronized");
            isSuccess = true;
            break;
        }
        case CMD_START_ZERO_DETECTION:
        {
            int index = mRxDataFrame.pack.data[0];
            isSuccess = startZeroDetection(index);
            break;
        }
        case CMD_CHANGE_POSITION:
        {
            ServoReqData data;
            memcpy(&data, mRxDataFrame.pack.data, sizeof(ServoReqData));
            isSuccess = requestPosition(data.index, data.position);
            break;
        }
        case CMD_RESET:
        {
            ServoReqData data;
            memcpy(&data, mRxDataFrame.pack.data, sizeof(ServoReqData));
            if (data.index >= 0 && data.index < SERVO_NUMS)
            {
                println("Reset servo %d", data.index);
                mServo[data.index]->reset();
                isSuccess = true;
            }
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
                servoParamsData.minPosition = mServo[index]->getPositionLimit().min;
                servoParamsData.maxPosition = mServo[index]->getPositionLimit().max;
                create(mTxDataFrame, mRxDataFrame.pack.command, &servoParamsData, sizeof(ServoParamsData), RESP_CODE_SUCCESS);
            }
            break;
        }
        case CMD_ESTOP:
        {
            println("E Stop");
            disableServos();
            isSuccess = true;
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
                ServoRespData data;
                data.index = mSettingsData.focusedIndex;
                data.state = mServo[data.index]->getState();
                data.requestedPosition = mServo[data.index]->getRequestedPosition();
                data.currentPosition = mServo[data.index]->getCurrentPosition();
                data.controlValue = mServo[data.index]->getControlValue();
                memcpy(mTxDataFrame.pack.data, &data, SPI_DATA_SIZE);
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

    if (mMonitorIndex >= 0 && mMonitorIndex < SERVO_NUMS)
    {
        static uint32_t preTimeTick = 0, timeTick = 0;
        static double prePosition = 0, position = 0;
        static double speed = 0;

        double currentPositon = mServo[mMonitorIndex]->getCurrentPosition();

        if (HAL_GetTick() - preTimeTick > 1000)
        {
            preTimeTick = HAL_GetTick();
            speed = currentPositon - prePosition;
            prePosition = currentPositon;
        }
        if (HAL_GetTick() - timeTick > 100 && abs(position - currentPositon) > 1)
        {
            timeTick = HAL_GetTick();
            position = currentPositon;
            println("E%d S%.2f F%.2f V%.2f %.2fdeg/s %.2frpm",
                    mServo[mMonitorIndex]->getEncoderPluse(),
                    mServo[mMonitorIndex]->getRequestedPosition(),
                    currentPositon,
                    mServo[mMonitorIndex]->getControlValue(),
                    speed,
                    speed / 6);
        }
    }
}

void DeviceController::tune(int index, PidParams params)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        mServo[index]->tune(params);
    }
}

void DeviceController::debugMotor(int index)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        mServo[index]->printData();
    }
}

void DeviceController::startMonitor(int index)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        println("Start debugging %d", index);
        mMonitorIndex = index;
    }
}

void DeviceController::stopMonitor()
{
    println("Stop debugging");
    mMonitorIndex = -1;
}

bool DeviceController::startZeroDetection(int index)
{
    if (index >= 0 && index < SERVO_NUMS && mServo[index]->zeroDetect())
    {
        println("Servo %d start zero detection", index);
        return true;
    }
    else
    {
        return false;
    }
}

bool DeviceController::requestPosition(int index, float position)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        return mServo[index]->requestPosition(position);
    }
    return false;
}

float DeviceController::getCurrentPosition(int index)
{
    if (index >= 0 && index < SERVO_NUMS)
    {
        return mServo[index]->getCurrentPosition();
    }
    return 0;
}
