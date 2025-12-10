#include "DeviceController.h"
#include "Log.h"
#include <cstring>

static const GearBox GearBox_Motor370_12VDC_72rpm{64, 13};
static const GearBox GearBox_Motor_12VDC_80rpm{98.775, 1};

DeviceController::DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi)
{
    mHspi = hspi;
    mMonitorIndex = 1;
    mCurrentComamnd = CMD_NONE;
    mCmdTimeTick = 0;
    mPreTxFramePtr = nullptr;

    mServo[0] = new Servo(htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, M1_E1_GPIO_Port, M1_E1_Pin, M1_E2_GPIO_Port, M1_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160000, 170000, 180}, PidParams{100, 0, 10, 0.1, 0.5});
    mServo[1] = new Servo(htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, M2_E1_GPIO_Port, M2_E1_Pin, M2_E2_GPIO_Port, M2_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0, 0, 0});
    mServo[2] = new Servo(htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, M3_E1_GPIO_Port, M3_E1_Pin, M3_E2_GPIO_Port, M3_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0, 0, 0});
    mServo[3] = new Servo(htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, M4_E1_GPIO_Port, M4_E1_Pin, M4_E2_GPIO_Port, M4_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0, 0, 0});
    mServo[4] = new Servo(htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, M5_E1_GPIO_Port, M5_E1_Pin, M5_E2_GPIO_Port, M5_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0, 0, 0});
    mServo[5] = new Servo(htim3, TIM_CHANNEL_3, TIM_CHANNEL_4, M6_E1_GPIO_Port, M6_E1_Pin, M6_E2_GPIO_Port, M6_E2_Pin, GearBox_Motor370_12VDC_72rpm, PositionLimit{-160, 170, 180}, PidParams{20, 0, 0, 0, 0});

    memset(&mRxDataFrame, 0, sizeof(DataFrame));

    // Prepare data frames for each command
    for (int i = 0; i < CMD_MAX; i++)
    {
        mDataFrameMap[i] = PacketPacker::create(i, nullptr, 0);
    }

    startSpiTransfer(mDataFrameMap[CMD_NONE]);
}

void DeviceController::run()
{
    // Update data frame for the current command
    if (HAL_GetTick() - mCmdTimeTick > 5)
    {
        mCmdTimeTick = HAL_GetTick();
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

void DeviceController::startSpiTransfer(DataFrame &txFrame, bool force)
{
    if (force || mPreTxFramePtr != &txFrame)
    {
        mPreTxFramePtr = &txFrame;
        resetSpiTransfer();
        int ret = HAL_SPI_TransmitReceive_DMA(mHspi, (uint8_t *)&txFrame, (uint8_t *)&mRxDataFrame, sizeof(DataFrame));
        if (ret != HAL_OK)
        {
            println("HAL_SPI_TransmitReceive_DMA error %d", ret);
        }
    }
}

void DeviceController::resetSpiTransfer()
{
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
    if (!PacketPacker::verify(mRxDataFrame))
    {
        println("Verify failed");
        startSpiTransfer(mDataFrameMap[CMD_NONE], true);
    }
    else
    {
        if (mRxDataFrame.command > CMD_NONE && mRxDataFrame.command < CMD_MAX)
        {
            mCurrentComamnd = (CommandType_e)mRxDataFrame.command;
            startSpiTransfer(mDataFrameMap[mCurrentComamnd]);

            // extra processing for specific commands
            switch (mCurrentComamnd)
            {
            case CMD_SET_JOINT_SETTING:
            {
                int index = mRxDataFrame.data.jointSetting.index;
                if (index < 0 || index > SERVO_NUMS)
                {
                    break;
                }
                mDataFrameMap[mCurrentComamnd].data.jointSetting.index = index;
                mServo[index]->setGearBox(mRxDataFrame.data.jointSetting.gearBox);
                mServo[index]->setPositionLimit(mRxDataFrame.data.jointSetting.positionLimit);
                mServo[index]->tune(mRxDataFrame.data.jointSetting.pidParams);
                break;
            }
            case CMD_GET_JOINT_STATUS:
            {
                int index = mRxDataFrame.data.jointStatus.index;
                if (index < 0 || index > SERVO_NUMS)
                {
                    break;
                }
                mDataFrameMap[mCurrentComamnd].data.jointStatus.index = index;
                break;
            }
            case CMD_SET_GET_MULTI_DOF_STATUS:
            {
                for (int i = 0; i < SERVO_NUMS; i++)
                {
                    mServo[i]->setMode((ServoMode)mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.mode[i]);
                    mServo[i]->setState((ServoState)mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.state[i]);
                    mServo[i]->requestPosition(mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.setpoint[i]);
                }
                break;
            }
            default:
                break;
            }

            updateResponseFrameData();
        }
        else
        {
            println("Unknown command %d", mRxDataFrame.command);
        }
    }
}

void DeviceController::updateResponseFrameData()
{
    switch (mCurrentComamnd)
    {
    case CMD_SET_JOINT_SETTING:
    {
        uint8_t index = mDataFrameMap[mCurrentComamnd].data.jointSetting.index;
        if (index >= 0 && index < SERVO_NUMS)
        {
            mDataFrameMap[mCurrentComamnd].data.jointSetting.gearBox = mServo[index]->getGearBox();
            mDataFrameMap[mCurrentComamnd].data.jointSetting.positionLimit = mServo[index]->getPositionLimit();
            mDataFrameMap[mCurrentComamnd].data.jointSetting.pidParams = mServo[index]->getPidParams();
            PacketPacker::update(mDataFrameMap[mCurrentComamnd]);
        }
        break;
    }
    case CMD_GET_JOINT_STATUS:
    {
        uint8_t index = mDataFrameMap[mCurrentComamnd].data.jointStatus.index;
        if (index >= 0 && index < SERVO_NUMS)
        {
            mDataFrameMap[mCurrentComamnd].data.jointStatus.mode = static_cast<uint8_t>(mServo[index]->getMode());
            mDataFrameMap[mCurrentComamnd].data.jointStatus.state = static_cast<uint8_t>(mServo[index]->getState());
            mDataFrameMap[mCurrentComamnd].data.jointStatus.setpoint = static_cast<float>(mServo[index]->getRequestedPosition());
            mDataFrameMap[mCurrentComamnd].data.jointStatus.position = static_cast<float>(mServo[index]->getCurrentPosition());
            PacketPacker::update(mDataFrameMap[mCurrentComamnd]);
        }
        break;
    }
    case CMD_SET_GET_MULTI_DOF_STATUS:
    {
        for (int i = 0; i < SERVO_NUMS; i++)
        {
            mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.mode[i] = static_cast<uint8_t>(mServo[i]->getMode());
            mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.state[i] = static_cast<uint8_t>(mServo[i]->getState());
            mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.setpoint[i] = static_cast<float>(mServo[i]->getRequestedPosition());
            mDataFrameMap[mCurrentComamnd].data.multiDOFStatus.position[i] = static_cast<float>(mServo[i]->getCurrentPosition());
            PacketPacker::update(mDataFrameMap[mCurrentComamnd]);
        }
        break;
    }
    default:
        break;
    }
}

void DeviceController::onDataError()
{
    // println("onDataError");
    resetSpiTransfer();
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
