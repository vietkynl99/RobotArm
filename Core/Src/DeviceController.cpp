#include "DeviceController.h"
#include "Log.h"
#include <cstring>

#define CMD_REQUEST_PING 1

#define CMD_RESPONSE_PING 101
#define CMD_RESPONSE_DATA_ERROR 102

DeviceController::DeviceController(SPI_HandleTypeDef *hspi)
{
    mHspi = hspi;
    memset(&mTxDataFrame, 0, sizeof(DataFrame));
    memset(&mRxDataFrame, 0, sizeof(DataFrame));
    mStatus = STATUS_DISCONNECTED;

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

void DeviceController::createDataFrame(DataFrame &dataFrame, uint8_t command, const uint8_t *data, size_t length)
{
    memset(&dataFrame, 0, sizeof(DataFrame));
    dataFrame.start = SPI_DATA_START_BYTE;
    dataFrame.command = command;
    if (data && length > 0 && length <= SPI_DATA_SIZE)
    {
        memcpy(dataFrame.data, data, length);
    }
    dataFrame.checksum = calculateChecksum(dataFrame.rawData, SPI_FRAME_SIZE - 1);
}

DeviceStatus DeviceController::verifyDataFrame(const DataFrame &frame)
{
    if (frame.start == 0)
    {
        for (int i = 0; i < SPI_FRAME_SIZE; i++)
        {
            if (frame.rawData[i] != 0)
            {
                return STATUS_DATA_ERROR;
            }
        }
        return STATUS_DISCONNECTED;
    }
    else if (frame.start != SPI_DATA_START_BYTE)
    {
        return STATUS_START_BYTE_ERROR;
    }
    if (!verifyChecksum(frame.rawData, SPI_FRAME_SIZE - 1, frame.checksum))
    {
        return STATUS_CHECKSUM_ERROR;
    }
    return STATUS_CONNECTED;
}

void DeviceController::onDataReceived()
{
    DeviceStatus status = verifyDataFrame(mRxDataFrame);
    println("received: %s -> status: %d", getString(mRxDataFrame).c_str(), status);
    if (mStatus != status)
    {
        mStatus = status;
        if (mStatus == STATUS_CONNECTED)
        {
            createDataFrame(mTxDataFrame, CMD_RESPONSE_PING, nullptr, 0);
        }
        else
        {
            createDataFrame(mTxDataFrame, CMD_RESPONSE_DATA_ERROR, nullptr, 0);
        }
    }

    HAL_SPI_TransmitReceive_DMA(mHspi, mTxDataFrame.rawData, mRxDataFrame.rawData, SPI_FRAME_SIZE);
}