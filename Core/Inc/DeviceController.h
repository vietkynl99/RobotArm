#ifndef INC_DEVICECONTROLLER_H_
#define INC_DEVICECONTROLLER_H_

#include "main.h"
#include <string>

using namespace std;

#define SPI_FRAME_SIZE 			(10)
#define SPI_DATA_SIZE 			(SPI_FRAME_SIZE - 3)
#define SPI_DATA_START_BYTE 	(153)

typedef union
{
	uint8_t rawData[SPI_FRAME_SIZE];
	struct
	{
		uint8_t start;
		uint8_t command;
		uint8_t data[SPI_DATA_SIZE];
		uint8_t checksum;
	};
} DataFrame;

enum DeviceState
{
    STATE_DISCONNECTED,
    STATE_DATA_ERROR,
    STATE_UNKNOWN_CMD_ERROR,
    STATE_CONNECTED,
};

class DeviceController
{
private:
    DataFrame mTxDataFrame;
    DataFrame mRxDataFrame;
    SPI_HandleTypeDef *mHspi;
    DeviceState mState;
    uint32_t mLastTime;

public:
    DeviceController(SPI_HandleTypeDef *hspi);

    void onDataReceived();
    void onDataError();
    void run();
    
private:
    string getString(const DataFrame &frame);
    uint8_t calculateChecksum(const uint8_t *data, size_t length);
    bool verifyChecksum(const uint8_t *data, size_t length, uint8_t checksum);
    void setState(DeviceState state);
    void createDataFrame(DataFrame &dataFrame, uint8_t command, const uint8_t *data, size_t length);
    DeviceState verifyDataFrame(const DataFrame &frame);
};

#endif /* INC_DEVICECONTROLLER_H_ */
