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

enum DeviceStatus
{
    STATUS_DISCONNECTED,
    STATUS_DATA_ERROR,
    STATUS_START_BYTE_ERROR,
    STATUS_CHECKSUM_ERROR,
    STATUS_UNKNOWN_CMD_ERROR,
    STATUS_CONNECTED,
};

class DeviceController
{
private:
    DataFrame mTxDataFrame;
    DataFrame mRxDataFrame;
    SPI_HandleTypeDef *mHspi;
    DeviceStatus mStatus;

public:
    DeviceController(SPI_HandleTypeDef *hspi);

    void onDataReceived();
    
private:
    string getString(const DataFrame &frame);
    uint8_t calculateChecksum(const uint8_t *data, size_t length);
    bool verifyChecksum(const uint8_t *data, size_t length, uint8_t checksum);
    void createDataFrame(DataFrame &dataFrame, uint8_t command, const uint8_t *data, size_t length);
    DeviceStatus verifyDataFrame(const DataFrame &frame);
};

#endif /* INC_DEVICECONTROLLER_H_ */
