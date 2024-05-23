#ifndef INC_DEVICECONTROLLER_H_
#define INC_DEVICECONTROLLER_H_

#include <string>
#include "main.h"
#include "Servo.h"

using namespace std;

#define SERVO_NUMS (6)

#define SPI_DATA_SIZE 			(24)
#define SPI_FRAME_SIZE 			(SPI_DATA_SIZE + 4)
#define SPI_DATA_KEY_BYTE 	    (0x99D7)

typedef struct
{
    int32_t position[SERVO_NUMS];
} ServoData;

typedef union
{
    uint8_t rawData[SPI_FRAME_SIZE];
    struct
    {
        union
        {
            uint8_t data[SPI_DATA_SIZE];
            ServoData servoData;
        };
        uint16_t key;
        uint8_t command;
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

enum RequestCommand
{
    CMD_REQ_PING = 1,
    CMD_REQ_SET_AUTO_GET_SERVO_DATA
};

enum ResponseCommand
{
    CMD_RESP_PING = 101,
    CMD_RESP_DATA_ERROR,
    CMD_RESP_SERVO_DATA
};

class DeviceController
{
private:
    DataFrame mTxDataFrame;
    DataFrame mRxDataFrame;
    SPI_HandleTypeDef *mHspi;
    DeviceState mState;
    uint32_t mLastTime;
    bool mAutoGetData;

    Servo *mServo[SERVO_NUMS];

public:
    DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi);

    void onEncoderEvent(uint16_t pin);
    void onZeroDetected(int index);
    void onControllerInterrupt();

    void onDataReceived();
    void onDataError();
    void run();
    
private:
    string getString(const DataFrame &frame);
    uint8_t calculateChecksum(const uint8_t *data, size_t length);
    bool verifyChecksum(const uint8_t *data, size_t length, uint8_t checksum);
    void setState(DeviceState state);
    void createDataFrame(DataFrame &dataFrame, uint8_t command);
    void createDataFrame(DataFrame &dataFrame, uint8_t command, const uint8_t *data, size_t length);
    DeviceState verifyDataFrame(const DataFrame &frame);
    void handleData();
};

#endif /* INC_DEVICECONTROLLER_H_ */
