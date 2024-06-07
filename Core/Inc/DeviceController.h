#ifndef INC_DEVICECONTROLLER_H_
#define INC_DEVICECONTROLLER_H_

#include <string>
#include <cassert>
#include "main.h"
#include "Servo.h"

using namespace std;

#define SERVO_NUMS          (6)

#define SPI_DATA_SIZE       (24)
#define SPI_FRAME_SIZE      (SPI_DATA_SIZE + 6)
#define SPI_DATA_KEY1 	    (0x99)
#define SPI_DATA_KEY2       (0xD7)

// For testing
#define SERVO_TEST_INDEX    (0)

typedef struct
{
    uint8_t index;
	float minPosition;
	float maxPosition;
} ServoParamsData;

typedef struct SettingsData
{
    bool autoSend;
    uint8_t focusedIndex;

    SettingsData()
    {
        autoSend = false;
        focusedIndex = 0;
    }
} SettingsData;

typedef struct
{
	float position;
    uint8_t index;
} ServoReqData;

typedef struct
{
    int index;
    uint8_t mode;
    uint8_t zeroDetectionState;
    float requestedPosition;
	float currentPosition;
    float controlValue;
} ServoRespData;

enum eResponsecode 
{
    RESP_CODE_SUCCESS = 100,
    RESP_CODE_ERROR
};

typedef struct
{
	uint8_t unused;
	uint8_t key1;
	uint8_t key2;
	uint8_t command;
    uint8_t data[SPI_DATA_SIZE];
    uint8_t responseCode;
	uint8_t checksum;
} PackedData;

typedef union
{
	uint8_t frame[SPI_FRAME_SIZE];
	PackedData pack;
} DataFrame;

static_assert(sizeof(float) == 4);
static_assert(sizeof(ServoParamsData) <= SPI_DATA_SIZE);
static_assert(sizeof(SettingsData) <= SPI_DATA_SIZE);
static_assert(sizeof(ServoReqData) <= SPI_DATA_SIZE);
static_assert(sizeof(ServoRespData) <= SPI_DATA_SIZE);
static_assert(sizeof(PackedData) == SPI_FRAME_SIZE);
static_assert(sizeof(DataFrame) == SPI_FRAME_SIZE);

enum DeviceState
{
    STATE_DISCONNECTED,
    STATE_DATA_ERROR,
    STATE_UNKNOWN_CMD_ERROR,
    STATE_CONNECTED,
};

enum DataCommand
{
    CMD_PING = 1,
    CMD_START_ZERO_DETECTION,
    CMD_SET_POSITION,
    CMD_SYNC_SETTINGS,
    CMD_GET_SERVO_PARAMS,

    CMD_DATA_ERROR = 200,
    CMD_SERVO_DATA
};

class DeviceController
{
private:
    DataFrame mTxDataFrame;
    DataFrame mRxDataFrame;
    SPI_HandleTypeDef *mHspi;
    DeviceState mState;
    uint32_t mLastTime;
    SettingsData mSettingsData;

    Servo *mServo[SERVO_NUMS];

public:
    DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi);

    void onEncoderEvent(uint16_t pin);
    void onZeroDetected(int index);
    void onControllerInterrupt();

    void onDataReceived();
    void onDataError();
    void run();

    bool startZeroDetection(int index);
    bool requestPosition(int index, float position);
    float getCurrentPosition(int index);
    
private:
    string getString(const DataFrame &frame);
    uint8_t calculateChecksum(const uint8_t *data, size_t length);
    bool verifyChecksum(const uint8_t *data, size_t length, uint8_t checksum);
    void setState(DeviceState state);
    void create(DataFrame &dataFrame, uint8_t command, uint8_t responseCode);
    void create(DataFrame &dataFrame, uint8_t command, const void *data, size_t length, uint8_t responseCode);
    DeviceState verifyDataFrame(const DataFrame &frame);
    void handleData();
};

#endif /* INC_DEVICECONTROLLER_H_ */
