#ifndef INC_DEVICECONTROLLER_H_
#define INC_DEVICECONTROLLER_H_

#include <string>
#include "main.h"
#include "PacketPacker.h"
#include "Servo.h"

using namespace std;

class DeviceController
{
private:
    SPI_HandleTypeDef *mHspi;
    DataFrame mRxDataFrame;
    DataFrame *mPreTxFramePtr;
    CommandType_e mCurrentComamnd;
    uint32_t mCmdTimeTick;

    // Data for SPI transfer
    DataFrame mDataFrameMap[CMD_MAX];

    Servo *mServo[SERVO_NUMS];
    int mMonitorIndex;

public:
    DeviceController(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, SPI_HandleTypeDef *hspi);

    void onEncoderEvent(uint16_t pin);
    void onZeroDetected(int index);
    void onControllerInterrupt();

    void startSpiTransfer(DataFrame &txFrame, bool force = false);
    void resetSpiTransfer();
    void onDataReceived();
    void onDataError();
    void run();

    void tune(int index, PidParams params);
    void debugMotor(int index);
    void startMonitor(int index);
    void stopMonitor();
    bool startZeroDetection(int index);
    bool requestPosition(int index, float position);
    float getCurrentPosition(int index);

    void forceOutput(int index, int pwmValue);
    void enableServo(int index);
    void enableServos();
    void disableServo(int index);
    void disableServos();
    void reset(int index, double position = 0);

private:
};

#endif /* INC_DEVICECONTROLLER_H_ */
