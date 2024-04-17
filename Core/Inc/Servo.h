#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "RobotArm.h"
#include "PidController.h"

class Servo
{
private:
    double mSetpoint;
    double mOutput;
    double mError;
    double mFeedback;

    uint64_t mEncoderPulse;
    double mEncoderResolution;

    PidController *mPidController;
    TIM_HandleTypeDef *mOutputTimer;
    uint16_t mOutputTimerCh1;
    uint16_t mOutputTimerCh2;

public:
    Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double sampleTime, uint16_t pulsePerRev);
    ~Servo();

    void onEncoderEvent(bool direction);
    void run();
    void reset();
    void requestPosition(double postion);
    double getRequestedPosition();
    double getCurrentPosition();

private:
};
#endif /* INC_SERVO_H_ */
