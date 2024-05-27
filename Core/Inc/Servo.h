#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "RobotArm.h"
#include "PidController.h"

#define SERVO_SAMPLE_TIME_S         (1E-3)  // must matched with timer interrupt
#define SERVO_ENCODER_RESOLUTION    (2)     // pulse per revolution
#define SERVO_PWM_RESOLUTION        (999)   // must matched with timer pwm generator
#define SERVO_FIXED_PWN_OUT         (170)   // the minimum value of pwm that the motor can run
#define SERVO_FIXED_PWN_IN          (0.3)     // the minimum value of pwm that the motor can run

class Servo
{
private:
    double mSetpoint;
    double mOutput;
    double mError;

    int64_t mEncoderPulse;
    double mEncoderResolution;
    double mResolution;
    double mMinPosition;
    double mMaxPosition;
    double mZeroPosition;

    PidController *mPidController;
    TIM_HandleTypeDef *mOutputTimer;
    uint16_t mOutputTimerCh1;
    uint16_t mOutputTimerCh2;
    bool mEnabled;
    bool mZeroChecked;
    bool mIsZeroDetecting;

public:
    Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double gearRatio, double minPosition, double maxPosition, double zeroPosition, double kp, double ki, double kd);
    ~Servo();

    void onEncoderEvent(bool direction);
    void onZeroDectected();

    void setEnable(bool enabled);
    void tune(PidParams params);
    void run();
    void reset(double position = 0);
    void zeroDetect();
    bool requestPosition(double postion);

    double getRequestedPosition();
    double getCurrentPosition();
    double getControlValue();

private:
    double map(double input, double inMin, double inMax, double outMin, double outMax);
    void setOutput(int value);
};
#endif /* INC_SERVO_H_ */
