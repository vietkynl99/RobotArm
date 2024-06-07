#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "RobotArm.h"
#include "PidController.h"

#define SERVO_SAMPLE_TIME_S         (1E-3)  // must matched with timer interrupt
#define SERVO_ENCODER_RESOLUTION    (2)     // pulse per revolution
#define SERVO_PWM_RESOLUTION        (999)   // must matched with timer pwm generator
#define SERVO_FIXED_PWN_OUT         (170)   // the minimum value of pwm that the motor can run
#define SERVO_FIXED_PWN_IN          (0.3)   // the minimum value of pwm that the motor can run
#define SERVO_ENABLE_ERR_DETECTION  (1)     // enable the error detection
#define SERVO_ZERO_DETECTION_SPEED  (3)    // [rpm] enable the error detection

enum ServoMode
{
    SERVO_MODE_DISABLED,
    SERVO_MODE_SPEED,
    SERVO_MODE_POSITION
};

enum ZeroDetectionState
{
    ZERO_DETECTION_NONE,
    ZERO_DETECTION_RUNNING,
    ZERO_DETECTION_FINISHED
};

class Servo
{
private:
    double mSetpoint;
    double mOutput;
    double mError;
#if SERVO_ENABLE_ERR_DETECTION
    double mPrevError;
#endif

    int64_t mEncoderPulse;
    double mEncoderResolution;
    double mResolution;
    double mMinPosition;
    double mMaxPosition;
    double mZeroPosition;
    double mSpeed;

    PidController *mPidController;
    TIM_HandleTypeDef *mOutputTimer;
    uint16_t mOutputTimerCh1;
    uint16_t mOutputTimerCh2;
    int mMode;
    int8_t mZeroDetectionState;
    uint32_t mOriginTime;
#if SERVO_ENABLE_ERR_DETECTION
    uint8_t mTick;
    uint8_t mInvalidCount;
#endif

public:
    Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2, double gearRatio, double minPosition, double maxPosition, double zeroPosition, double kp, double ki, double kd);
    ~Servo();

    void onEncoderEvent(bool direction);
    void onZeroDectected();

    void setMode(int mode);
    int getMode();

    int getZeroDetectionState();

    void tune(PidParams params);
    void run();
    void reset(double position = 0);
    void zeroDetect();
    bool requestPosition(double postion);
    bool requestSpeed(double speed);

    double getMinPostion();
    double getMaxPostion();

    double getRequestedPosition();
    double getCurrentPosition();
    double getControlValue();

private:
    double map(double input, double inMin, double inMax, double outMin, double outMax);
    void setOutput(int value);
};
#endif /* INC_SERVO_H_ */
