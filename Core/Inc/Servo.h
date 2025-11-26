#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "RobotArm.h"
#include "PidController.h"

#define SERVO_SAMPLE_TIME_S         (1E-3)  // must matched with timer interrupt
#define SERVO_PWM_RESOLUTION        (999)   // must matched with timer pwm generator
#define SERVO_FIXED_PWN_OUT         (170)   // the minimum value of pwm that the motor can run
#define SERVO_FIXED_PWN_IN          (0.3)   // the minimum value of pwm that the motor can run
#define SERVO_ENABLE_ERR_DETECTION  (1)     // enable the error detection
#define SERVO_ZERO_DETECTION_SPEED  (10)    // [rpm] enable the error detection

enum ServoState
{
    SERVO_STATE_ERROR,
    SERVO_STATE_DISABLED,
    SERVO_STATE_ZERO_DETECTION,
    SERVO_STATE_POSITION
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

    GPIO_TypeDef *mE1GPIO;
    GPIO_TypeDef *mE2GPIO;
    uint16_t mE1Pin;
    uint16_t mE2Pin;
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
    ServoState mState;
    uint32_t mOriginTime;
#if SERVO_ENABLE_ERR_DETECTION
    uint8_t mTick;
    uint8_t mInvalidCount;
#endif

public:
    Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2,
            GPIO_TypeDef *e1GPIO, uint16_t e1Pin, GPIO_TypeDef *e2GPIO, uint16_t e2Pin,
            double gearRatio, double pulsePerRevolution,
            double minPosition, double maxPosition, double zeroPosition,
            double kp, double ki, double kd);
    ~Servo();

    void onEncoderEvent();
    void onZeroDectected();

    void setState(ServoState state);
    int getState();

    void tune(PidParams params);
    void run();
    void reset(double position = 0);
    bool zeroDetect();
    bool requestPosition(double postion);
    bool requestSpeed(double speed);

    uint16_t getE1Pin();
    uint16_t getE2Pin();

    double getMinPostion();
    double getMaxPostion();

    int getEncoderPluse();
    double getRequestedPosition();
    double getCurrentPosition();
    double getControlValue();

private:
    double map(double input, double inMin, double inMax, double outMin, double outMax);
    void setOutput(int value);
};
#endif /* INC_SERVO_H_ */
