#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "RobotArm.h"
#include "PidController.h"

#define SERVO_SAMPLE_TIME_S         (1E-3)  // must matched with timer interrupt
#define SERVO_PWM_RESOLUTION        (999)   // must matched with timer pwm generator
#define SERVO_ENABLE_ERR_DETECTION  (1)     // enable the error detection
#define SERVO_ZERO_DETECTION_SPEED  (10)    // [rpm] enable the error detection

enum ServoState
{
    SERVO_STATE_ERROR,
    SERVO_STATE_DISABLED,
    SERVO_STATE_RUNNING
};

enum ServoMode
{
    SERVO_MODE_SPEED,
    SERVO_MODE_POSITION
};

using namespace std;

class Servo
{
private:
    float mSetpoint;
    float mOutput;
    float mError;
#if SERVO_ENABLE_ERR_DETECTION
    float mPrevError;
#endif

    GPIO_TypeDef *mE1GPIO;
    GPIO_TypeDef *mE2GPIO;
    uint16_t mE1Pin;
    uint16_t mE2Pin;
    int64_t mEncoderPulse;
    float mEncoderResolution;
    float mResolution;
    float mSpeed;
    GearBox mGearBox;
    PositionLimit mPositionLimit;
    PidParams mPidParams;

    PidController *mPidController;
    TIM_HandleTypeDef *mOutputTimer;
    uint16_t mOutputTimerCh1;
    uint16_t mOutputTimerCh2;
    ServoState mState;
    ServoMode mMode;
    uint32_t mOriginTime;
#if SERVO_ENABLE_ERR_DETECTION
    uint8_t mTick;
    uint8_t mInvalidCount;
#endif

public:
    Servo(TIM_HandleTypeDef *outputTimer, uint16_t outputTimerCh1, uint16_t outputTimerCh2,
            GPIO_TypeDef *e1GPIO, uint16_t e1Pin, GPIO_TypeDef *e2GPIO, uint16_t e2Pin,
            GearBox gearBox, PositionLimit positionLimit, PidParams params);
    ~Servo();

    void onEncoderEvent();
    void onZeroDectected();

    void setState(ServoState state);
    void setMode(ServoMode mode);
    ServoState getState();
    ServoMode getMode();
    GearBox getGearBox();
    PositionLimit getPositionLimit();
    PidParams getPidParams();

    void setGearBox(GearBox gearBox);
    void setPositionLimit(PositionLimit positionLimit);
    void tune(PidParams params);
    void run();
    void reset(float position = 0);
    bool zeroDetect();
    bool requestPosition(float postion);
    bool requestSpeed(float speed);

    uint16_t getE1Pin();
    uint16_t getE2Pin();
    int getEncoderPluse();
    float getRequestedPosition();
    float getCurrentPosition();
    float getControlValue();

    void setOutput(int value);
    void printData();

private:
    const char* toString(ServoState value);
    const char* toString(ServoMode value);
};
#endif /* INC_SERVO_H_ */
