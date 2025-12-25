#pragma once

#include <cstdint>

#define SERVO_NUMS (6)
#define MAGIC_NUMBER 0xABCD
#define SPI_DATA_SIZE (64)

// Private Data Structures
typedef struct
{
    float kp;
    float ki;
    float kd;
} PidParams;

typedef struct
{
    float ratio;
    int encoderResolution;
} GearBox;

typedef struct
{
    float min;
    float max;
    float zero;
} PositionLimit;

// Command Types
enum CommandType_e
{
    CMD_NONE,
    CMD_SET_JOINT_SETTING,
    CMD_GET_JOINT_STATUS,
    CMD_SET_GET_MULTI_DOF_STATUS,
    // Add more command types as needed
    CMD_MAX
};

// Data Structures for each command
typedef struct JointSettingMessage
{
    static constexpr int CMD = CMD_SET_JOINT_SETTING;
    uint8_t index;
    GearBox gearBox;
    PositionLimit positionLimit;
    PidParams pidParams;
} JointSettingMessage_t;

typedef struct JointStatusMessage
{
    static constexpr int CMD = CMD_GET_JOINT_STATUS;
    uint8_t index;
    uint8_t state;
    float setpoint;
    float position;
} JointStatusMessage_t;

typedef struct MultiDOFStatusMessage
{
    static constexpr int CMD = CMD_SET_GET_MULTI_DOF_STATUS;
    uint8_t mode[SERVO_NUMS];
    uint8_t state[SERVO_NUMS];
    float setpoint[SERVO_NUMS];
    float position[SERVO_NUMS];
} MultiDOFStatusMessage_t;

typedef union
{
    uint8_t data[SPI_DATA_SIZE];
    // Add other packed structures as needed
    JointSettingMessage_t jointSetting;
    JointStatusMessage_t jointStatus;
    MultiDOFStatusMessage_t multiDOFStatus;
} PackedData;

#pragma pack(push, 1)
typedef struct
{
    uint16_t magicNumber;
    uint8_t command;
    PackedData data;
    uint16_t crc;
} DataFrame;
#pragma pack(pop)

static_assert(sizeof(float) == 4, "float size is not 4 bytes");
static_assert(sizeof(JointSettingMessage_t) <= SPI_DATA_SIZE, "JointSettingMessage_t size exceeds SPI_DATA_SIZE");
static_assert(sizeof(JointStatusMessage_t) <= SPI_DATA_SIZE, "JointStatusMessage_t size exceeds SPI_DATA_SIZE");
static_assert(sizeof(MultiDOFStatusMessage_t) <= SPI_DATA_SIZE, "MultiDOFStatusMessage_t size exceeds SPI_DATA_SIZE");
