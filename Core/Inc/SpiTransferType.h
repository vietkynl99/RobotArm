#pragma once

#include <cstdint>

#define MAGIC_NUMBER 0xABCD
#define SPI_DATA_SIZE (24)

enum CommandType_e
{
    CMD_NONE,
    CMD_GET_JOINT_STATE,
    // Add more command types as needed
    CMD_MAX
};

typedef struct JointStateMessage
{
    static constexpr int CMD = CMD_GET_JOINT_STATE;
    uint8_t index;
    uint8_t mode;
    uint8_t state;
    float kp;
    float kd;
    float ki;
    float setpoint;
    float position;
} JointStateMessage_t;

typedef union
{
    uint8_t data[SPI_DATA_SIZE];
    // Add other packed structures as needed
    JointStateMessage_t jointState;
} PackedData;

#pragma pack(push,1)
typedef struct
{
    uint16_t magicNumber;
    uint8_t command;
    PackedData data;
    uint8_t checksum;
} DataFrame;
#pragma pack(pop)

static_assert(sizeof(float) == 4, "float size is not 4 bytes");
static_assert(sizeof(JointStateMessage_t) <= SPI_DATA_SIZE, "JointStateMessage_t size exceeds SPI_DATA_SIZE");
