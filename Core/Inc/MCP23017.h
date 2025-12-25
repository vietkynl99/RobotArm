#pragma once

#include "main.h"
#include "SoftI2c.h"

#define MCP23017_ADDRESS 0x20

enum MCP23017_Pin
{
    MCP23017_A0_Pin = 0,
    MCP23017_A1_Pin,
    MCP23017_A2_Pin,
    MCP23017_A3_Pin,
    MCP23017_A4_Pin,
    MCP23017_A5_Pin,
    MCP23017_A6_Pin,
    MCP23017_A7_Pin,
    MCP23017_B0_Pin,
    MCP23017_B1_Pin,
    MCP23017_B2_Pin,
    MCP23017_B3_Pin,
    MCP23017_B4_Pin,
    MCP23017_B5_Pin,
    MCP23017_B6_Pin,
    MCP23017_B7_Pin
};

class MCP23017
{
private:
    SoftI2c *mI2c;
    uint8_t mAddress;
    bool mInterruptFlag;
    void (*mOnGpioExt)(MCP23017_Pin);

    bool mErrorFlag;
    uint8_t mGpioA;

public:
    MCP23017(SoftI2c *i2c, uint8_t address = MCP23017_ADDRESS, void (*extCallback)(MCP23017_Pin) = nullptr);
    bool init();
    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &out);
    void run();
    void interrupt();

    bool isRunning();
};