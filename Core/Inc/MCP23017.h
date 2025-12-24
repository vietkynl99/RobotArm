#pragma once

#include "main.h"
#include "SoftI2c.h"

#define MCP23017_ADDRESS 0x20

class MCP23017
{
private:
    SoftI2c *mI2c;
    uint8_t mAddress;

public:
    MCP23017(SoftI2c *i2c, uint8_t address = MCP23017_ADDRESS);
    bool init();
    bool writeReg(uint8_t reg, uint8_t value);
};