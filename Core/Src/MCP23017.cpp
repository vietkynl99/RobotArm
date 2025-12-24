#include "MCP23017.h"

// Config IO as input or output (1=input, 0=output)
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
// Enable pull-up resistor (1=enabled, 0=disabled)
#define MCP23017_GPPUA 0x0C
#define MCP23017_GPPUB 0x0D
// Read GPIO port
#define MCP23017_GPIOA 0x12
#define MCP23017_GPIOB 0x13
// Write GPIO port
#define MCP23017_OLATA 0x14
#define MCP23017_OLATB 0x15
// Interrupt on change control
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05
#define MCP23017_INTCONA 0x08
#define MCP23017_INTCONB 0x09

MCP23017::MCP23017(SoftI2c *i2c, uint8_t address)
    : mI2c(i2c), mAddress(address)
{
}

bool MCP23017::init()
{
    if (!writeReg(MCP23017_IODIRA, 0x00)) // A = output
    {
        return false;
    }
    if (!writeReg(MCP23017_IODIRB, 0xFF)) // B = input
    {
        return false;
    }

    if (!writeReg(MCP23017_GPPUB, 0xFF)) // pull-up input B
    {
        return false;
    }

    if (!writeReg(MCP23017_OLATA, 0x00)) // output A = LOW
    {
        return false;
    }

    // Configure interrupt on change for port B
    if (!writeReg(MCP23017_GPINTENB, 0xFF)) // enable interrupt
    {
        return false;
    }
    if (!writeReg(MCP23017_INTCONB, 0x00)) // any change
    {
        return false;
    }
    return true;
}

bool MCP23017::writeReg(uint8_t reg, uint8_t value)
{
    return mI2c->writeReg(mAddress, reg, value);
}