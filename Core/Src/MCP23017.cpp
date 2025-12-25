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
#define MCP23017_DEFVALA 0x06
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTFA 0x0E
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPA 0x10
#define MCP23017_INTCAPB 0x11
#define MCP23017_IOCON 0x0A

MCP23017::MCP23017(SoftI2c *i2c, uint8_t address, void (*extCallback)(MCP23017_Pin))
    : mI2c(i2c), mAddress(address), mInterruptFlag(true), mOnGpioExt(extCallback), mErrorFlag(false), mGpioA(0)
{
}

bool MCP23017::init()
{
    if (!writeReg(MCP23017_IODIRA, 0xFF) ||   // A = input
        !writeReg(MCP23017_IODIRB, 0xFF) ||   // B = input
        !writeReg(MCP23017_GPPUA, 0xFF) ||    // Pull-up enabled
        !writeReg(MCP23017_GPINTENA, 0xFF) || // Enable interrupt on change
        !writeReg(MCP23017_INTCONA, 0x00))    // Compare to previous value
    {
        return false;
    }

    return true;
}

bool MCP23017::writeReg(uint8_t reg, uint8_t value)
{
    if (mI2c->writeReg(mAddress, reg, value))
    {
        return true;
    }
    else
    {
        println("writeReg 0x%02X failed", reg);
        mErrorFlag = true;
        return false;
    }
}

bool MCP23017::readReg(uint8_t reg, uint8_t &out)
{
    if (mI2c->readReg(mAddress, reg, out))
    {
        return true;
    }
    else
    {
        println("readReg 0x%02X failed", reg);
        mErrorFlag = true;
        return false;
    }
}

void MCP23017::run()
{
    if (mErrorFlag)
    {
        return;
    }
    if (mInterruptFlag /* || HAL_GPIO_ReadPin(IO_EXPANDER_INTA_GPIO_Port, IO_EXPANDER_INTA_Pin) == GPIO_PIN_RESET */)
    {
        uint8_t gpioA;
        if (readReg(MCP23017_INTCAPA, gpioA))
        {
            mInterruptFlag = false;
            if (mGpioA != gpioA)
            {
                println("MCP23017 A: 0x%02X -> 0x%02X", mGpioA, gpioA);
                if (mOnGpioExt)
                {
                    for (uint8_t i = 0; i < 8; i++)
                    {
                        bool oldBit = mGpioA & (1 << i);
                        bool newBit = gpioA & (1 << i);
                        // Falling edge
                        if (oldBit != newBit && !newBit)
                        {
                            mOnGpioExt((MCP23017_Pin)i);
                        }
                    }
                }
                mGpioA = gpioA;
            }
        }
    }
}

void MCP23017::interrupt()
{
    mInterruptFlag = true;
}

bool MCP23017::isRunning()
{
    return !mErrorFlag;
}