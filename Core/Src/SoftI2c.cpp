#include "SoftI2c.h"

SoftI2c::SoftI2c(GPIO_TypeDef *sdaGpioPort, uint16_t sdaPin, GPIO_TypeDef *sclGpioPort, uint16_t sclPin)
    : mSdaGpioPort(sdaGpioPort), mSclGpioPort(sclGpioPort), mSDAPin(sdaPin), mSCLPin(sclPin)
{
}

bool SoftI2c::begin()
{
    if (!verifyBus())
    {
        println("I2C bus error");
        return false;
    }
    return true;
}

void SoftI2c::start()
{
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
    _delay();
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_RESET);
    _delay();
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);
}

void SoftI2c::stop()
{
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
    _delay();
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);
    _delay();
}

bool SoftI2c::writeByte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        (data & 0x80) ? HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET) : HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_RESET);
        _delay();
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
        _delay();
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);
        data <<= 1;
    }

    // ACK
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);
    _delay();
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
    bool ack = HAL_GPIO_ReadPin(mSdaGpioPort, mSDAPin) == GPIO_PIN_RESET;
    _delay();
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);

    return ack;
}

uint8_t SoftI2c::readByte(bool ack)
{
    uint8_t data = 0;
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);

    for (int i = 0; i < 8; i++)
    {
        data <<= 1;
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
        _delay();
        if (HAL_GPIO_ReadPin(mSdaGpioPort, mSDAPin))
            data |= 1;
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);
        _delay();
    }

    // ACK/NACK
    ack ? HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_RESET) : HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);
    _delay();
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_SET);
    _delay();
    HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, GPIO_PIN_SET);

    return data;
}

void SoftI2c::_delay()
{
    for (volatile int i = 0; i < 50; i++)
    {
    }
}

bool SoftI2c::writeReg(uint8_t addr, uint8_t reg, uint8_t val)
{
    start();

    if (!writeByte((addr << 1) | 0) || !writeByte(reg) || !writeByte(val))
    {
        stop();
        return false;
    }

    stop();
    return true;
}

bool SoftI2c::readReg(uint8_t addr, uint8_t reg, uint8_t &out)
{
    start();

    if (!writeByte((addr << 1) | 0) || !writeByte(reg))
    {
        stop();
        return false;
    }

    start();

    if (!writeByte((addr << 1) | 1))
    {
        stop();
        return false;
    }

    out = readByte(false); // NACK last byte
    stop();

    return true;
}

bool SoftI2c::probe(uint8_t addr)
{
    start();

    bool isOk = writeByte((addr << 1) | 0); // WRITE

    stop();

    return isOk;
}

bool SoftI2c::verifyBus(void)
{
    GPIO_PinState state = GPIO_PIN_RESET;
    for (int i = 0; i < 3; i++)
    {
        state = state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(mSdaGpioPort, mSDAPin, state);
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, state);
        _delay();
        if (HAL_GPIO_ReadPin(mSdaGpioPort, mSDAPin) != state || HAL_GPIO_ReadPin(mSclGpioPort, mSCLPin) != state)
        {
            stop();
            return false;
        }
    }

    stop();
    return true;
}

void SoftI2c::test()
{
    uint32_t startTime = HAL_GetTick();
    for (uint32_t i = 0; i < 100000UL; i++)
    {
        HAL_GPIO_WritePin(mSdaGpioPort, mSCLPin, GPIO_PIN_RESET);
        _delay();
        HAL_GPIO_WritePin(mSclGpioPort, mSCLPin, GPIO_PIN_RESET);
        _delay();
    }
    println("delay test %d ms", HAL_GetTick() - startTime);

    startTime = HAL_GetTick();
    for (int i = 0; i < 1000; i++)
    {
        writeReg(0x20, 0x12, 0xFF);
    }

    println("writeReg test %d ms", HAL_GetTick() - startTime);
}