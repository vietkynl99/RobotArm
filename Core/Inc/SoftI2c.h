#pragma once

#include "main.h"
#include "Log.h"

class SoftI2c
{
private:
    GPIO_TypeDef *mSdaGpioPort;
    GPIO_TypeDef *mSclGpioPort;
    uint16_t mSDAPin;
    uint16_t mSCLPin;

public:
    SoftI2c(GPIO_TypeDef *sdaGpioPort, uint16_t sdaPin, GPIO_TypeDef *sclGpioPort, uint16_t sclPin);

    bool begin();
    void start();
    void stop();

    bool writeByte(uint8_t data);
    uint8_t readByte(bool ack);

    bool writeReg(uint8_t addr, uint8_t reg, uint8_t val);
    bool readReg(uint8_t addr, uint8_t reg, uint8_t &out);

    bool probe(uint8_t address);
    bool verifyBus();

    void test();

private:
    void _delay();
};