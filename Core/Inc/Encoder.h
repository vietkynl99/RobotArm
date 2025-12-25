#pragma once

#include "main.h"

enum EncoderEvent
{
    ENCODER_EVENT_NONE,
    ENCODER_EVENT_INC,
    ENCODER_EVENT_DEC
};

class Encoder
{
private:
    GPIO_TypeDef *mE1GPIO;
    GPIO_TypeDef *mE2GPIO;
    uint16_t mE1Pin;
    uint16_t mE2Pin;

public:
    Encoder(GPIO_TypeDef *e1GPIO, uint16_t e1Pin, GPIO_TypeDef *e2GPIO, uint16_t e2Pin);

    EncoderEvent onExt(uint16_t extPin);
};