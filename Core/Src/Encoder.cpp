#include "Encoder.h"

Encoder::Encoder(GPIO_TypeDef *e1GPIO, uint16_t e1Pin, GPIO_TypeDef *e2GPIO, uint16_t e2Pin)
    : mE1GPIO(e1GPIO), mE1Pin(e1Pin), mE2GPIO(e2GPIO), mE2Pin(e2Pin)
{
}

EncoderEvent Encoder::onExt(uint16_t extPin)
{
    if (extPin == mE1Pin)
    {
        if (HAL_GPIO_ReadPin(mE2GPIO, mE2Pin))
        {
            return ENCODER_EVENT_INC;
        }
        else
        {
            return ENCODER_EVENT_DEC;
        }
    }
    else
    {
        return ENCODER_EVENT_NONE;
    }
}