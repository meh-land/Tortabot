
#include "ISR_Encoder.h"

ISR_Encoder::ISR_Encoder(float resolution, uint8_t pinA, uint8_t pinB) : Encoder(resolution)
{
    PinA = pinA;
    PinB = pinB;
}

void ISR_Encoder::setup()
{
    pinMode(PinA, INPUT_PULLUP);
    pinMode(PinB, INPUT_PULLUP);
}

void ISR_Encoder::update_ISR_B(void)
{
    if (digitalRead(PinA) != digitalRead(PinB))
    {
        counts++;
    }
    else
    {
        counts--;
    }
}
void ISR_Encoder::update_ISR_A(void)
{
    if (digitalRead(PinA) == digitalRead(PinB))
    {
        counts++;
    }
    else
    {
        counts--;
    }
}
