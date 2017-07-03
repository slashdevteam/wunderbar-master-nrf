#pragma once

#include "PinNames.h"
#include "DigitalOut.h"
#include "mbed_wait_api.h"

using mbed::DigitalOut;

class SpiCsCrit
{
public:
    SpiCsCrit(PinName _sselPin)
        : pin(_sselPin),
          ssel((pin == NC) ? NRF_SSEL : pin)
    {
        if(NC == pin)
        {
            ssel = LOW;
            delay();
        }
    }

    ~SpiCsCrit()
    {
        if(NC == pin)
        {
            delay();
            ssel = HIGH;
        }
    }

private:
    void delay()
    {
        wait_us(10);
    }


private:
    PinName     pin;
    DigitalOut  ssel;
    const uint32_t LOW  = 0;
    const uint32_t HIGH = 1;
};
