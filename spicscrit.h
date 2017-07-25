#pragma once

#include "DigitalOut.h"
#include "mbed_wait_api.h"

using mbed::DigitalOut;

class SpiCsCrit
{
public:
    SpiCsCrit(PinName _sselPin)
        : ssel(_sselPin)
    {
        ssel = LOW;
        delay();
    }

    ~SpiCsCrit()
    {
        delay();
        ssel = HIGH;
    }

private:
    void delay()
    {
        wait_us(10);
    }


private:
    DigitalOut  ssel;
    const uint32_t LOW  = 0;
    const uint32_t HIGH = 1;
};
