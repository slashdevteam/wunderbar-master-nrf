#pragma once

#include "PinNames.h"
#include "Callback.h"
#include "SPI.h"
#include "InterruptIn.h"
#include "Thread.h"
#include "nrf51822types.h"

using SpiSlaveReadReqCb = mbed::Callback<void()>;
using mbed::SPI;
using mbed::InterruptIn;
using rtos::Thread;

class Nrf51822
{
public:
    Nrf51822(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq);
    // do HW chip reset
    void reset();
    void on();
    void off();
    // do SW reset via kill signal over SPI
    void softwareReset();
    // config communication settings
    void config();
    // sets/changes callback to receive data from nrf when requesting
    void setRecvReadyCb(SpiSlaveReadReqCb cb);

    // sending and receiving data
    void read(char* rxData, size_t len);
    void write(const char* txData, size_t len);
    void readWrite(char* rxData, const char* txData, size_t len);

    // ble logic
    void setMode(Modes newMode);
    void configServerPass(DataId client, uint8_t* data);
    void requestPasskeyStoring();
    void requestCharacteristicRead(const DataId client, FieldId bleChar);
    void requestCharacteristicWrite(DataId client, FieldId bleChar, const uint8_t* data, size_t len);

private:
    // callback handling
    void recvDataIrqCb();
    void recvDataHandler();

    // get total number of sent bytes (wraps arround)
    size_t getNBytesSent() const;

private:
    SPI          spiDriver;
    PinName      ssel;
    InterruptIn  recvDataIrq;
    Thread       recvDataThread;

    SpiSlaveReadReqCb recvDataExtCb;
};

