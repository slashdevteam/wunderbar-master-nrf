
#include "nrf51822.h"
#include "spicscrit.h"
#include "wunderbarble.h"
#include "mbed_wait_api.h"
#include "DigitalInOut.h"
#include "Callback.h"

const uint32_t LOW  = 0;
const uint32_t HIGH = 1;
const uint32_t SIGNAL_IRQ = 0x1;
const uint32_t NRF_RESET_TIME_US = 100;
const uint32_t DUMMY_BYTE = 0xFF;

const auto SPI_BITS_PER_FRAME = 8;
// Polarity + phase
const auto SPI_MODE = 1;

Nrf51822::Nrf51822(PinName _mosi,
                   PinName _miso,
                   PinName _sclk,
                   PinName _ssel,
                   PinName _extIrq)
    : spiDriver(_mosi, _miso, _sclk, _ssel),
      ssel(_ssel),
      recvDataIrq(_extIrq),
      recvDataThread()
{
}

void Nrf51822::reset()
{
    // take control of reset pin and put it up
    mbed::DigitalInOut resetPin(NRF_NRESET, PinDirection::PIN_OUTPUT, PinMode::PullUp, HIGH);
    
    // hold it to stabilise the level
    wait_us(NRF_RESET_TIME_US*2);

    // pull reset pin down
    resetPin = LOW;

    // hold it down
    wait_us(NRF_RESET_TIME_US*2);

    // pull it up again
    resetPin = HIGH;
}

void Nrf51822::config(SpiSlaveReadReqCb cb)
{
    spiDriver.frequency(2e6);
    spiDriver.format(SPI_BITS_PER_FRAME, SPI_MODE);

    recvDataExtCb = cb;
    recvDataIrq.mode(PinMode::PullDown);
    recvDataIrq.rise(mbed::callback(this, &Nrf51822::recvDataIrqCb));

    recvDataThread.start(mbed::callback(this, &Nrf51822::recvDataHandler));
}

void Nrf51822::setRecvReadyCb(SpiSlaveReadReqCb cb)
{
    recvDataExtCb = cb;
}

void Nrf51822::recvDataIrqCb()
{
    recvDataThread.signal_set(SIGNAL_IRQ);
}

void Nrf51822::recvDataHandler()
{
    while (true)
    {
        Thread::signal_wait(SIGNAL_IRQ);
        if (recvDataExtCb)
        {
            recvDataExtCb();
        }
    }
}

void Nrf51822::write(const char* txData, size_t len)
{
    SpiCsCrit scrit(ssel);

    for (auto byte = 0U; byte < len; ++byte)
    {
        spiDriver.write(txData[byte]);
    }
}

void Nrf51822::read(char* rxData, size_t len)
{
    SpiCsCrit scrit(ssel);

    for (auto byte = 0U; byte < len; ++byte)
    {
        rxData[byte] = spiDriver.write(DUMMY_BYTE);
    }
}

void Nrf51822::readWrite(char* rxData, const char* txData, size_t len)
{
    SpiCsCrit scrit(ssel);

    for (auto byte = 0U; byte < len; ++byte)
    {
        rxData[byte] = spiDriver.write(txData[byte]);
    }
}

size_t Nrf51822::getNBytesSent() const
{
    // 16 MSBs of SPI0 TCR register
    const size_t spi0TcrReg = *reinterpret_cast<const volatile size_t*>(0x4002C008);
    return (spi0TcrReg >> 16);
}

void Nrf51822::setMode(Modes newMode)
{
    SpiFrame frame = {DataId::CONFIG,
                      static_cast<FieldId>(newMode),
                      Operation::NOT_USED};

    write(reinterpret_cast<char*>(&frame), sizeof(frame));
}

void Nrf51822::resetNrfSoftware()
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::KILL, 
                      Operation::NOT_USED};

    write(reinterpret_cast<char*>(&frame), sizeof(frame));
}

void Nrf51822::configServerPass(DataId client, uint8_t* data)
{
    SpiFrame frame = {DataId::CONFIG,
                      static_cast<FieldId>(client),
                      Operation::NOT_USED};

    std::memcpy(frame.data, data, wunderbar::limits::SERVER_PASS_MAX_LEN);
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
}

void Nrf51822::requestPasskeyStoring()
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::CONFIG_STORE_PASSKEYS,
                      Operation::NOT_USED};

    write(reinterpret_cast<char*>(&frame), sizeof(frame));
}

void Nrf51822::readCharacteristic(const DataId client, FieldId bleChar, uint8_t* data)
{
    SpiFrame frame = {static_cast<DataId>(client),
                      bleChar,
                      Operation::READ};

    read(reinterpret_cast<char*>(&frame), sizeof(frame));
    std::memcpy(data, frame.data,  sizeof(frame));
}

void Nrf51822::writeCharacteristic(DataId client, FieldId bleChar, const uint8_t* data, size_t len)
{
    SpiFrame frame = {static_cast<DataId>(client),
                      bleChar,
                      Operation::WRITE};
                      
    std::memcpy(frame.data, data, len);
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
}

