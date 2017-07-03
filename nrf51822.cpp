
#include "nrf51822.h"
#include "spicscrit.h"
#include "wunderbarble.h"
#include "mbed_wait_api.h"
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
    if(NC == ssel)
    {
        DigitalOut SSEL(NRF_SSEL, HIGH);
    }
}

void Nrf51822::reset()
{
    // take control of reset pin
    DigitalOut resetPin(NRF_NRESET);
    // pull reset pin down
    resetPin = LOW;

    // hold it down
    wait_us(NRF_RESET_TIME_US);

    // pull it up
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
    {
        SpiCsCrit scrit(ssel);
        // DigitalOut SSEL(NRF_SSEL, LOW);

        for (auto byte = 0U; byte < len; ++byte)
        {
            spiDriver.write(txData[byte]);
        }
        // SSEL = HIGH;
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
    const size_t spi0TcrReg = *reinterpret_cast<const volatile size_t*>(0x4002C008);
    return (spi0TcrReg >> 16);
}

bool Nrf51822::setMode(Modes newMode)
{
    SpiFrame frame = {DataId::CONFIG, static_cast<FieldId>(newMode), 0, Operation::WRITE};
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::addDiscoveryService(ServiceDescriptor& discoveryService)
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::ADD_DISCOVERY_SERVICE,
                      0,
                      Operation::WRITE};
    std::memcpy(frame.data, discoveryService.data(), sizeof(ServiceDescriptor));
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::addServerCharacteristic(CharcteristicDescriptor& characteristic)
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::ADD_SERVER_CHARACTERISTIC,
                      0,
                      Operation::WRITE};
    std::memcpy(frame.data, characteristic.data(), sizeof(CharcteristicDescriptor));
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::addServerName(const ServerName& name)
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::CONFIG_SERVER_NAME,
                      0,
                      Operation::WRITE};
    std::memcpy(frame.data, name.c_str(), wunderbar::limits::SERVER_NAME_MAX_LEN);
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::addServerPass(uint8_t client, const PassKey& pass)
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::CONFIG_SERVER_PASS,
                      client,
                      Operation::WRITE};
    std::memcpy(frame.data, pass.data(), wunderbar::limits::SERVER_PASS_MAX_LEN);
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::addServerUUID(uint8_t client, const ServerUUID& uuid)
{
    SpiFrame frame = {DataId::CONFIG,
                      FieldId::CONFIG_SERVER_UUID,
                      client,
                      Operation::WRITE};
    std::memcpy(frame.data, uuid.data(), wunderbar::limits::SERVER_UUID_MAX_LEN);
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::readCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid)
{
    SpiFrame frame = {static_cast<DataId>(client),
                      FieldId::READ_CHARACTERISTIC,
                      0,
                      Operation::READ};
    std::memcpy(frame.data, uuid.data(), sizeof(uuid.uuid));
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

bool Nrf51822::writeCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid, const uint8_t* data, size_t len)
{
    SpiFrame frame = {static_cast<DataId>(client),
                      FieldId::WRITE_CHARACTERISTIC,
                      0,
                      Operation::WRITE};
    std::memcpy(frame.data, uuid.data(), sizeof(uuid.uuid));
    write(reinterpret_cast<char*>(&frame), sizeof(frame));
    return true;
}

