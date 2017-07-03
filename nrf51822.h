#pragma once

#include "PinNames.h"
#include "Callback.h"
#include "SPI.h"
#include "InterruptIn.h"
#include "Thread.h"
#include "iblegateway.h"

using SpiSlaveReadReqCb = mbed::Callback<void()>;
using mbed::SPI;
using mbed::InterruptIn;
using mbed::InterruptIn;
using rtos::Thread;

const size_t SPI_PACKET_DATA_SIZE = 20;

enum class DataId : uint8_t
{
    DEV_0               = 0x0,
    DEV_1               = 0x1,
    DEV_2               = 0x2,
    DEV_3               = 0x3,
    DEV_4               = 0x4,
    DEV_5               = 0x5,
    DEV_CENTRAL         = 0x7,
    RESPONSE_OK         = 0x64,
    RESPONSE_ERROR      = 0x65,
    RESPONSE_BUSY       = 0x66,
    RESPONSE_NOT_FOUND  = 0x67,
    CONFIG              = 0xC8,
    ERROR               = 0xFF
};

enum class FieldId : uint8_t
{
    CHAR_SENSOR_ID                  = 0,
    CHAR_SENSOR_BEACON_FREQUENCY    = 1,
    CHAR_SENSOR_FREQUENCY           = 2,
    CHAR_SENSOR_LED_STATE           = 3,
    CHAR_SENSOR_THRESHOLD           = 4,
    CHAR_SENSOR_CONFIG              = 5,
    CHAR_SENSOR_DATA_R              = 6,
    CHAR_SENSOR_DATA_W              = 7,
    CHAR_BATTERY_LEVEL              = 8,
    CHAR_MANUFACTURER_NAME          = 9,
    CHAR_HARDWARE_REVISION          = 10,
    CHAR_FIRMWARE_REVISION          = 11,
    SENSOR_STATUS                   = 12,
    CONFIG_HTU_PASS                 = 0,
    CONFIG_GYRO_PASS                = 1,
    CONFIG_LIGHT_PASS               = 2,
    CONFIG_SOUND_PASS               = 3,
    CONFIG_BRIDGE_PASS              = 4,
    CONFIG_IR_PASS                  = 5,
    CONFIG_WIFI_SSID                = 6,
    CONFIG_WIFI_PASS                = 7,
    CONFIG_MASTER_MODULE_ID         = 8,
    CONFIG_MASTER_MODULE_SEC        = 9,
    CONFIG_MASTER_MODULE_URL        = 10,
    CONFIG_START                    = 11,
    CONFIG_COMPLETE                 = 12,
    CONFIG_STOP                     = 13,
    CONFIG_ACK                      = 14,
    CONFIG_ERROR                    = 15,
    RUN                             = 16,
    ADD_DISCOVERY_SERVICE           = 17,
    ADD_SERVER_CHARACTERISTIC       = 18,
    CONFIG_SERVER_NAME              = 19,
    CONFIG_SERVER_PASS              = 20,
    CONFIG_SERVER_UUID              = 21,
    CONTROL_START_DISCOVERY         = 22,
    RUN_WRITE_RSP_OK                = 23,
    CONTROL_DISCOVERY_COMPLETE      = 24,
    CONTROL_CHARACTERISITC_DONE     = 25,
    READ_CHARACTERISTIC             = 26,
    WRITE_CHARACTERISTIC            = 27,
    RUN_ERROR                       = 0xFF
};

enum class Operation : uint8_t
{
    WRITE  = 0,
    READ   = 1
};

struct SpiFrame
{
    DataId      dataId;
    FieldId     fieldId;
    uint8_t     clientIdx;
    Operation   operation;
    uint8_t     data[SPI_PACKET_DATA_SIZE];
} __attribute__((packed));

enum class Modes : uint8_t
{
    CONFIG = static_cast<uint8_t>(FieldId::CONFIG_START),
    DISCOVERY = static_cast<uint8_t>(FieldId::CONTROL_START_DISCOVERY),
    RUN = static_cast<uint8_t>(FieldId::RUN)
};

class Nrf51822
{
public:
    Nrf51822(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq);
    // do HW chip reset
    void reset();
    // config communication settings, sets callback to receive data from nrf when requesting
    void config(SpiSlaveReadReqCb cb);
    // sets/changes callback to receive data from nrf when requesting
    void setRecvReadyCb(SpiSlaveReadReqCb cb);

    void write(const char* txData, size_t len);
    void read(char* rxData, size_t len);

    // ble logic
    bool setMode(Modes newMode);
    bool addDiscoveryService(ServiceDescriptor& discoveryService);
    bool addServerCharacteristic(CharcteristicDescriptor& characteristic);
    bool addServerName(const ServerName& name);
    bool addServerPass(uint8_t client, const PassKey& pass);
    bool addServerUUID(uint8_t client, const ServerUUID& uuid);
    bool writeCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid, const uint8_t* data, size_t len);
    bool readCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid);

private:
    void recvDataIrqCb();
    void recvDataHandler();
    // sending and receiving data

    void readWrite(char* rxData, const char* txData, size_t len);

    // get total number of sent bytes (wraps arround)
    size_t getNBytesSent() const;

private:
    SPI          spiDriver;
    PinName      ssel;
    InterruptIn  recvDataIrq;
    Thread       recvDataThread;

    SpiSlaveReadReqCb recvDataExtCb;


};

