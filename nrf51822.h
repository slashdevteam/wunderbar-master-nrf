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
    DEV_HTU             = 0x0,
    DEV_GYRO            = 0x1,
    DEV_LIGHT           = 0x2,
    DEV_SOUND           = 0x3,
    DEV_BRIDGE          = 0x4,
    DEV_IR              = 0x5,
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
    CHAR_SENSOR_ID                  = 0x0,
    CHAR_SENSOR_BEACON_FREQUENCY    = 0x1,
    CHAR_SENSOR_FREQUENCY           = 0x2,
    CHAR_SENSOR_LED_STATE           = 0x3,
    CHAR_SENSOR_THRESHOLD           = 0x4,
    CHAR_SENSOR_CONFIG              = 0x5,
    CHAR_SENSOR_DATA_R              = 0x6,
    CHAR_SENSOR_DATA_W              = 0x7,
    CHAR_BATTERY_LEVEL              = 0x8,
    CHAR_MANUFACTURER_NAME          = 0x9,
    CHAR_HARDWARE_REVISION          = 0xA,
    CHAR_FIRMWARE_REVISION          = 0xB,
    SENSOR_STATUS                   = 0xC,

    CONFIG_HTU_PASS                 = 0x0,
    CONFIG_GYRO_PASS                = 0x1,
    CONFIG_LIGHT_PASS               = 0x2,
    CONFIG_SOUND_PASS               = 0x3,
    CONFIG_BRIDGE_PASS              = 0x4,
    CONFIG_IR_PASS                  = 0x5,
    CONFIG_WIFI_SSID                = 0x6,
    CONFIG_WIFI_PASS                = 0x7,
    CONFIG_MASTER_MODULE_ID         = 0x8,
    CONFIG_MASTER_MODULE_SEC        = 0x9,
    CONFIG_MASTER_MODULE_URL        = 0xA,
    CONFIG_START                    = 0xB,
    CONFIG_COMPLETE                 = 0xC,
    CONFIG_STOP                     = 0xD,
    CONFIG_ACK                      = 0xE,
    CONFIG_ERROR                    = 0xF,
    RUN                             = 0x10,
    CONFIG_ONBOARD_DONE             = 0x11,
    KILL                            = 0x12,
    // ADD_DISCOVERY_SERVICE           = 17,
    // ADD_SERVER_CHARACTERISTIC       = 18,
    // CONFIG_SERVER_NAME              = 19,
    // CONFIG_SERVER_PASS              = 20,
    // CONFIG_SERVER_UUID              = 21,
    // CONTROL_START_DISCOVERY         = 22,
    // RUN_WRITE_RSP_OK                = 23,
    // CONTROL_DISCOVERY_COMPLETE      = 24,
    // CONTROL_CHARACTERISITC_DONE     = 25,
    // READ_CHARACTERISTIC             = 26,
    // WRITE_CHARACTERISTIC            = 27,


    // CONFIG_DONE_MASK                = 0x20,
    // CONFIG_HTU_DONE                 = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_HTU),
    // CONFIG_GYRO_DONE                = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_GYRO),
    // CONFIG_LIGHT_DONE               = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_LIGHT),
    // CONFIG_SOUND_DONE               = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_SOUND),
    // CONFIG_BRIDGE_DONE              = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_BRIDGE),
    // CONFIG_IR_DONE                  = CONFIG_DONE_MASK | static_cast<uint8_t>(DataId::DEV_IR),

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
    // uint8_t     clientIdx;
    Operation   operation;
    uint8_t     data[SPI_PACKET_DATA_SIZE];
} __attribute__((packed));

enum class Modes : uint8_t
{
    CONFIG = static_cast<uint8_t>(FieldId::CONFIG_START),
    // DISCOVERY = static_cast<uint8_t>(FieldId::CONTROL_START_DISCOVERY),
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
    // bool setMode(Modes newMode);
    // bool addDiscoveryService(ServiceDescriptor& discoveryService);
    // bool addServerCharacteristic(CharcteristicDescriptor& characteristic);
    // bool addServerName(const ServerName& name);
    // bool addServerPass(uint8_t client, const PassKey& pass);
    // bool addServerUUID(uint8_t client, const ServerUUID& uuid);
    // bool writeCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid, const uint8_t* data, size_t len);
    // bool readCharacteristic(uint8_t client, const CharcteristicDescriptor& uuid);

    void readWrite(char* rxData, const char* txData, size_t len);
private:
    void recvDataIrqCb();
    void recvDataHandler();
    // sending and receiving data


    // get total number of sent bytes (wraps arround)
    size_t getNBytesSent() const;

private:
    SPI          spiDriver;
    PinName      ssel;
    InterruptIn  recvDataIrq;
    Thread       recvDataThread;

    SpiSlaveReadReqCb recvDataExtCb;


};

