#pragma once

#include <cstdint>

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

    CONFIG_HTU_PASS                 = static_cast<uint8_t>(DataId::DEV_HTU),
    CONFIG_GYRO_PASS                = static_cast<uint8_t>(DataId::DEV_GYRO),
    CONFIG_LIGHT_PASS               = static_cast<uint8_t>(DataId::DEV_LIGHT),
    CONFIG_SOUND_PASS               = static_cast<uint8_t>(DataId::DEV_SOUND),
    CONFIG_BRIDGE_PASS              = static_cast<uint8_t>(DataId::DEV_BRIDGE),
    CONFIG_IR_PASS                  = static_cast<uint8_t>(DataId::DEV_IR),

    CONFIG_START                    = 0x10,
    CONFIG_COMPLETE                 = 0x11,
    CONFIG_STOP                     = 0x12,
    CONFIG_ACK                      = 0x13,
    CONFIG_ERROR                    = 0x14,
    CONFIG_STORE_PASSKEYS           = 0x15,

    RUN                             = 0x20,
    ONBOARD_DONE                    = 0x21,
    KILL                            = 0x22,
    SENSOR_WRITE_OK                 = 0x23,
    
    INVALID                         = 0xFF
};

enum class Operation : uint8_t
{
    WRITE = 0x0,
    READ  = 0x1,

    // It is used in this manner for FIELD_ID_SENSOR_STATUS, so let's have it explicitly
    CONNECTION_OPENED = 0x0,
    CONNECTION_CLOSED = 0x1,

    NOT_USED = 0xFF
};

struct SpiFrame
{
    DataId      dataId;
    FieldId     fieldId;
    Operation   operation;
    uint8_t     data[SPI_PACKET_DATA_SIZE];
} __attribute__((packed));

enum class Modes : uint8_t
{
    CONFIG = static_cast<uint8_t>(FieldId::CONFIG_START),
    RUN    = static_cast<uint8_t>(FieldId::RUN)
};
