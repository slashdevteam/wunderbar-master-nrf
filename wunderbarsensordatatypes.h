#pragma once

#include <cstdint>
#include <cstdio>
#include "mbed.h"
#include <string>
#include "nrf51822types.h"

const size_t MAX_SENSOR_PAYLOAD_LEN = 20;
constexpr size_t WUNDERBAR_SENSORS_NUM = 6;

using ServerName = std::string;
const ServerName& WunderbarSensorNames(uint8_t sensorId);
DataId ServerNamesToDataId(const ServerName& name);

struct threshold_int8_t
{
    uint8_t sbl;
    int8_t  low;
    int8_t  high;
} __attribute__((packed));

struct threshold_int16_t
{
    uint16_t sbl;
    int16_t  low;
    int16_t  high;
} __attribute__((packed));

struct threshold_int32_t
{
    uint32_t sbl;
    int32_t  low;
    int32_t  high;
} __attribute__((packed));


struct threshold_float_t
{
    float sbl;
    float low;
    float high;
} __attribute__((packed));

