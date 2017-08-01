#include "wunderbarsensordatatypes.h"
#include <unordered_map>
#include "wunderbarble.h"

const ServerName& WunderbarSensorNames(uint8_t sensorId)
{
    static const ServerName sensorNameHtu        = "WunderbarHTU"s;
    static const ServerName sensorNameGyro       = "WunderbarGYRO";
    static const ServerName sensorNameLightProx  = "WunderbarLIGHT";
    static const ServerName sensorNameMicrophone = "WunderbarMIC";
    static const ServerName sensorNameBridge     = "WunderbarBRIDG";
    static const ServerName sensorNameInfraRed   = "WunderbarIR";

    static const ServerName names[] = {
        sensorNameHtu,
        sensorNameGyro,
        sensorNameLightProx,
        sensorNameMicrophone,
        sensorNameBridge,
        sensorNameInfraRed
    };

    return names[sensorId];
}

DataId ServerNamesToDataId(const ServerName& name)
{
    static const std::unordered_map<ServerName, DataId> name2Id = {
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_HTU), DataId::DEV_HTU},
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_GYRO), DataId::DEV_GYRO},
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_LIGHT), DataId::DEV_LIGHT},
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_SOUND), DataId::DEV_SOUND},
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_BRIDGE), DataId::DEV_BRIDGE},
    {WunderbarSensorNames(wunderbar::sensors::DATA_ID_DEV_IR), DataId::DEV_IR}
    };
    return name2Id.at(name);
}
