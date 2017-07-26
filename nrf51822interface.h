#pragma once

#include "nrf51822.h"
#include "iblegateway.h"
#include "Thread.h"
#include <unordered_map>
#include <tuple>
#include <list>
#include "wunderbarble.h"

const std::unordered_map<FieldId, uint16_t> FieldIdToCharUuid = {
    {CHAR_SENSOR_ID              , characteristics::sensor::ID},
    {CHAR_SENSOR_BEACON_FREQUENCY, characteristics::sensor::BEACON_FREQ},
    {CHAR_SENSOR_FREQUENCY       , characteristics::sensor::FREQUENCY},
    {CHAR_SENSOR_LED_STATE       , characteristics::sensor::LED_STATE},
    {CHAR_SENSOR_THRESHOLD       , characteristics::sensor::THRESHOLD},
    {CHAR_SENSOR_CONFIG          , characteristics::sensor::CONFIG},
    {CHAR_SENSOR_DATA_R          , characteristics::sensor::DATA_R},
    {CHAR_SENSOR_DATA_W          , characteristics::sensor::DATA_W},
    {CHAR_BATTERY_LEVEL          , characteristics::ble::BATTERY_LEVEL},
    {CHAR_MANUFACTURER_NAME      , characteristics::ble::MANUFACTURER_NAME_STRING},
    {CHAR_HARDWARE_REVISION      , characteristics::ble::HARDWARE_REVISION_STRING},
    {CHAR_FIRMWARE_REVISION      , characteristics::ble::FIRMWARE_REVISION_STRING}
};

const std::unordered_map<uint16_t, FieldId> CharUuidToFieldId = {
    {characteristics::sensor::ID,                    CHAR_SENSOR_ID              },
    {characteristics::sensor::BEACON_FREQ,           CHAR_SENSOR_BEACON_FREQUENCY},
    {characteristics::sensor::FREQUENCY,             CHAR_SENSOR_FREQUENCY       },
    {characteristics::sensor::LED_STATE,             CHAR_SENSOR_LED_STATE       },
    {characteristics::sensor::THRESHOLD,             CHAR_SENSOR_THRESHOLD       },
    {characteristics::sensor::CONFIG,                CHAR_SENSOR_CONFIG          },
    {characteristics::sensor::DATA_R,                CHAR_SENSOR_DATA_R          },
    {characteristics::sensor::DATA_W,                CHAR_SENSOR_DATA_W          },
    {characteristics::ble::BATTERY_LEVEL,            CHAR_BATTERY_LEVEL          },
    {characteristics::ble::MANUFACTURER_NAME_STRING, CHAR_MANUFACTURER_NAME      },
    {characteristics::ble::HARDWARE_REVISION_STRING, CHAR_HARDWARE_REVISION      },
    {characteristics::ble::FIRMWARE_REVISION_STRIN,  CHAR_FIRMWARE_REVISION      }
};


class IStdInOut;

using OnboardSequence  = std::list<FieldId>;
using ServerList       = std::list<DataId>;

struct OnboardInfo {
    bool            onboarded;
    OnboardSequence onboardSeq;

    OnboardInfo() 
        : onboarded(false),
          onboardSeq()
          {}

};

struct ServerInfo {
    BleServerConfig*  serverConfig;
    BleServerCallback bleServerCb;
    OnboardInfo       onboardInfo;

    ServerInfo() = default;

    ServerInfo(BleServerConfig& _serverConfig, BleServerCallback _bleServerCb)
        : serverConfig(&_serverConfig),
          bleServerCb(_bleServerCb),
          onboardInfo()
          {}

};

const ServerName WunderbarSensorNames[] = {
    "WunderbarHTU",
    "WunderbarGYRO",
    "WunderbarLIGHT",
    "WunderbarMIC",
    "WunderbarBRIDG",
    "WunderbarIR"
};

const std::unordered_map<ServerName, DataId> ServerNamesToDataId = {
    {WunderbarSensorNames[0], DataId::DEV_HTU},
    {WunderbarSensorNames[1], DataId::DEV_GYRO},
    {WunderbarSensorNames[2], DataId::DEV_LIGHT},
    {WunderbarSensorNames[3], DataId::DEV_SOUND},
    {WunderbarSensorNames[4], DataId::DEV_BRIDGE},
    {WunderbarSensorNames[5], DataId::DEV_IR}
};

using Servers = std::unordered_map<DataId, ServerInfo>;

class Nrf51822Interface : public IBleGateway
{
public:
    Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq, IStdInOut* _log);
    virtual ~Nrf51822Interface();

    // IBleGateway interface
    virtual bool registerServer(BleServerConfig& config, BleServerCallback serverCallback) override;
    virtual void serverDiscoveryComlpete(BleServerConfig& config) override;
    virtual bool sendToServer(const BleServerConfig& config, BleServerCallback doneCallback) override;
    virtual bool configure() override;
    virtual void startOperation() override;
    virtual bool storeConfig() override;
    virtual bool requestCharacteristicRead(const BleServerConfig& server, uint16_t bleCharUuid) override;
    virtual bool requestCharacteristicWrite(const BleServerConfig& server,
                                            uint16_t bleCharUuid,
                                            const uint8_t* data,
                                            const size_t len) override;
private:
    void spiCallback();
    void onboardSensors();
    void handleOnboarding(SpiFrame&);

private:
    Nrf51822 nrfDriver;
    rtos::Thread configurator;
    volatile bool configOk;

    // fields to be received from sensor being onboarded
    const OnboardSequence reqConfigFields = {FieldId::ONBOARD_DONE, FieldId::SENSOR_STATUS};
    
    ServerList serverList;
    ServerList serversOnboarded;
    Servers    servers;

    BleEvent fieldId2BleEvent(FieldId fId, Operation op);
    IStdInOut* log;
};
