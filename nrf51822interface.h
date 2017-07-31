#pragma once

#include "nrf51822.h"
#include "iblegateway.h"
#include "Thread.h"
#include <unordered_map>
#include <tuple>
#include <list>
#include "wunderbarble.h"
#include "wunderbarsensordatatypes.h"

const std::unordered_map<FieldId, uint16_t> FieldIdToCharUuid = {
    {FieldId::CHAR_SENSOR_ID              , wunderbar::characteristics::sensor::ID},
    {FieldId::CHAR_SENSOR_BEACON_FREQUENCY, wunderbar::characteristics::sensor::BEACON_FREQ},
    {FieldId::CHAR_SENSOR_FREQUENCY       , wunderbar::characteristics::sensor::FREQUENCY},
    {FieldId::CHAR_SENSOR_LED_STATE       , wunderbar::characteristics::sensor::LED_STATE},
    {FieldId::CHAR_SENSOR_THRESHOLD       , wunderbar::characteristics::sensor::THRESHOLD},
    {FieldId::CHAR_SENSOR_CONFIG          , wunderbar::characteristics::sensor::CONFIG},
    {FieldId::CHAR_SENSOR_DATA_R          , wunderbar::characteristics::sensor::DATA_R},
    {FieldId::CHAR_SENSOR_DATA_W          , wunderbar::characteristics::sensor::DATA_W},
    
    {FieldId::CHAR_BATTERY_LEVEL          , wunderbar::characteristics::ble::BATTERY_LEVEL},
    {FieldId::CHAR_MANUFACTURER_NAME      , wunderbar::characteristics::ble::MANUFACTURER_NAME},
    {FieldId::CHAR_HARDWARE_REVISION      , wunderbar::characteristics::ble::HARDWARE_REVISION},
    {FieldId::CHAR_FIRMWARE_REVISION      , wunderbar::characteristics::ble::FIRMWARE_REVISION}
};

const std::unordered_map<uint16_t, FieldId> CharUuidToFieldId = {
    {wunderbar::characteristics::sensor::ID,             FieldId::CHAR_SENSOR_ID              },
    {wunderbar::characteristics::sensor::BEACON_FREQ,    FieldId::CHAR_SENSOR_BEACON_FREQUENCY},
    {wunderbar::characteristics::sensor::FREQUENCY,      FieldId::CHAR_SENSOR_FREQUENCY       },
    {wunderbar::characteristics::sensor::LED_STATE,      FieldId::CHAR_SENSOR_LED_STATE       },
    {wunderbar::characteristics::sensor::THRESHOLD,      FieldId::CHAR_SENSOR_THRESHOLD       },
    {wunderbar::characteristics::sensor::CONFIG,         FieldId::CHAR_SENSOR_CONFIG          },
    {wunderbar::characteristics::sensor::DATA_R,         FieldId::CHAR_SENSOR_DATA_R          },
    {wunderbar::characteristics::sensor::DATA_W,         FieldId::CHAR_SENSOR_DATA_W          },

    {wunderbar::characteristics::ble::BATTERY_LEVEL,     FieldId::CHAR_BATTERY_LEVEL          },
    {wunderbar::characteristics::ble::MANUFACTURER_NAME, FieldId::CHAR_MANUFACTURER_NAME      },
    {wunderbar::characteristics::ble::HARDWARE_REVISION, FieldId::CHAR_HARDWARE_REVISION      },
    {wunderbar::characteristics::ble::FIRMWARE_REVISION, FieldId::CHAR_FIRMWARE_REVISION      }
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
    virtual bool requestRead(const BleServerConfig& server, uint16_t bleCharUuid) override;
    virtual bool requestWrite(const BleServerConfig& server,
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
