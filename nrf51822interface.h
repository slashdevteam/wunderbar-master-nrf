#pragma once

#include "nrf51822.h"
#include "iblegateway.h"
#include "Thread.h"
#include <unordered_map>
#include <tuple>
#include <map>
#include <list>

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

using Servers = std::unordered_map<DataId, ServerInfo>;

// using ServersToOnboard = std::list<std::string>;
// using ServerInfo       = std::tuple<BleServerCallback, BleServerConfig*, bool>;
// using SearchServers = std::unordered_map<ServerName, std::list<ServerInfo>>;

const ServerName WunderbarSensorNames[] = {
    "WunderbarHTU",
    "WunderbarGYRO",
    "WunderbarLIGHT",
    "WunderbarMIC",
    "WunderbarBRIDG",
    "WunderbarIR"
};

const std::map<ServerName, DataId> ServerNamesToDataId = {
    {WunderbarSensorNames[0], DataId::DEV_HTU},
    {WunderbarSensorNames[1], DataId::DEV_GYRO},
    {WunderbarSensorNames[2], DataId::DEV_LIGHT},
    {WunderbarSensorNames[3], DataId::DEV_SOUND},
    {WunderbarSensorNames[4], DataId::DEV_BRIDGE},
    {WunderbarSensorNames[5], DataId::DEV_IR}
};

class Nrf51822Interface : public IBleGateway
{
public:
    Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq);
    virtual ~Nrf51822Interface();

    // IBleGateway interface
    virtual bool registerServer(BleServerConfig& config, BleServerCallback incomingCallback) override;
    virtual void serverDiscoveryComlpete(BleServerConfig& config) override;
    virtual bool sendToServer(const BleServerConfig& config, BleServerCallback doneCallback) override;
    virtual bool configure() override;
    virtual bool storeConfig() override;
    virtual bool readCharacteristic(const BleServerConfig& server, const CharcteristicDescriptor& characteristic) override;
    virtual bool writeCharacteristic(const BleServerConfig& server,
                                     const CharcteristicDescriptor& characteristic,
                                     const uint8_t* data,
                                     const size_t len) override;

private:
    void spiCallback();
    void doConfig();
    void handleDeviceDiscovery(SpiFrame&);

private:
    Nrf51822 nrfDriver;
    rtos::Thread configurator;
    volatile bool configOk;

    const OnboardSequence reqConfigFields = {FieldId::CONFIG_ONBOARD_DONE, FieldId::SENSOR_STATUS, FieldId::SENSOR_STATUS};
    
    // IBleGateway
    ServerList serversToBeOnboarded;
    ServerList serversOnboarded;

    // Characteristics characteristics;
    Servers         servers;
};
