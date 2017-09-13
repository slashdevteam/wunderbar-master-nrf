#pragma once

#include "nrf51822.h"
#include "iblegateway.h"
#include "Thread.h"
#include <unordered_map>
#include <tuple>
#include <list>
#include "wunderbarble.h"

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
    void idleModeCb();
    void onboardModeCb();
    void runModeCb();
    void goToRunMode();
    void onboardSensors();
    void handleOnboarding(SpiFrame&);

private:
    Nrf51822 nrfDriver;
    SpiFrame readSpiFrame();
    rtos::Thread onboardMode;
    rtos::Thread runMode;
    volatile bool configOk;

    // fields to be received from sensor being onboarded
    const OnboardSequence reqConfigFields = {FieldId::ONBOARD_DONE, FieldId::SENSOR_STATUS};

    ServerList serverList;
    ServerList serversOnboarded;
    Servers    servers;

    BleEvent fieldId2BleEvent(FieldId fId, Operation op);
    IStdInOut* log;
};
