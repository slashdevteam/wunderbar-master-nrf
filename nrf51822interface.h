#pragma once

#include "nrf51822.h"
#include "iblegateway.h"
#include "Thread.h"
#include <unordered_map>
#include <tuple>

using ServerInfo = std::tuple<BleServerCallback, BleServerConfig*, bool>;
using Servers = std::unordered_map<uint8_t, ServerInfo>;
using SearchNames = std::list<std::string>;
using SearchServers = std::unordered_map<ServerName, std::list<ServerInfo>>;

class Nrf51822Interface : public IBleGateway
{
public:
    Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq);
    virtual ~Nrf51822Interface();

    // // IBleGateway interface
    // virtual bool registerServer(BleServerConfig& config, BleServerCallback incomingCallback) override;
    // virtual void serverDiscoveryComlpete(BleServerConfig& config) override;
    // virtual bool sendToServer(const BleServerConfig& config, BleServerCallback doneCallback) override;
    // virtual bool configure() override;
    // virtual bool storeConfig() override;
    // virtual bool readCharacteristic(const BleServerConfig& server, const CharcteristicDescriptor& characteristic) override;
    // virtual bool writeCharacteristic(const BleServerConfig& server,
    //                                  const CharcteristicDescriptor& characteristic,
    //                                  const uint8_t* data,
    //                                  const size_t len) override;
private:
    void spiCallback();
    void doConfig();
    void handleDeviceDiscovery(uint8_t client, const uint8_t* data);

private:
    Nrf51822 spi;
    rtos::Thread configurator;
    volatile bool configOk;
    // IBleGateway
    RequiredServices services;
    SearchNames searchNames;
    Characteristics characteristics;
    SearchServers searchServers;
    Servers servers;
};
