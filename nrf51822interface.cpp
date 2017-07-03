#include "nrf51822interface.h"

#include "wunderbarble.h"
#include  <algorithm>

const int32_t FW_VERSION_READ = 0x8;
const int32_t CONFIG_MSG_ACK = 0x4;
const int32_t SERVER_DISCOVERY_COMPLETE = 0x2;

Nrf51822Interface::Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq)
    : spi(_mosi, _miso, _sclk, _ssel, _extIrq),
      configurator(osPriorityNormal, 0x400),
      configOk(false)
{
    // spi.reset();
    spi.config(mbed::callback(this, &Nrf51822Interface::spiCallback));
}

Nrf51822Interface::~Nrf51822Interface()
{

}

bool Nrf51822Interface::registerServer(BleServerConfig& config, BleServerCallback incomingCallback)
{
    bool success = false;

    if(servers.size() < wunderbar::limits::MAX_SERVERS)
    {
        // fill in required services for new server
        for(auto& requiredService : config.services)
        {
            auto existingService = std::find(services.begin(), services.end(), requiredService);
            if(existingService == services.end())
            {
                if(services.size() < wunderbar::limits::MAX_DISCOVERY_SERVICES)
                {
                    services.emplace_back(requiredService);
                }
                else
                {
                    return false;
                }
            }
        }

        // add server search name if not already on list
        auto existingName = std::find(searchNames.begin(), searchNames.end(), config.name);
        if(existingName == searchNames.end())
        {
            if(searchNames.size() < wunderbar::limits::MAX_SERVERS)
            {
                searchNames.emplace_back(config.name);
            }
            else
            {
                return false;
            }
        }

        auto existingServer = searchServers.find(config.name);
        if(existingServer == searchServers.end())
        {
            if(searchServers.size() < wunderbar::limits::MAX_SERVERS)
            {
                searchServers.emplace(config.name, std::list<ServerInfo>());
            }
            else
            {
                return false;
            }
        }

        searchServers[config.name].emplace_back(std::make_tuple(incomingCallback, &config, false));
        // fill in characteristics for new server
        // for(auto& characteristic : config.characteristics)
        // {
        //     auto existingCharacteristic = std::find(characteristics.begin(), characteristics.end(), characteristic);
        //     if(existingCharacteristic == characteristics.end())
        //     {
        //         if(characteristics.size() <= wunderbar::limits::MAX_NUMBER_OF_CHARACTERISTICS)
        //         {
        //             characteristics.emplace_back(characteristic);
        //         }
        //         else
        //         {
        //             return false;
        //         }
        //     }
        // }

        // assign next free id
        // config.id = servers.size();
        // servers.emplace(config.id, std::make_tuple(incomingCallback, &config));
        success = true;
    }

    return success;
}

bool Nrf51822Interface::sendToServer(const BleServerConfig& config, BleServerCallback doneCallback)
{
    return false;
}

bool Nrf51822Interface::configure()
{
    configurator.start(mbed::callback(this, &Nrf51822Interface::doConfig));
    configurator.join();
    return configOk;
}

void Nrf51822Interface::doConfig()
{
    // dummy read for initial FW version packet from NRF
    spiCallback();
    rtos::Thread::signal_wait(FW_VERSION_READ);

    spi.setMode(Modes::CONFIG);
    // rtos::Thread::signal_wait(CONFIG_MSG_ACK);

    for(auto& requiredService : services)
    {
        if(requiredService.descriptor.useMode & UseMode::ONBOARD)
        {
            spi.addDiscoveryService(requiredService.descriptor);
            // rtos::Thread::signal_wait(CONFIG_MSG_ACK);
        }
    }

    size_t serversToDiscover = searchNames.size();
    for(auto& searchName : searchNames)
    {
        spi.addServerName(searchName);
    }
    // for(auto& characteristic : characteristics)
    // {
    //     spi.addServerCharacteristic(characteristic);
    // }

    // for(auto& server : servers)
    // {
    //     auto& config = std::get<1>(std::get<1>(server));
    //     spi.addServerName(config->id, config->name);
    //     // rtos::Thread::signal_wait(CONFIG_MSG_ACK);
    //     // spi.addServerPass(config->id, config->passKey);
    //     // spi.addServerUUID(config->id, config->uuid);
    // }

    spi.setMode(Modes::DISCOVERY);
    rtos::Thread::signal_wait(CONFIG_MSG_ACK);


    while(serversToDiscover)
    {
        rtos::Thread::signal_wait(SERVER_DISCOVERY_COMPLETE);
        serversToDiscover--;
    }

    spi.setMode(Modes::CONFIG);
    // spi.setMode(Modes::RUN);
    configOk = false;
}

bool Nrf51822Interface::storeConfig()
{
    return false;
}

bool Nrf51822Interface::readCharacteristic(const BleServerConfig& server, const CharcteristicDescriptor& characteristic)
{
    return spi.readCharacteristic(server.handle, characteristic);
}

bool Nrf51822Interface::writeCharacteristic(const BleServerConfig& server,
                                     const CharcteristicDescriptor& characteristic,
                                     const uint8_t* data,
                                     const size_t len)
{
    return spi.writeCharacteristic(server.handle, characteristic, data, len);
}

void Nrf51822Interface::handleDeviceDiscovery(uint8_t client, const uint8_t* data)
{
    ServerIdentificator serverId(data);

    if(searchServers.find(serverId.name) != searchServers.end())
    {
        auto& serverList = searchServers[serverId.name];
        if(!serverList.empty())
        {
            auto& server = serverList.front();
            servers.emplace(client, server);
            std::get<1>(servers[client])->handle = client;
            serverList.pop_front();
        }
    }
    std::get<0>(servers[client])(BleEvent::DISCOVERY_COMPLETE, data, sizeof(ServerIdentificator));
}

void Nrf51822Interface::serverDiscoveryComlpete(BleServerConfig& config)
{
    size_t server = static_cast<size_t>(config.handle);
    std::get<2>(servers[server]) = true;
    configurator.signal_set(SERVER_DISCOVERY_COMPLETE);
}

BleEvent fieldId2BleEvent(FieldId fId, Operation op)
{
    BleEvent event = BleEvent::NONE;
    switch(fId)
    {
        case FieldId::CONTROL_CHARACTERISITC_DONE:
            if(Operation::READ == op)
            {
                event = BleEvent::DISCOVERY_CHARACTERISTIC_READ_DONE;
            }
            else
            {
                event = BleEvent::DISCOVERY_CHARACTERISTIC_WRITE_DONE;
            }
            break;
        default:
            break;
    };
    return event;
}

void Nrf51822Interface::spiCallback()
{
    SpiFrame incoming;
    spi.read(reinterpret_cast<char*>(&incoming), sizeof(incoming));

    switch(incoming.dataId)
    {
        case DataId::DEV_0: // intentional fall-through
        case DataId::DEV_1: // intentional fall-through
        case DataId::DEV_2: // intentional fall-through
        case DataId::DEV_3: // intentional fall-through
        case DataId::DEV_4: // intentional fall-through
        case DataId::DEV_5:
            if(FieldId::CONTROL_DISCOVERY_COMPLETE == incoming.fieldId)
            {
                handleDeviceDiscovery(static_cast<uint8_t>(incoming.dataId), incoming.data);
            }
            else
            {
                size_t server = static_cast<size_t>(incoming.dataId);
                std::get<0>(servers[server])(fieldId2BleEvent(incoming.fieldId, incoming.operation),
                                                      incoming.data,
                                                      sizeof(incoming.data));
            }
            break;
        case DataId::CONFIG:
            break;
        case DataId::ERROR:
            break;
        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == incoming.fieldId)
            {
                configurator.signal_set(FW_VERSION_READ);
            }
            if(FieldId::CONFIG_ACK == incoming.fieldId)
            {
                configurator.signal_set(CONFIG_MSG_ACK);
            }
        default:
            break;
    }

    return;
}
