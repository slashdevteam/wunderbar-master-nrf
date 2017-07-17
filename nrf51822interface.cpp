#include "nrf51822interface.h"

#include "wunderbarble.h"
#include  <algorithm>

#include "Stream.h"

const int32_t SIGNAL_FW_VERSION_READ = 0x1;
const int32_t SIGNAL_ONBOARDING_DONE = 0x2;
const int32_t SIGNAL_CONFIG_ACK      = 0x4;

Nrf51822Interface::Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq, mbed::Stream* _log)
    : nrfDriver(_mosi, _miso, _sclk, _ssel, _extIrq),
      configurator(osPriorityNormal, 0x400),
      configOk(true),
      serverList(),
      serversOnboarded(),
      log(_log)
{
    nrfDriver.config(mbed::callback(this, &Nrf51822Interface::spiCallback));
    nrfDriver.reset();
}

Nrf51822Interface::~Nrf51822Interface()
{
}

bool Nrf51822Interface::registerServer(BleServerConfig& config, BleServerCallback serverCallback)
{
    bool success = false;

    log->printf("Registering server %s... ", config.name.c_str());

    if(servers.size() < wunderbar::limits::MAX_SERVERS)
    {
        auto serverId = ServerNamesToDataId.at(config.name);
        auto existingServer = servers.find(serverId);
        
        if(existingServer == servers.end())
        {
            servers.emplace(ServerNamesToDataId.at(config.name), ServerInfo(config, serverCallback));
            serverList.emplace_back(serverId);
            success = true;
            log->printf("ok!\n");
        }
        else
        {
            log->printf("failed, server name already on the list!\n");
        }
    }
    else
    {
        log->printf("failed, limit reached!\n");
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
    // have a sorted list for later comparison
    serverList.sort();

    // perform HW reset on NRF module
    nrfDriver.reset();

    // wait for callback after reset
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);

    // configure passkeys for all servers 
    log->printf("Sending passwords to NRF...\n");
    for(auto& server : servers)
    {
        auto& config = std::get<ServerInfo>(server).serverConfig;
        nrfDriver.configServerPass(ServerNamesToDataId.at(config->name), config->passKey);

        // rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK);
        log->printf("...%s done!\n", config->name.c_str());
    }

    // request config to perform sensor onboarding
    log->printf("Commencing sensors' onboarding...\n");
    nrfDriver.setMode(Modes::CONFIG);

    // wait till onboarding is done
    rtos::Thread::signal_wait(SIGNAL_ONBOARDING_DONE);

    // to confiugure new services NRF has to be restarted
    log->printf("Reseting NRF...\n");
    nrfDriver.resetNrfSoftware();

    // wait for callback after restart
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);

    // move to run mode
    log->printf("Moving to run mode...\n");
    nrfDriver.setMode(Modes::RUN);
}

bool Nrf51822Interface::storeConfig()
{
    return false;
}

bool Nrf51822Interface::readCharacteristic(const BleServerConfig& server, const CharcteristicDescriptor& characteristic)
{

    //TODO translate char and return?
    uint8_t ret[20];
    nrfDriver.readCharacteristic(ServerNamesToDataId.at(server.name), FieldId::CHAR_SENSOR_DATA_R, ret);
    return true;
}

bool Nrf51822Interface::writeCharacteristic(const BleServerConfig& server,
                                     const CharcteristicDescriptor& characteristic,
                                     const uint8_t* data,
                                     const size_t len)
{
    //TODO translate char
    nrfDriver.writeCharacteristic(ServerNamesToDataId.at(server.name), FieldId::CHAR_SENSOR_DATA_W, data, len);
    return true;
}

void Nrf51822Interface::handleOnboarding(SpiFrame& inboundFrame)
{
    auto serverId = inboundFrame.dataId;
    auto serverEntry = std::find(serverList.begin(), serverList.end(), inboundFrame.dataId);

    if(serverEntry != serverList.end())
    {

        if (FieldId::CONFIG_ONBOARD_DONE == inboundFrame.fieldId ||
            FieldId::SENSOR_STATUS       == inboundFrame.fieldId)
        {
            // mark a event received for a given server; orded is constant
            servers[serverId].onboardInfo.onboardSeq.push_back(inboundFrame.fieldId);

            // if all events arrived, server is considered onboarded
            if (reqConfigFields == servers[serverId].onboardInfo.onboardSeq)
            {
                servers[serverId].bleServerCb(BleEvent::DISCOVERY_COMPLETE, inboundFrame.data, sizeof(inboundFrame.data));
            }
        }

    }
}

void Nrf51822Interface::serverDiscoveryComlpete(BleServerConfig& config)
{
    auto server = ServerNamesToDataId.at(config.name);

    servers[server].onboardInfo.onboarded = true;
    serversOnboarded.push_back(server);
    serversOnboarded.sort();

    log->printf("Server discovery confirmed by %s \n", config.name.c_str());

    if (serverList == serversOnboarded)
    {
        log->printf("All servers ready! \n", config.name);
        configurator.signal_set(SIGNAL_ONBOARDING_DONE);
    }
}

BleEvent Nrf51822Interface::fieldId2BleEvent(FieldId fId, Operation op)
{
    BleEvent event = BleEvent::NONE;
    switch(fId)
    {
        case FieldId::SENSOR_STATUS:
            if (Operation::CONNECTION_OPENED == op) 
            {
                event = BleEvent::CONNECTION_OPENED;
            }
            else if (Operation::CONNECTION_CLOSED == op)
            {
                event = BleEvent:: CONNECTION_CLOSED;
            } 
            break;

        case FieldId::CHAR_SENSOR_DATA_R:
            if (Operation::WRITE == op)
            {
                event = BleEvent::NEW_DATA_READOUT;
            }
            break;

        default:
            break;
    };

    return event;
}

void Nrf51822Interface::spiCallback()
{
    SpiFrame inbound;
    nrfDriver.read(reinterpret_cast<char*>(&inbound), sizeof(inbound));
    log->printf("SPI cb, dataId 0x%X, fieldId 0x%X, op 0x%X \n", inbound.dataId, inbound.fieldId, inbound.operation);

    switch(inbound.dataId)
    {
        case DataId::DEV_HTU: // intentional fall-through
        case DataId::DEV_GYRO: // intentional fall-through
        case DataId::DEV_LIGHT: // intentional fall-through
        case DataId::DEV_SOUND: // intentional fall-through
        case DataId::DEV_BRIDGE: // intentional fall-through
        case DataId::DEV_IR:
            {   
                // parse data only if server is on the list
                auto serverEntry = std::find(serverList.begin(), serverList.end(), inbound.dataId);

                if(serverEntry != serverList.end())
                {
                    if(servers[inbound.dataId].onboardInfo.onboarded)
                    {   
                        servers[inbound.dataId].bleServerCb(fieldId2BleEvent(inbound.fieldId, inbound.operation),
                                                                            inbound.data,
                                                                            sizeof(inbound.data));
                    }
                    else
                    {
                        handleOnboarding(inbound);
                    }
                }
            }
            break;

        case DataId::CONFIG:

            if (FieldId::CONFIG_ACK == inbound.fieldId)
            {
                configurator.signal_set(SIGNAL_CONFIG_ACK);
            } 
            else if (FieldId::CONFIG_ERROR == inbound.fieldId)
            {
                log->printf("Fatal error, configuration rejected!");
                configOk = false;
                configurator.terminate();
            }
            break;

        case DataId::ERROR:
            break;

        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == inbound.fieldId)
            {
                configurator.signal_set(SIGNAL_FW_VERSION_READ);
            }
            break;

        default:
            break;
    }

    return;
}
