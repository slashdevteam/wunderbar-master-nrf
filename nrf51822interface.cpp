#include "nrf51822interface.h"

#include  <algorithm>
#include "istdinout.h"
#include "mbed.h"

const int32_t SIGNAL_FW_VERSION_READ = 0x1;
const int32_t SIGNAL_ONBOARDING_DONE = 0x2;
const int32_t SIGNAL_CONFIG_ACK      = 0x4;
const int32_t SIGNAL_CONFIG_COMPLETE = 0x8;

Nrf51822Interface::Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq, IStdInOut* _log)
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
    // Have sorted list of all registered servers for comparison during onboarding
    serverList.sort();

    // perform onboarding
    configurator.start(mbed::callback(this, &Nrf51822Interface::onboardSensors));
    configurator.join();

    return configOk;
}

void Nrf51822Interface::startOperation()
{
    // move to run mode
    log->printf("Moving to run mode...\n");
    nrfDriver.setMode(Modes::RUN);
}

void Nrf51822Interface::onboardSensors()
{   
    // signal should be already received from initial NRF reset
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);

    //use new passkeys for all servers 
    nrfDriver.requestPasskeyStoring();
    rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK);

    log->printf("Sending passwords to NRF...\n");
    for(auto& server : servers)
    {
        auto& config = std::get<ServerInfo>(server).serverConfig;
        nrfDriver.configServerPass(ServerNamesToDataId.at(config->name), config->passKey.data());
        rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK);
    }
    log->printf("...done!");

    // request config mode from NRF to perform sensor onboarding
    log->printf("Commencing sensors' onboarding...\n");
    nrfDriver.setMode(Modes::CONFIG);

    // wait till onboarding is done and all data is stored in the NRF's NVRAM
    rtos::Thread::signal_wait(SIGNAL_ONBOARDING_DONE);
    rtos::Thread::signal_wait(SIGNAL_CONFIG_COMPLETE);
    log->printf("...done!");

    // to confiugure new services NRF has to be restarted
    log->printf("Reseting NRF...\n");
    nrfDriver.softwareReset();

    // wait for callback after restart
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);
    log->printf("...done!");
}

bool Nrf51822Interface::storeConfig()
{
    return false;
}

bool Nrf51822Interface::requestCharacteristicRead(const BleServerConfig& server, uint16_t bleCharUuid)
{
    nrfDriver.requestCharacteristicRead(ServerNamesToDataId.at(server.name), CharUuidToFieldId.at(bleCharUuid));
    return true;
}

bool Nrf51822Interface::requestCharacteristicWrite(const BleServerConfig& server,
                                                   uint16_t               bleCharUuid,
                                                   const uint8_t*         data,
                                                   const size_t           len)
{
    nrfDriver.requestCharacteristicWrite(ServerNamesToDataId.at(server.name), CharUuidToFieldId.at(bleCharUuid), data, len);
    return true;
}

void Nrf51822Interface::handleOnboarding(SpiFrame& inboundFrame)
{
    auto serverId = inboundFrame.dataId;
    auto serverEntry = std::find(serverList.begin(), serverList.end(), inboundFrame.dataId);

    if(serverEntry != serverList.end())
    {

        if (FieldId::ONBOARD_DONE  == inboundFrame.fieldId ||
            FieldId::SENSOR_STATUS == inboundFrame.fieldId)
        {
            // mark a event received for a given server; orded is constant
            servers[serverId].onboardInfo.onboardSeq.push_back(inboundFrame.fieldId);

            // if all events arrived, server is considered onboarded
            if (reqConfigFields == servers[serverId].onboardInfo.onboardSeq)
            {   
                log->printf("Server %s onboarded, waiting for confirmation...\n", servers[serverId].serverConfig->name.c_str());
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

    log->printf("...server discovery confirmed by %s!\n", config.name.c_str());

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
            else if (FieldId::CONFIG_COMPLETE == inbound.fieldId)
            {
                configurator.signal_set(SIGNAL_CONFIG_COMPLETE);
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
