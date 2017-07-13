#include "nrf51822interface.h"

#include "wunderbarble.h"
#include  <algorithm>


const int32_t SIGNAL_FW_VERSION_READ = 0x1;
const int32_t SIGNAL_ONBOARDING_DONE = 0x2;


Nrf51822Interface::Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq)
    : nrfDriver(_mosi, _miso, _sclk, _ssel, _extIrq),
      configurator(osPriorityNormal, 0x400),
      configOk(false),
      serversToBeOnboarded(),
      serversOnboarded()
{
    // nrfDriver.reset();
    nrfDriver.config(mbed::callback(this, &Nrf51822Interface::spiCallback));
}

Nrf51822Interface::~Nrf51822Interface()
{
}

bool Nrf51822Interface::registerServer(BleServerConfig& config, BleServerCallback inboundCallback)
{
    bool success = false;

    if(servers.size() < wunderbar::limits::MAX_SERVERS)
    {
        auto existingServer = servers.find(ServerNamesToDataId.at(config.name));
        if(existingServer == servers.end())
        {
            if(servers.size() < wunderbar::limits::MAX_SERVERS)
            {
                // std::pair<DataId, ServerInfo> para(static_cast<DataId>(),)
                servers.emplace(static_cast<DataId>(servers.size()), ServerInfo(config, inboundCallback));
                serversToBeOnboarded.emplace_back(ServerNamesToDataId.at(config.name));
                success = true;
            }
        }
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
    serversToBeOnboarded.sort();

    // dummy read for initial FW version packet from NRF
    spiCallback();
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);

    // configure passkeys for all servers 
    for(auto& server : servers)
    {
        auto& config = std::get<ServerInfo>(server).serverConfig;
        nrfDriver.configServerPass(ServerNamesToDataId.at(config->name), config->passKey);
    }
    // request config to perform sensor onboarding
    nrfDriver.setMode(Modes::CONFIG);

    // wait till onboarding is done
    rtos::Thread::signal_wait(SIGNAL_ONBOARDING_DONE);

    // to confiugure new services NRF has to be restarted
    nrfDriver.resetNrfSoftware();

    // wait for callback after restart
    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);

    // move to run mode
    nrfDriver.setMode(Modes::RUN);

    configOk = false;
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

void Nrf51822Interface::handleDeviceDiscovery(SpiFrame& inboundFrame)
{
    auto serverId = inboundFrame.dataId;
    auto serverEntry = std::find(serversToBeOnboarded.begin(), serversToBeOnboarded.end(), inboundFrame.dataId);

    if(serverEntry != serversToBeOnboarded.end())
    {   
        if (FieldId::CONFIG_COMPLETE == inboundFrame.fieldId ||
            FieldId::SENSOR_STATUS   == inboundFrame.fieldId)
        {
            // mark a event received for a given server; orded is constant so sorting not needed
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

    if (serversToBeOnboarded == serversOnboarded)
    {
        configurator.signal_set(SIGNAL_ONBOARDING_DONE);
    }
}

BleEvent fieldId2BleEvent(FieldId fId, Operation op)
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
                event =BleEvent:: CONNECTION_CLOSED;
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

    switch(inbound.dataId)
    {
        case DataId::DEV_HTU: // intentional fall-through
        case DataId::DEV_GYRO: // intentional fall-through
        case DataId::DEV_LIGHT: // intentional fall-through
        case DataId::DEV_SOUND: // intentional fall-through
        case DataId::DEV_BRIDGE: // intentional fall-through
        case DataId::DEV_IR:
            {
                const bool onboardingOngoing = (false == servers[inbound.dataId].onboardInfo.onboarded);

                if(onboardingOngoing)
                {   
                    handleDeviceDiscovery(inbound);
                }
                else
                {
                    servers[inbound.dataId].bleServerCb(fieldId2BleEvent(inbound.fieldId, inbound.operation),
                                                                        inbound.data,
                                                                        sizeof(inbound.data));
                }
            }
            break;
        case DataId::CONFIG:
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
