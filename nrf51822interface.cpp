#include "nrf51822interface.h"

#include  <algorithm>
#include "istdinout.h"
#include "mbed.h"

#include "wunderbarsensordatatypes.h"

uint16_t FieldIdToCharUuid(FieldId& field)
{
    static const std::unordered_map<FieldId, uint16_t> field2Uuid = {
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
    return field2Uuid.at(field);
}

FieldId CharUuidToFieldId(uint16_t uuid)
{
    static const std::unordered_map<uint16_t, FieldId> uuid2Field = {
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
    return uuid2Field.at(uuid);
}

const int32_t SIGNAL_FW_VERSION_READ = 0x1;
const int32_t SIGNAL_ONBOARDING_DONE = 0x2;
const int32_t SIGNAL_CONFIG_ACK      = 0x4;
const int32_t SIGNAL_CONFIG_COMPLETE = 0x8;

Nrf51822Interface::Nrf51822Interface(PinName _mosi, PinName _miso, PinName _sclk, PinName _ssel, PinName _extIrq, IStdInOut* _log)
    : nrfDriver(_mosi, _miso, _sclk, _ssel, _extIrq),
      onboardMode(osPriorityNormal, 0x400),
      runMode(osPriorityNormal, 0x400),
      configOk(true),
      serverList(),
      serversOnboarded(),
      log(_log)
{
    nrfDriver.off();
    nrfDriver.config();
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
        auto serverId = ServerNamesToDataId(config.name);
        auto existingServer = servers.find(serverId);

        if(existingServer == servers.end())
        {
            servers.emplace(ServerNamesToDataId(config.name), ServerInfo(config, serverCallback));
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

    // Perform onboarding
    onboardMode.start(mbed::callback(this, &Nrf51822Interface::onboardSensors));
    onboardMode.join();

    return configOk;
}

void Nrf51822Interface::startOperation()
{
    runMode.start(mbed::callback(this, &Nrf51822Interface::goToRunMode));
    runMode.join();
}

void Nrf51822Interface::goToRunMode()
{
    // Attach corresponding callback
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::runModeCb));

    // Powering up NRF module
    log->printf("Powering up NRF...\n");
    nrfDriver.on();

    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);
    log->printf("...done!");

    // Move to run mode
    log->printf("Moving to run mode!\n");
    nrfDriver.setMode(Modes::RUN);
}

void Nrf51822Interface::onboardSensors()
{
    // Set callback
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::onboardModeCb));

    // Powering up NRF module
    log->printf("Powering up NRF...\n");
    nrfDriver.on();

    rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);
    log->printf("...done!\r\n");

    //use new passkeys for all servers
    nrfDriver.requestPasskeyStoring();
    rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK);

    log->printf("Sending passwords to NRF...\n");
    for(auto& server : servers)
    {
        auto& config = std::get<ServerInfo>(server).serverConfig;
        nrfDriver.configServerPass(ServerNamesToDataId(config->name), config->passKey.data());
        rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK);
    }
    log->printf("...done!\r\n");
    
    // move to config mode to run flash driver and commence onboarding of requested sensors
    nrfDriver.setMode(Modes::CONFIG);

    log->printf("Saving passwords in NRF's flash memory...\n");
    
    // wait till  all data is stored in the NRF's NVRAM
    rtos::Thread::signal_wait(SIGNAL_CONFIG_COMPLETE);
    log->printf("...done!\r\n");
    
    log->printf("Please put all Bluetooth sensors in onboarding mode by\r\n");
    log->printf("pressing & releasing button on sensor\r\n");
    log->printf("Leds should start blinking.\r\n");
    log->printf("Now press ENTER to continue.\r\n");
    log->getc();
    log->printf("Onboarding started, please wait a moment...\n");
    
    // wait till onboarding is done
    rtos::Thread::signal_wait(SIGNAL_ONBOARDING_DONE);

    // change callback
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::idleModeCb));

    // switching off NRF
    nrfDriver.off();

    log->printf("...done! NRF switched off.\r\n");
}

bool Nrf51822Interface::storeConfig()
{
    return false;
}

bool Nrf51822Interface::requestRead(const BleServerConfig& server, uint16_t bleCharUuid)
{
    nrfDriver.requestCharacteristicRead(ServerNamesToDataId(server.name), CharUuidToFieldId(bleCharUuid));
    return true;
}

bool Nrf51822Interface::requestWrite(const BleServerConfig& server,
                                     uint16_t               bleCharUuid,
                                     const uint8_t*         data,
                                     const size_t           len)
{
    nrfDriver.requestCharacteristicWrite(ServerNamesToDataId(server.name), CharUuidToFieldId(bleCharUuid), data, len);
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
    auto server = ServerNamesToDataId(config.name);

    servers[server].onboardInfo.onboarded = true;
    serversOnboarded.push_back(server);
    serversOnboarded.sort();

    log->printf("...server discovery confirmed by %s!\n", config.name.c_str());

    if (serverList == serversOnboarded)
    {
        log->printf("All servers ready! \n", config.name);
        onboardMode.signal_set(SIGNAL_ONBOARDING_DONE);
    }
}

BleEvent Nrf51822Interface::fieldId2BleEvent(FieldId fId, Operation op)
{
    BleEvent event = BleEvent::NONE;
    switch(fId)
    {
        case FieldId::CHAR_SENSOR_ID:
            event = BleEvent::DATA_SENSOR_ID;
        break;

        case FieldId::CHAR_SENSOR_BEACON_FREQUENCY:
            event = BleEvent::DATA_SENSOR_BEACON_FREQUENCY;
        break;

        case FieldId::CHAR_SENSOR_FREQUENCY:
            event = BleEvent::DATA_SENSOR_FREQUENCY;
        break;

        case FieldId::CHAR_SENSOR_THRESHOLD:
            event = BleEvent::DATA_SENSOR_THRESHOLD;
        break;

        case FieldId::CHAR_SENSOR_CONFIG:
            event = BleEvent::DATA_SENSOR_CONFIG;
        break;

        case FieldId::CHAR_SENSOR_DATA_R:
            if (Operation::WRITE == op)
            {
                event = BleEvent::DATA_SENSOR_NEW_DATA;
            }
        break;

        case FieldId::CHAR_BATTERY_LEVEL:
            event = BleEvent::DATA_BATTERY_LEVEL;
        break;

        case FieldId::CHAR_MANUFACTURER_NAME:
            event = BleEvent::DATA_HARDWARE_REVISION;
        break;

        case FieldId::CHAR_HARDWARE_REVISION:
            event = BleEvent::DATA_HARDWARE_REVISION;
        break;

        case FieldId::CHAR_FIRMWARE_REVISION:
            event = BleEvent::DATA_FIRMWARE_REVISION;
        break;

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

        case FieldId::SENSOR_WRITE_OK:
            event = BleEvent::WRITE_OK;
        default:
            break;
    }

    return event;
}

SpiFrame Nrf51822Interface::readSpiFrame()
{
    SpiFrame inbound;
    nrfDriver.read(reinterpret_cast<char*>(&inbound), sizeof(inbound));
    log->printf("SPI cb, dataId 0x%X, fieldId 0x%X, op 0x%X \n", inbound.dataId, inbound.fieldId, inbound.operation);

    return inbound;
}

void Nrf51822Interface::idleModeCb()
{
    readSpiFrame();
}

void Nrf51822Interface::onboardModeCb()
{
    SpiFrame inbound = readSpiFrame();

    const Thread::State threadState = onboardMode.get_state();
    const bool onboardModeRunning = (threadState > Thread::State::Ready) && (threadState < Thread::State::Deleted);

    if(!onboardModeRunning)
    {
        log->printf("Error occured, %s executed while expected thread not running! \n", __PRETTY_FUNCTION__);
        return;
    }

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
                    if(!servers[inbound.dataId].onboardInfo.onboarded)
                    {
                        handleOnboarding(inbound);
                    }
                }
            }
            break;

        case DataId::CONFIG:

            if (FieldId::CONFIG_ACK == inbound.fieldId)
            {
                onboardMode.signal_set(SIGNAL_CONFIG_ACK);
            }
            else if (FieldId::CONFIG_COMPLETE == inbound.fieldId)
            {
                onboardMode.signal_set(SIGNAL_CONFIG_COMPLETE);
            }
            else if (FieldId::CONFIG_ERROR == inbound.fieldId)
            {
                log->printf("Fatal error, configuration rejected!");
                configOk = false;
             
                onboardMode.terminate();
            }
            break;

        case DataId::ERROR:
            break;

        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == inbound.fieldId)
            {
                onboardMode.signal_set(SIGNAL_FW_VERSION_READ);
            }
            break;

        default:
            break;
    }

}

void Nrf51822Interface::runModeCb()
{
    SpiFrame inbound = readSpiFrame();

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
                        // servers[inbound.dataId].bleServerCb(fieldId2BleEvent(inbound.fieldId, inbound.operation),
                        //                                     inbound.data,
                        //                                     sizeof(inbound.data));
                    }
                }
            }
            break;

        case DataId::ERROR:
            break;

        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == inbound.fieldId)
            {
                const Thread::State threadState = runMode.get_state();
                const bool threadRunning = (threadState > Thread::State::Ready) && (threadState < Thread::State::Deleted);

                if(threadRunning)
                {
                    runMode.signal_set(SIGNAL_FW_VERSION_READ);
                }
            }
            break;

        default:
            break;
    }

}
