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
      onboardMode(nullptr),
      signalTimeout(osWaitForever),
      runMode(nullptr),
      configOk(false),
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
            // serverList.emplace_back(serverId);
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

void Nrf51822Interface::setTimeout(uint32_t timeout)
{
    if(timeout == 0)
    {
        signalTimeout = osWaitForever;
    }
    else
    {
        signalTimeout = timeout * 1000;
    }
}

bool Nrf51822Interface::configure(const BleConfig& bleConfig)
{
    configOk = false;
    serverList.clear();
    serversOnboarded.clear();
    for(uint32_t serverIdx = 0; serverIdx < WUNDERBAR_SENSORS_NUM; ++serverIdx)
    {
        DataId serverId = static_cast<DataId>(serverIdx);
        if(bleConfig.sensorAvailability[serverIdx] != SensorAvailability::IGNORE)
        {
            serverList.emplace_back(serverId);
        }
        else
        {
            auto existingServer = servers.find(serverId);
            if(existingServer != servers.end())
            {
                servers.erase(existingServer);
            }
        }
    }
    // Have sorted list of all registered servers for comparison during onboarding
    serverList.sort();

    // Perform onboarding
    onboardMode = std::make_unique<rtos::Thread>(osPriorityNormal, 0x400);
    onboardMode->start(mbed::callback(this, &Nrf51822Interface::setPasswords));
    onboardMode->join();
    onboardMode.reset(nullptr);
    return configOk;
}


bool Nrf51822Interface::onboard(BleConfig& config)
{
    configOk = false;
    bleConfig = &config;

    // Have sorted list of all registered servers for comparison during onboarding
    serverList.sort();

    // initially no sensor is available
    for(auto& sensorStatus : bleConfig->sensorAvailability)
    {
        if(sensorStatus != SensorAvailability::IGNORE)
        {
            sensorStatus = SensorAvailability::NOT_AVAILABLE;
        }
    }

    for(auto& server : servers)
    {
        std::get<ServerInfo>(server).onboardInfo.onboarded = false;
        std::get<ServerInfo>(server).onboardInfo.onboardSeq.clear();
    }

    // Perform onboarding
    onboardMode = std::make_unique<rtos::Thread>(osPriorityNormal, 0x400);
    onboardMode->start(mbed::callback(this, &Nrf51822Interface::onboardSensors));
    onboardMode->join();
    onboardMode.reset(nullptr);

    bleConfig = nullptr;
    return configOk;
}

void Nrf51822Interface::startOperation()
{
    serverList.sort();
    runMode = std::make_unique<rtos::Thread>(osPriorityNormal, 0x400);
    runMode->start(mbed::callback(this, &Nrf51822Interface::goToRunMode));
    runMode->join();
    runMode.reset(nullptr);
}

void Nrf51822Interface::stopOperation()
{
    nrfDriver.off();
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

void Nrf51822Interface::setPasswords()
{
    osEvent waitEvent;
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::onboardModeCb));

    // Powering up NRF module
    log->printf("Powering up NRF...\r\n");
    nrfDriver.on();

    waitEvent = rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ, signalTimeout);
    if(osEventSignal == waitEvent.status)
    {
        log->printf("...done!\r\n");
        //use new passkeys for all servers
        nrfDriver.requestPasskeyStoring();
        waitEvent = rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK, signalTimeout);

        if(osEventSignal == waitEvent.status)
        {
            log->printf("Sending passwords to NRF...\r\n");
            for(auto& server : servers)
            {
                auto& config = std::get<ServerInfo>(server).serverConfig;
                uint8_t serverIdx = static_cast<uint8_t>(ServerNamesToDataId(config->name));
                if(bleConfig->sensorAvailability[serverIdx] != SensorAvailability::IGNORE)
                {
                    log->printf("Password for %s: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
                    config->name.c_str(),
                    config->passKey.data()[0],
                    config->passKey.data()[1],
                    config->passKey.data()[2],
                    config->passKey.data()[3],
                    config->passKey.data()[4],
                    config->passKey.data()[5],
                    config->passKey.data()[6],
                    config->passKey.data()[7]);
                    nrfDriver.configServerPass(ServerNamesToDataId(config->name), config->passKey.data());
                    waitEvent = rtos::Thread::signal_wait(SIGNAL_CONFIG_ACK, signalTimeout);
                    if(osEventSignal != waitEvent.status)
                    {
                        log->printf("Error while storing password! Aborting.\r\n");
                        return;
                    }
                }
            }
            log->printf("...done!\r\n");

            // move to config mode to run flash driver and commence onboarding of requested sensors
            nrfDriver.setMode(Modes::CONFIG);

            log->printf("Saving passwords in NRF's flash memory...\r\n");

            // wait till  all data is stored in the NRF's NVRAM
            waitEvent = rtos::Thread::signal_wait(SIGNAL_CONFIG_COMPLETE, signalTimeout);
            if(osEventSignal == waitEvent.status)
            {
                log->printf("...done!\r\n");
                configOk = true;
            }
            else
            {
                log->printf("Failed to store passwords in NRF memory! Aborting.\r\n");
            }
        }
        else
        {
            log->printf("NRF not moving to password store mode! Aborting.\r\n");
        }
    }
    else
    {
        log->printf("No communication with NRF! Aborting.\r\n");
    }

    nrfDriver.off();
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::idleModeCb));
}

void Nrf51822Interface::onboardSensors()
{
    osEvent waitEvent;
    // Set callback
    nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::onboardModeCb));

    nrfDriver.on();
    waitEvent = rtos::Thread::signal_wait(SIGNAL_FW_VERSION_READ);
    if(osEventSignal == waitEvent.status)
    {
        nrfDriver.setMode(Modes::CONFIG);
        // wait till onboarding is done
        waitEvent = rtos::Thread::signal_wait(SIGNAL_ONBOARDING_DONE, signalTimeout);
        if(osEventSignal == waitEvent.status)
        {
            // change callback
            nrfDriver.setRecvReadyCb(mbed::callback(this, &Nrf51822Interface::idleModeCb));

            configOk = true;
            log->printf("...done! NRF switched off.\r\n");
        }
        else
        {
            log->printf("Timeout before all sensors confirmed onboarding.\r\n");
        }
    }
    else
    {
        log->printf("No communication with NRF! Aborting.\r\n");
    }
    // switching off NRF
    nrfDriver.off();
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
                bleConfig->sensorAvailability[static_cast<uint8_t>(serverId)] = SensorAvailability::AVAILABLE;
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

    if(serverList == serversOnboarded)
    {
        log->printf("All servers ready! \n");
        onboardMode->signal_set(SIGNAL_ONBOARDING_DONE);
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

    return inbound;
}

void Nrf51822Interface::idleModeCb()
{
    readSpiFrame();
}

void Nrf51822Interface::onboardModeCb()
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
                onboardMode->signal_set(SIGNAL_CONFIG_ACK);
            }
            else if (FieldId::CONFIG_COMPLETE == inbound.fieldId)
            {
                onboardMode->signal_set(SIGNAL_CONFIG_COMPLETE);
            }
            else if (FieldId::CONFIG_ERROR == inbound.fieldId)
            {
                log->printf("Fatal error, configuration rejected!");
                configOk = false;

                onboardMode.reset();
            }
            break;

        case DataId::ERROR:
            break;

        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == inbound.fieldId)
            {
                onboardMode->signal_set(SIGNAL_FW_VERSION_READ);
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
                    servers[inbound.dataId].bleServerCb(fieldId2BleEvent(inbound.fieldId, inbound.operation),
                                                        inbound.data,
                                                        sizeof(inbound.data));
                }
            }
            break;

        case DataId::ERROR:
            break;

        case DataId::DEV_CENTRAL:
            if(FieldId::CHAR_FIRMWARE_REVISION == inbound.fieldId)
            {
                const Thread::State threadState = runMode->get_state();
                const bool threadRunning = (threadState > Thread::State::Ready) && (threadState < Thread::State::Deleted);

                if(threadRunning)
                {
                    runMode->signal_set(SIGNAL_FW_VERSION_READ);
                }
            }
            break;

        default:
            break;
    }

}
