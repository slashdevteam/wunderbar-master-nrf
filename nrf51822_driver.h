#pragma once

#include "mbed.h"

using spiSlaveReadReqCb = mbed::Callback<void(void)>;

class Nrf51822Driver
{
public:
    Nrf51822Driver();
    // do HW chip reset
    void reset_chip();
    // config communication settings, sets callback to receive data from nrf when requesting
    void config(spiSlaveReadReqCb cb);
    // sets/changes callback to receive data from nrf when requesting
    void set_recv_ready_cb(spiSlaveReadReqCb cb);

    // sending and receiving data
    void write(const char* tx_data, uint32_t size);
    void read(char* rx_data, uint32_t size);
    void read_write(char* rx_data, const char* tx_data, uint32_t size);

    // get total number of sent bytes (wraps arround)
    uint32_t get_n_bytes_sent() const;
    
private:
    #ifndef SPI_HW_SSEL
    DigitalOut   ssel;
    #endif
    SPI          spi_driver;
    InterruptIn  recv_data_irq;
    Thread       recv_data_thread;

    spiSlaveReadReqCb recv_data_ext_cb;
    void              recv_data_irq_cb();
    void              recv_data_handler();

    //chip select on/off
    void cs_act();
    void cs_deact(); 
};

