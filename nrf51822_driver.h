#pragma once

#include "mbed.h"

const uint32_t DUMMY_BYTE = 0xFF;

// Whether or not HW CS line should be used in SPI driver
#if 0
#define SPI_HW_SSEL
#endif 

class nrf51822_driver
{
    public:
    nrf51822_driver();
    // do HW chip reset
    void reset_chip();
    // config communication settings, sets callback to receive data from nrf when requesting
    void config(void (*recv_func)());
    // sets/changes callback to receive data from nrf when requesting
    void set_recv_ready_cb(void (*recv_func)());

    // sending and receiving data
    void write(char* tx_data, uint32_t size);
    void write(char* tx_data, char* rx_data, uint32_t size);
    void read(char* rx_data, uint32_t size);

    // get total number of sent bytes (wraps arround)
    uint32_t get_n_bytes_sent() const;
    
    private:  
    #ifndef SPI_HW_SSEL
    DigitalOut   ssel;
    #endif
    SPI          spi_driver;
    InterruptIn  recv_data_irq;
    Thread       recv_data_thread;

    void         (*recv_data_ext_cb)();
    void         recv_data_irq_cb();
    void         recv_data_handler();

    //chip select on/off
    void cs_act();
    void cs_deact(); 
};

