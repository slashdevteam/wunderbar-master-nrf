
#include "nrf51822_driver.h"
#include "PinNames.h"

const uint32_t LOW  = 0;
const uint32_t HIGH = 1;
const uint32_t SIGNAL_IRQ = 0x1;
const uint32_t NRF_RESET_TIME_US = 100;
const uint32_t DUMMY_BYTE = 0xFF;

const auto SPI_BITS_PER_FRAME = 8;
// Polarity + phase
const auto SPI_MODE = 1;

Nrf51822Driver::Nrf51822Driver() :
    #ifdef SPI_HW_SSEL
    spi_driver(MOSI, MISO, SCLK, SSEL),
    #else
    ssel(SSEL, HIGH),
    spi_driver(MOSI, MISO, SCLK),
    #endif	
    recv_data_irq(SPI_EXT_INT),
    recv_data_thread()
{
}

void Nrf51822Driver::reset_chip()
{
    // take control of reset pin
    DigitalOut reset_pin(NRF_NRESET);
    // pull reset pin down
    reset_pin = LOW;

    // hold it down
    wait_us(NRF_RESET_TIME_US);

    // pull it up
    reset_pin = HIGH;
}

void Nrf51822Driver::config(spiSlaveReadReqCb cb)
{
    spi_driver.frequency(2e6);
    spi_driver.format(SPI_BITS_PER_FRAME, SPI_MODE);

    recv_data_ext_cb = cb;
    recv_data_irq.mode(PinMode::PullDown);
    recv_data_irq.rise(callback(this, &Nrf51822Driver::recv_data_irq_cb));

    recv_data_thread.start(callback(this, &Nrf51822Driver::recv_data_handler));
}

void Nrf51822Driver::set_recv_ready_cb(spiSlaveReadReqCb cb)
{
    recv_data_ext_cb = cb;
}

void Nrf51822Driver::recv_data_irq_cb()
{
    recv_data_thread.signal_set(SIGNAL_IRQ);
}

void Nrf51822Driver::recv_data_handler() 
{
    while (true)
    {
        Thread::signal_wait(SIGNAL_IRQ);
        if (recv_data_ext_cb)
        {
            recv_data_ext_cb();
        }
    }
}

 void spi_delay()
{
//TODO check if needed
}

void Nrf51822Driver::cs_act()
{
    #ifndef SPI_HW_SSEL
    ssel = LOW;
    spi_delay();
    #endif
}

void Nrf51822Driver::cs_deact()
{
    #ifndef SPI_HW_SSEL
    spi_delay();
    ssel = HIGH;
    #endif
}

void Nrf51822Driver::write(const char* tx_data, uint32_t size)
{
    cs_act();

    for (auto byte = 0; byte < size; ++byte)
    {
        spi_driver.write(tx_data[byte]);
    }

    cs_deact();
}

void Nrf51822Driver::read(char* rx_data, uint32_t size)
{
    cs_act();

    for (auto byte = 0; byte < size; ++byte)
    {
        rx_data[byte] = spi_driver.write(DUMMY_BYTE);
    }

    cs_deact();
}

void Nrf51822Driver::read_write(char* rx_data, const char* tx_data, uint32_t size)
{
    cs_act();

    for (auto byte = 0; byte < size; ++byte)
    {
        rx_data[byte] = spi_driver.write(tx_data[byte]);
    }

    cs_deact();
}

uint32_t Nrf51822Driver::get_n_bytes_sent() const
{
    const uint32_t spi0_tcr_reg = *reinterpret_cast<const volatile uint32_t*>(0x4002C008);
    return (spi0_tcr_reg >> 16);
}

