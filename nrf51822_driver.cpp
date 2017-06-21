
#include "nrf51822_driver.h"
#include "PinNames.h"

enum {
	LOW  = 0,
	HIGH = 1,
	SIGNAL_IRQ = 0x1,
	NRF_RESET_TIME_US = 100
};

nrf51822_driver::nrf51822_driver() :
	#ifdef SPI_HW_SSEL
	spi_driver(MOSI, MISO, SCLK, SSEL),
	#else
	ssel(SSEL, HIGH),
	spi_driver(MOSI, MISO, SCLK),
	#endif	
	recv_data_irq(SPI_EXT_INT),
	recv_data_thread()
	{}

void nrf51822_driver::reset_chip() {
	// take control of reset pin
    DigitalOut reset_pin(NRF_NRESET);
	// pull reset pin down
	reset_pin = LOW;

	// hold it down
	wait_us(NRF_RESET_TIME_US);

	// pull it up
	reset_pin = HIGH;
}

void nrf51822_driver::config(void (*recv_func)()) {
	spi_driver.frequency(2e6);
	spi_driver.format(8,1);

	recv_data_ext_cb = recv_func;
	recv_data_irq.mode(PinMode::PullDown);
	recv_data_irq.rise(callback(this, &nrf51822_driver::recv_data_irq_cb));

	recv_data_thread.start(callback(this, &nrf51822_driver::recv_data_handler));
}

void nrf51822_driver::set_recv_ready_cb(void (*recv_func)()) {
    recv_data_ext_cb = recv_func;
}

void nrf51822_driver::recv_data_irq_cb() {
	recv_data_thread.signal_set(SIGNAL_IRQ);
}

void nrf51822_driver::recv_data_handler() {
	while (true) {
		Thread::signal_wait(SIGNAL_IRQ);
		(*recv_data_ext_cb)();
	}
}

 void spi_delay(){
//TODO check if needed
}

void nrf51822_driver::cs_act() {
	#ifndef SPI_HW_SSEL
    ssel = LOW;
	spi_delay();
	#endif
}

void nrf51822_driver::cs_deact() {
	#ifndef SPI_HW_SSEL
	spi_delay();
	ssel = HIGH;
	#endif
}

void nrf51822_driver::write(char* sendbyte, uint32_t size) {

	cs_act();

	char* dataPtr = sendbyte;
	for (auto byte = 0; byte < size; ++byte, ++dataPtr)
	{
		spi_driver.write(*dataPtr);
	}

	cs_deact();
}

void nrf51822_driver::write(char* sendbyte, char* recvbyte, uint32_t size) {

	cs_act();

	char* dataPtr = sendbyte;
	for (auto byte = 0; byte < size; ++byte, ++dataPtr)
	{
		recvbyte[byte] = spi_driver.write(*dataPtr);
	}

	cs_deact();
}

void nrf51822_driver::read(char* recvbyte, uint32_t size) {

	cs_act();

	char* dataPtr = recvbyte;
	for (auto byte = 0; byte < size; ++byte, ++dataPtr)
	{
		*dataPtr = spi_driver.write(DUMMY_BYTE);
	}

	cs_deact();
}

uint32_t nrf51822_driver::get_n_bytes_sent() const {
	const uint32_t spi0_tcr_reg = *reinterpret_cast<const volatile uint32_t*>(0x4002C008);
    return (spi0_tcr_reg >> 16);
}

