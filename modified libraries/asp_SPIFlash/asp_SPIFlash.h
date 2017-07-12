/* SPIFlash library - a small library for basic I/O functions for an SPI flash chip.
 * This version assumes the chip is the Winbond W25Q128FV.
 * Written by Rob Sandford for AspireSpace Arduino-based flight computer.
 */ 
#include <SPI.h>


#ifndef ASPSPIFLASH_h_
#define ASPSPIFLASH_h_

class SPIFlashChip
{
	uint8_t CS_pin;
public:
	bool begin(uint8_t pin = 6);
	
	void read_JEDEC_ID(uint8_t* buf); // needs a three-byte buffer
	uint8_t read_status_register(uint8_t regID);
	void erase_chip(void);
	void SPI_enqueue_address(uint32_t address);

	void erase_64K_block(uint32_t address); //address must be a multiple of 65536.

	uint8_t write_some_data(byte* data_ptr, uint16_t data_length, uint32_t address);
	void read_some_data(byte* data_ptr, uint16_t data_length, uint32_t address);

	bool is_busy(void);
};

#endif