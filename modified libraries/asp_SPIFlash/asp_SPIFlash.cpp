#include "asp_SPIFlash.h"

// Flash memory register addresses:
const byte READ_REGISTER[3] = {0x05, 0x35, 0x15};

// Flash memory commands:
const byte MEM_GET_JEDEC_ID = 0x9F;
const byte MEM_WRITE_ENABLE = 0x06;
const byte MEM_PAGE_PROGRAM = 0x02;
const byte MEM_READ = 0x03;
const byte MEM_ERASE_64K_BLOCK = 0xD8;
const byte MEM_ERASE_ENTIRE_CHIP = 0xC7;

// Flash memory misc constants:
const uint16_t PAGE_LENGTH = 0x100;
const uint32_t MEM_LENGTH = 0x1000000;
const SPISettings MEM_SPI_SETTINGS = SPISettings(4000000L, MSBFIRST, SPI_MODE0);

const uint8_t STATUS_REGISTER_FLAG_IS_BUSY = 0x01;
const uint8_t EXPECTED_JEDEC_ID_0 = 0x17; 
const uint8_t EXPECTED_JEDEC_ID_1 = 0x40; 
const uint8_t EXPECTED_JEDEC_ID_2 = 0x18; 

////// SPIFlashChip functions
bool SPIFlashChip::begin(uint8_t pin)
{
	CS_pin = pin;
	delay(10); // give things time to get set up. Probably unnecessary, but nevertheless.
	
	uint8_t JEDEC_ID_buffer[3];
	read_JEDEC_ID(JEDEC_ID_buffer);
	
	return (JEDEC_ID_buffer[0]==EXPECTED_JEDEC_ID_0) && (JEDEC_ID_buffer[1]==EXPECTED_JEDEC_ID_1) && (JEDEC_ID_buffer[2]==EXPECTED_JEDEC_ID_2);
}

void SPIFlashChip::read_JEDEC_ID(uint8_t *buf)
{
  SPI.beginTransaction(MEM_SPI_SETTINGS);
  digitalWrite(CS_pin, LOW);
  SPI.transfer(MEM_GET_JEDEC_ID);
  buf[0] = SPI.transfer(0);
  buf[1] = SPI.transfer(0);
  buf[2] = SPI.transfer(0);  
  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();
}

void SPIFlashChip::erase_chip(void)
{
  Serial.println("Sending chip erase command. On a bad day this may take as long as 200 seconds to complete. Sit tight.");
  SPI.beginTransaction(MEM_SPI_SETTINGS);  
  digitalWrite(CS_pin, LOW); 
  SPI.transfer(MEM_WRITE_ENABLE);
  digitalWrite(CS_pin, HIGH);
  
  digitalWrite(CS_pin, LOW);
  SPI.transfer(MEM_ERASE_ENTIRE_CHIP);
  digitalWrite(CS_pin, HIGH);  
  SPI.endTransaction();
}

uint8_t SPIFlashChip::read_status_register(uint8_t regID)
{
  SPI.beginTransaction(MEM_SPI_SETTINGS);
  digitalWrite(CS_pin, LOW);
  
  SPI.transfer(READ_REGISTER[regID - 1]);
  
  uint8_t retval = SPI.transfer(0);  
  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();  
 
  return retval; 
}

void SPIFlashChip::SPI_enqueue_address(uint32_t address)
{
  byte* sptr = reinterpret_cast<byte*>(&address);

  // SPI.transfer(*(sptr+3)); // placeholder for when we move to chips needing 32-bit addresses
  SPI.transfer(*(sptr+2));  
  SPI.transfer(*(sptr+1));  
  SPI.transfer(*(sptr));   
}

bool SPIFlashChip::is_busy(void)
{
  return ((read_status_register(1) & 0x01) > 0);
}

uint8_t SPIFlashChip::write_some_data(byte* data_ptr, uint16_t data_length, uint32_t address)
{
  SPI.beginTransaction(MEM_SPI_SETTINGS);
  // Enable writing
  digitalWrite(CS_pin, LOW); 
  SPI.transfer(MEM_WRITE_ENABLE);
  digitalWrite(CS_pin, HIGH);

  digitalWrite(CS_pin, LOW);
  SPI.transfer(MEM_PAGE_PROGRAM);
  
  SPI_enqueue_address(address);

  uint8_t ii = 0;
  // enqueue bytes for writing until we reach the length of the data, the end of the page, or the end of the chip.
  // if you paged-write past the end of a page, the flash memory chip will silently loop back to the beginning of that page, which is clearly undesirable.
  while ((ii < data_length) && (((address + ii) % PAGE_LENGTH) >= (address % PAGE_LENGTH)) && ((address + ii) < MEM_LENGTH))
  {
    SPI.transfer(*(data_ptr+ii));
    ii++;
  }
  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();
  
  // if there's still data left to write, and we're not off the end of the chip, recurse. 
  if ((address + ii) >= MEM_LENGTH)
  {
    Serial.println("End of chip reached!");
    return 1;
  } 
  else 
  {
    if (ii >= data_length)
    {
      return 0;
    }	
    else
	// reached end of page with data still to write.
	// Note that the current implementation of this is slowish, so for maximum speed try not to write straddling page boundaries.
    {
      while(is_busy()); // "I'm not standing still/I am lying in wait" (for the chip to finish writing)
      return write_some_data(data_ptr+ii, data_length-ii, address+ii);
    }
  }
}

void SPIFlashChip::read_some_data(byte* data_ptr, uint16_t data_length, uint32_t address)
{
  SPI.beginTransaction(MEM_SPI_SETTINGS);
  
  digitalWrite(CS_pin, LOW);    
  
  SPI.transfer(MEM_READ);
  SPI_enqueue_address(address);
  
  for (uint32_t ii = 0; ii < data_length; ii++)
  { 
    *(data_ptr+ii) = SPI.transfer(0);
  }
  
  digitalWrite(CS_pin, HIGH);  
}

void SPIFlashChip::erase_64K_block(uint32_t address) //address must be a multiple of 65536.
{
  if ((address % 65536) == 0)
  {
    Serial.print("Erasing block at ");
    Serial.println(address, HEX);
    
    SPI.beginTransaction(MEM_SPI_SETTINGS);
    // Enable writing (erasure is just writing all FFs)
    digitalWrite(CS_pin, LOW); 
    SPI.transfer(MEM_WRITE_ENABLE);
    digitalWrite(CS_pin, HIGH);
  
    // erase 64 KB block starting at address.
    digitalWrite(CS_pin, LOW);
    SPI.transfer(MEM_ERASE_64K_BLOCK);
    SPI_enqueue_address(address);
    digitalWrite(CS_pin, HIGH);
    SPI.endTransaction();
  }
  else
  {
    Serial.print(address, HEX);
    Serial.println(" isn't aligned with 64 KB block boundaries. Possibly could erase? Not gonna, just in case.");
  }
}