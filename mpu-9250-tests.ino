#include <Wire.h>
#include <SPI.h>
#include <MPU9250.h>
#include "asp_SPIFlash.h"

/*
 * Comment on removal of quaternionFilters.h:
 * Sparkfun's MPU-9250 library suggests the use of an orientation estimator based on Madgwick's algorithm.
 * However, I strongly suspect that it is useless for rocketry without extensive modification:
 * it uses measured acceleration and measured magnetic field as absolute reference vectors, under the assumption
 * that measured acceleration will be the local gravity vector.
 * This is not a reasonable approximation in a rocket: measured acceleration is independent of the Earth's gravity.
 * 
 * We'll need to investigate other options for sensor fusion. Even without using the onboard sensor fusion 
 * algorithm, the MPU9250 appears to be quite a nice chip for our purposes.
*/
//#include <quaternionFilters.h> 

// switches, LEDs, and other interface things
const uint8_t SWITCH_PIN = 7;

// flow control things
//volatile bool sampling_terminated = false;
volatile bool already_printed_full_message = false;

// define flash-memory-related stuff
const uint32_t FLASHMEM_CAPACITY_TO_USE = 64000; // enough for 10 seconds of samples. The next version of this code will use all of the flash memory, and save how far it got.
const uint32_t FLASHMEM_RESERVED_BYTES = 256; // will be handy for e.g. status flags.

const uint8_t FLASHMEM_CS_PIN = 6;

volatile uint32_t current_sample_addr = FLASHMEM_RESERVED_BYTES;

SPIFlashChip flashmem;

// define IMU-related stuff
MPU9250 testIMU;
const int MPU_9250_int_pin = 2;
volatile bool MPU_9250_sample_ready = false;

volatile unsigned long MPU_9250_sample_timestamp = micros();
unsigned long MPU_9250_sample_timestamp_to_write; // I feel like there should be a more elegant way of doing this robustly

//define other useful functions


// general sample-related stuff
struct sensor_sample 
{
  uint32_t timestamp; // 4 bytes
  int16_t accel_data[3]; // 6 bytes total
  int16_t gyro_data[3]; // 6 bytes total
  int16_t mag_data[3]; // 6 bytes total  

  uint8_t altitude_data[3]; // 3 bytes (for a 20-bit sample)

   // The current flash memory implementation has lowest jitter when saved sample sizes are a factor of the page length, so pad sample to 32 bytes.
  uint8_t PADDING[7];
};

void serial_dump_sample(sensor_sample theSample);

sensor_sample working_sample;

void setup() {
  Serial.begin(38400);     

  Wire.begin();
  Wire.setClock(400000L); // Need 400 kHz I2C clock to get data fast enough.

  SPI.begin();
  
  // set up interfaces
  pinMode(SWITCH_PIN, INPUT);
  pinMode(13, OUTPUT);
  pinMode(FLASHMEM_CS_PIN, OUTPUT);  

  // Stolen from example code: test MPU-9250 communications by reading the WHO_AM_I register, then set it to active read mode
  byte c = testIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  testIMU.initMPU9250();
  testIMU.initAK8963(testIMU.magCalibration);

  // set up flash memory
  Serial.print("Attempting to set up flash memory...");

  delay(50);

  if (flashmem.begin(FLASHMEM_CS_PIN))
  {
    Serial.println(" success."); 
  } else {
    Serial.println(" error.");    
  }

  delay(50);

  // Only erase memory if it's set to start logging - otherwise need to preserve previous stuff.
  if (digitalRead(SWITCH_PIN))
  {
    Serial.println("Erasing previous data...");
    flashmem.erase_64K_block(0);
    while(flashmem.is_busy());
    Serial.println("Done.");
  }

  already_printed_full_message = false;

  // Final step: clear the MPU-9250 "data ready" register - a sample will almost certainly be ready already, 
  // so we discard it and wait for the next one, whose timestamp will be more accurate.
  testIMU.readByte(MPU9250_ADDRESS, INT_STATUS); 
}

void loop() {
  int buttonState = digitalRead(SWITCH_PIN);

  volatile uint32_t t1, t2, t3;
  
   // If button high then log if not dump.
  if(buttonState) {
    //Serial.println("Reading sensors and writing to flash memory.");

    if (current_sample_addr < (FLASHMEM_CAPACITY_TO_USE + FLASHMEM_RESERVED_BYTES))
    {
      MPU_9250_sample_ready = digitalRead(MPU_9250_int_pin);
      
      if(MPU_9250_sample_ready)
      {
        t1 = micros();
        working_sample.timestamp=micros();

        // these are 3×16-bit arrays. 22 bytes per sample. 
        testIMU.readAccelData(working_sample.accel_data);
        testIMU.readGyroData(working_sample.gyro_data);
        testIMU.readMagData(working_sample.mag_data);
        
        // Under the default settings of the Arduino MPU-9250 library, we clear the interrupt by reading the INT_STATUS register.
        // This returns a byte which we just discard.
        testIMU.readByte(MPU9250_ADDRESS, INT_STATUS);

        // write sample to memory
        t2 = micros();
        flashmem.write_some_data(reinterpret_cast<byte*>(&working_sample), sizeof(working_sample), current_sample_addr);   
        while(flashmem.is_busy()); // "I'm not standing still/I am lying in wait" (for the chip to finish writing)
        t3 = micros();

        current_sample_addr += sizeof(working_sample);

        // Lines below can be used to measure timing.
        /*Serial.print("dt = ");
        Serial.print(t2-t1);
        Serial.print(", dt2 = ");
        Serial.println(t3-t2);
        delay(500);*/
      }
    }
    else
    {
      if (!already_printed_full_message)
      {
        Serial.println("Mem full.");
        delay(100);
        already_printed_full_message = true;
      }
    }
  } else {
        
    uint16_t ii = FLASHMEM_RESERVED_BYTES;
    
    int16_t value;
    unsigned long timestamp;
    
    Serial.println("t, ax, ay, az, gx, gy, gz, mx, my, mz");

    // Dump flash memory here to the serial port.
    while (ii < FLASHMEM_CAPACITY_TO_USE + FLASHMEM_RESERVED_BYTES) {
      flashmem.read_some_data(reinterpret_cast<byte*>(&working_sample), sizeof(working_sample), ii);
      serial_dump_sample(working_sample);

      ii += sizeof(sensor_sample);
    }
    
    Serial.println("All data dumped, exiting.");
    
    // give it a little time to finish printing to serial port
    delay(100);
    // Stop running until Arduino is reset.
    exit(0);
  }
}

void serial_dump_sample(sensor_sample theSample)
{
  Serial.print(theSample.timestamp);
  Serial.print(" ");
  
  for (int i=0; i < 3; i++)
  {
    Serial.print(theSample.accel_data[i]);
    Serial.print(" ");
  }
  for (int i=0; i < 3; i++)
  {
    Serial.print(theSample.gyro_data[i]);
    Serial.print(" ");  
  }
  for (int i=0; i < 3; i++)
  {
    Serial.print(theSample.mag_data[i]);
    Serial.print(" ");
  }

  Serial.println(" ");
}
