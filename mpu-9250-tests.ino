#include <Wire.h>
#include <SPI.h>
#include <MPU9250.h>

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

#include <Adafruit_FRAM_I2C.h>

// switches, LEDs, and other interfacethings
const uint8_t SWITCH_PIN = 7;

// define FRAM-related stuff
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

const uint16_t FRAM_CAPACITY_TO_USE = 126;
const uint16_t FRAM_RESERVED_BYTES = 16; // could be handy I guess?

// define IMU-related stuff
MPU9250 testIMU;
const int MPU_9250_int_pin = 2;
volatile bool MPU_9250_sample_ready = true;

volatile unsigned long MPU_9250_sample_timestamp = micros();
unsigned long MPU_9250_sample_timestamp_to_write; // I feel like there should be a more elegant way of doing this robustly

//define other useful functions
void fram_write_int16_t_arr(int16_t vals[], int count, uint16_t start_address) ;
int16_t fram_read_int16_t(uint16_t start_address);

void mark_MPU9250_sample_ready(void);

void setup() {
  // Set the serial baud rate to 9600
  Wire.begin();
  Serial.begin(38400);     

  // set up interfaces
  pinMode(SWITCH_PIN, OUTPUT);
  pinMode(13, OUTPUT);

  // Stolen from example code: test MPU-9250 communications by reading the WHO_AM_I register, then set it to active read mode
  byte c = testIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  testIMU.initMPU9250();
  
  // Stolen from example code: Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication. Then set up magnetometer?
  byte d = testIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
  Serial.print(" I should be "); Serial.println(0x48, HEX);  
  
  testIMU.initAK8963(testIMU.magCalibration);

  // set up FRAM
  Serial.print("attempting to set up FRAM...");

  if (fram.begin())
  {
    Serial.println(" success."); 
  } else {
    Serial.println(" error.");    
  }

  for (int ii=0; ii<FRAM_RESERVED_BYTES; ii++)
  {
    fram.write8(ii,0); 
  }

  // set up the interrupt.
  // remark: this uses external interrupts, but Arduino Uno & friends only have two pins that can do this. Pin change interrupts will be needed for others.
  pinMode(MPU_9250_int_pin, INPUT); // example sketch claims that this is the correct way to do it 
  digitalWrite(MPU_9250_int_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(MPU_9250_int_pin), mark_MPU9250_sample_ready, RISING);
}

void loop() {
  int buttonState = digitalRead(SWITCH_PIN);
  
   // If button high then log if not dump.
  if(buttonState) {
    Serial.println("Reading Sensors and writing to FRAM");

    // Loop around all memory locations.
    
    uint16_t ii = FRAM_RESERVED_BYTES-1;
    while (ii < FRAM_CAPACITY_TO_USE)
    {
      
      if(MPU_9250_sample_ready)
      {       
        digitalWrite(13, HIGH);
        
        Serial.print("Sample ready at t=");
        Serial.println(MPU_9250_sample_timestamp_to_write);

        // these are 3×16-bit arrays. 22 bytes per sample. 
        testIMU.readAccelData(testIMU.accelCount);
        testIMU.readGyroData(testIMU.gyroCount);
        testIMU.readMagData(testIMU.magCount);
        
        // Under the default settings of the Arduino MPU-9250 library, we clear the interrupt by reading the INT_STATUS register.
        // This returns a byte which we just discard.
        testIMU.readByte(MPU9250_ADDRESS, INT_STATUS);        
        
        /* Apparently it's necessary to wrap the block below in "noInterrupts".
         * Otherwise an interrupt might trigger halfway through writing the volatile variables 
         * which are used by the interrupt and the main subroutine. 
         * 
         * This, for obvious reasons, would not be great. 
         *
         * (I realised some while after writing this that I could have just kludged it by only clearing the interrupt AFTER this bit,
         * but oh well, it's good practice.)
         */
        noInterrupts(); 
        MPU_9250_sample_ready = false;
        MPU_9250_sample_timestamp_to_write = MPU_9250_sample_timestamp;
        interrupts();
        
        // Write the samples to FRAM 
        fram_write_int16_t_arr(testIMU.accelCount, 3, ii);
        ii+=6;
        fram_write_int16_t_arr(testIMU.gyroCount, 3, ii);
        ii+=6;
        fram_write_int16_t_arr(testIMU.magCount, 3, ii);
        ii+=6;

        //  Also write timestamp to FRAM
        for (uint16_t jj=0; jj<4; jj++)
        {
           fram.write8(ii+jj, (uint8_t) ((MPU_9250_sample_timestamp_to_write >> 8*(3-jj)) & 0x000000FFUL));
        }       
        ii+=4;
       
        // 200 Hz is far too fast to see, so it'll just look like the LED is on when samples are being read
        digitalWrite(13, LOW); 
      }
    }

    Serial.println("Filled FRAM, starting again?");
  } else {    
    uint16_t ii = FRAM_RESERVED_BYTES - 1;
    
    int16_t value;
    unsigned long timestamp;
    
    // Dump FRAM here to the serial port.
    while (ii < FRAM_CAPACITY_TO_USE + FRAM_RESERVED_BYTES) {
      timestamp = 0;
      
      //HEX DUMP MODE - was useful for debugging
      /*Serial.print(fram.read8(ii), HEX);
      Serial.print(" ");
      ii++;
      
      if (((ii-FRAM_RESERVED_BYTES) % 22) == 0)
     {
       Serial.println(" ");
     } */
      
      //RAW SAMPLE DUMP MODE - somewhat more readable. Not in any physical units.
      Serial.print("New sample! ii = ");
      Serial.println(ii);
      for (uint16_t jj = 0; jj<9; jj++)
      {
        // bunch of magic numbers here, not great.
        value = fram_read_int16_t(ii+2*jj);
        Serial.print(value);
        Serial.print(", ");
      }
      Serial.print("t = ");
      
      ii+=18;
      
      Serial.print("(");
      for (uint16_t jj = 0; jj<4; jj++)
      {
        timestamp = timestamp << 8;
        Serial.print(fram.read8(ii+jj));
        Serial.print(", ");
        timestamp |= (unsigned long) (fram.read8(ii+jj));        
      }
      Serial.print(") ");
      Serial.println(timestamp);
      
      ii+=4;
    }

    // Stop running until Arduino is reset.
    exit(0);
  }
}

void mark_MPU9250_sample_ready(void) {
  MPU_9250_sample_ready = true;
  MPU_9250_sample_timestamp = micros();
}

void fram_write_int16_t_arr(int16_t vals[], int count, uint16_t start_address) {
  for (int ii=0; ii<count; ii++)
  {
    fram.write8(start_address+2*ii, highByte(vals[ii]));        
    fram.write8(start_address+2*ii+1, lowByte(vals[ii]));
  }
}

int16_t fram_read_int16_t(uint16_t start_address) {
    int16_t temp_read_num = ((int16_t) (fram.read8(start_address))) << 8;
    temp_read_num |= (int16_t) fram.read8(start_address+1);
    return temp_read_num;
}
