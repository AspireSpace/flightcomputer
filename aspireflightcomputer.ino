#include <Adafruit_FRAM_I2C.h>
#include <SparkFun_ADXL345.h>

// Use the ADXL in I2C mode
ADXL345 adxl = ADXL345();

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

int serialRead = 2;

void setup() {

  // Setup Pin 2 as an input 
  pinMode(serialRead, INPUT);

  // Set the serial baud rate to 9600
  Serial.begin(9600);

  // Power on the ADXL345.
  adxl.powerOn();         

  // Give the range settings Accepted values are 2g, 4g, 8g or 16g
  adxl.setRangeSetting(2);   
                               
}

void loop() {

  int buttonState = digitalRead(serialRead);
  
   // If button high then log if not dump.
  if(buttonState) {

    fram.begin();

    Serial.print("Reading Sensors and writing to FRAM");

    // Accelerometer Readings.
    int x,y,z;   

    // Loop around all memory locations.
    for (uint16_t i = 0; i < 32768; i+=3) {

      // Read the AXDL values for x,y,z
      adxl.readAccel(&x, &y, &z);

      // Write to the memory
      fram.write8(i, x);
      fram.write8(i+1, y);
      fram.write8(i+2, z);

      // TODO - We need to calculate the correct timing of the AXDL345 
      // How quick does it take to read from the I2C bus?
      // We need to try and take 200/sec
      // How fast does each command take? how quick is the readAccel reading?
      
    }

  } else {

    fram.begin();
    
    // Dump FRAM here to the serial port.
    for (uint16_t a = 0; a < 32768; a++) {
      uint16_t value = fram.read8(a);
      Serial.print(a);

      Serial.print("Memory Location Read: ");
      Serial.print(a);
      Serial.print("\n\n");
      Serial.print("Value:");
      Serial.print(value);
      Serial.print("\n\n");
    }

  }

  // Stop running until Ardiuno is reset.
  exit(0);

}
