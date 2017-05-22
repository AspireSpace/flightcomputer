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

    Serial.print("write");


    // Accelerometer Readings.
    int x,y,z;   

    x = 9;
    y = 10;
    z = 11;

    // Loop around all memory locations.
    for (uint16_t i = 0; i < 32768; i+=3) {
      
      adxl.readAccel(&x, &y, &z);

      /*Serial.print("Memory Location Write: ");
      Serial.print(i);
      Serial.print("\n\n");
      Serial.print("Value:");
      Serial.print(x);
      Serial.print("\n\n");*/
      fram.write8(i, x);
      fram.write8(i+1, y);
      fram.write8(i+2, z);

      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      
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
    // Stop running until Ardiuno is reset.
    exit(0);
  }

}
