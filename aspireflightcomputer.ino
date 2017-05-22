#include <Adafruit_FRAM_I2C.h>

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

int serialRead = 2;

void setup() {

  // Setup Pin 2 as an input 
  pinMode(serialRead, INPUT);

  // Set the serial baud rate to 9600
  Serial.begin(9600);

}

void loop() {

  int buttonState = digitalRead(serialRead);
  
   // If button high then log if not dump.
  if(buttonState) {
    
  } else {
    // Dump FRAM here to the serial port.
    for (uint16_t a = 0; a < 32768; a++) {
      uint16_t value = fram.read8(a);
      Serial.print(a);
    }
    // Stop running until Ardiuno is reset.
    exit(0);
  }

}
