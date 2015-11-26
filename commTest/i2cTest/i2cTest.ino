// Tests for I2C Protocol


#include <SoftwareSerial.h>
#include <Wire.h>
#include <SkyPointer_MotorShield.h>



SkyPointer_MotorShield MS = SkyPointer_MotorShield();

SkyPointer_MicroStepper *motor1 =MS.getMicroStepper(200, 1);


#define ADDR 0x60






void setup () {
  #ifdef DEBUG
    pinMode (blinkLed, OUTPUT);
    digitalWrite (blinkLed, LOW);   // Turn off the LED
  #endif

  Serial.begin(115200);
  //MS.begin();
  
  
  Wire.begin();
  
  Wire.beginTransmission(ADDR);
  Wire.write(0x0);
  Wire.write(0x21);
  Wire.endTransmission();
  
  Wire.beginTransmission(ADDR);
  Wire.write(0x04);
  Wire.endTransmission();
  
  Wire.requestFrom(ADDR, 1);
  //uint8_t data = Wire.read();
  
  //Serial.println(data, BIN);
  
}


void loop () {

}
