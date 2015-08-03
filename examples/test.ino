
#include "Wire.h"
#include <SkyPointer_MotorShield.h>
#include <utility/Adafruit_PWMServoDriver.h>

// Define Shield
SkyPointer_MotorShield MS = SkyPointer_MotorShield();
// Attach a motor in microstep mode
SkyPointer_MicroStepper *motor = MS.getMicroStepper (200, 1);

int pos;

void setup () {
    MS.begin();           // Start the motor shield
    Serial.begin(115200); // Start serial communications
}

void loop () {
    motor->microstep(1, FORWARD);   // Rotate 1 microstep
    Serial.print("Current Position [microsteps]: ");
    Serial.println(motor->getPosition());
    
    delay(50);                     // Wait a moment...
}
