/******************************************************************






 ******************************************************************/
 
#include "Wire.h"
#include <SkyPointer_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Define Shield
SkyPointer_MotorShield MS = SkyPointer_MotorShield();
// Attach a motor in microstep mode
SkyPointer_MicroStepper *motor = MS.getMicroStepper (200, 1);

uint16_t pos;

void setup () {
    MS.begin();           // Start the motor shield
    Serial.begin(115200); // Start serial communications
    // Set the speed to 1 rpm
    motor->setSpeed(0.0001);
}

void loop () {
    motor->microstep(1, FORWARD);   // Rotate 1 microstep
//    pos = motor->getPosition();     // Get current position

//    Serial.print("Current Position [microsteps]: ");
//    Serial.println(pos);            // Print current position to Serial
    

    uint32_t interval = motor->getSpeed(); 
    Serial.println(interval);

    delayMicroseconds(interval);                     // Wait a moment...
}
