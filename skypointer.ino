/*******************************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://github.com/juanmb/skypointer

*******************************************************************************/
// If DEBUG is defined, DT = 20ms
#define DEBUG

#include "SkyPointer_MotorShield.h"

#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <TimerOne.h>

// Serial Command
SerialCommand sCmd;
// Shield
MotorShield MS = MotorShield();
// Motor for Azimut axis
Motor AZ = Motor(X);


void ISR_rotate(void) {
    uint8_t dir;
    dir = AZ.guessDirection();
    //AZ.rotate(FW);
}


void setup () {
    MS.config();  // Configure pins

    Serial.begin(115200);
    Timer1.initialize(DT);
    //Timer1.attachInterrupt(ISR_rotate);
}

void loop () {
    // Wait for commands in the serial port
    sCmd.readSerial();

#ifdef DEBUG
    // DEBUG
    uint16_t currPos = AZ.getPosition();
    Serial.print("POS: ");
    Serial.println(currPos, DEC);
#endif
    delayMicroseconds(DT);
}
