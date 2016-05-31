/*******************************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://github.com/juanmb/skypointer

*******************************************************************************/
// If DEVELOP is defined, DT = 20ms
#define DEBUG

#include "SkyPointer_MotorShield.h"
//#include "Communications.h"

#include <SoftwareSerial.h>
#include <SerialCommand.h>
//#include <TimerOne.h>


// Motor for Azimut axis
Motor AZ = Motor(X);

void setup () {
    config_shield();

    Serial.begin(115200);
}

void loop () {
    AZ.rotate(FW);

#ifdef DEBUG
    // DEBUG
    uint16_t currPos = AZ.getPosition();
    Serial.print("POS: ");
    Serial.println(currPos, DEC);
#endif

    delayMicroseconds(DT);
}
