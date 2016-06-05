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
#define DEBUG_DIR

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
Motor DE = Motor(Y);


        /*** Interruptions ***/
void ISR_rotate(void) {
    uint8_t dir;
    // Azimuth motor
    if (!AZ.isTarget()){
        dir = AZ.guessDirection();
        AZ.microstep(dir);
    }
    // Declinatin motor
    if (MS.home) {
        if (analogRead(PHOTO_PIN < 512)) {
            DE.microstep(BW);
        }    
        else {
            DE.setPosition(0);
        }
    }
    else{
        if (!DE.isTarget()) {
            dir = DE.guessDirection();
            DE.microstep(dir);
        }
    }
    if (AZ.isTarget() && DE.isTarget()) {
        Timer1.detachInterrupt();
        // TO-DO -- Laser Temporization
    }
}


        /*** Serial Comunications ***/
void ProcessGoto(void) {
    uint16_t tgt1, tgt2;
    //tgt1 = 
}


        /*** Main Program ***/
void setup () {
    MS.config();  // Configure pins

    Serial.begin(115200);
    Timer1.initialize(DT);
    Timer1.attachInterrupt(ISR_rotate);
}

void loop () {
    // Wait for commands in the serial port
    //sCmd.readSerial();
    AZ.setTarget(3100);

#ifdef DEBUG
    // DEBUG
    uint16_t pos = AZ.getPosition();
    //Serial.print("POS: ");
    //Serial.println(pos, DEC);
#endif
    delayMicroseconds(DT);
}
