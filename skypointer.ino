/*******************************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://github.com/juanmb/skypointer

*******************************************************************************/
// If DEBUG is defined, DT = 20ms
//#define DEBUG

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
    tgt1 = MOD(atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD(atoi(sCmd.next()), USTEPS_REV);
    AZ.setTarget(tgt1);
    DE.setTarget(tgt2);
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessMove(void) {
    uint16_t tgt1, tgt2;
    tgt1 = MOD((int16_t)AZ.getPosition() + atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD((int16_t)DE.getPosition() + atoi(sCmd.next()), USTEPS_REV);
    AZ.setTarget(tgt1);
    DE.setTarget(tgt2);
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessStop(void) {
    AZ.setTarget(AZ.getPosition());
    DE.setTarget(DE.getPosition());
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessGetPos(void) {
    char buf[13];
    sprintf(buf, "P %04d %04d\r", AZ.getPosition(), DE.getPosition());
    Serial.print(buf);
}


        /*** Main Program ***/
void setup () {
    MS.config();  // Configure pins

    // Protocol Commands
    sCmd.addCommand("G", ProcessGoto);      // G pos1 pos2\r    
    sCmd.addCommand("M", ProcessMove);      // M rel1 rel2\r
    sCmd.addCommand("S", ProcessStop);      // S\r
    sCmd.addCommand("P", ProcessGetPos);    // P\r

    Serial.begin(115200);
    Timer1.initialize(DT);
}

void loop () {
    // Wait for commands in the serial port
    sCmd.readSerial();
    //AZ.setTarget(100);

#ifdef DEBUG
    // DEBUG
    Serial.print("POS: ");
    Serial.println(AZ.getPosition());
#endif
    //delayMicroseconds(DT);
}
