/*******************************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://github.com/juanmb/skypointer

*******************************************************************************/
#define DEBUG

#include "SkyPointer_MotorShield.h"

#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <TimerOne.h>

// Serial Command
SerialCommand sCmd;
// Shield
MotorShield MS = MotorShield();
// Motors
Motor AZ = Motor(X);
Motor ALT = Motor(Y);


        /*** Interruptions ***/
void ISR_timer() {
    uint32_t t_on = MS.getTimeOn();
    if(t_on >= LASER_ON_TIME) {
        MS.laser(0);
        Timer1.detachInterrupt();
        #ifdef DEBUG
            Serial.println("LASER OFF");
        #endif
    }
    MS.setTimeOn(t_on + (uint32_t)DT);
}

void ISR_rotate() {
    // Azimuth motor
    if (!AZ.isTarget()){
        AZ.microstep(AZ.getDirection());
    }
    // Altitude motor
    if (MS.home) {
        if (analogRead(PHOTO_PIN < 512)) {
            ALT.microstep(BW);
        }    
        else {
            ALT.setPosition(0);
        }
    }
    else{
        if (!ALT.isTarget()) {
            ALT.microstep(ALT.getDirection());
        }
    }
    #ifdef DEBUG
        Serial.println(AZ.getPosition(), DEC);
    #endif
    if (AZ.isTarget() && ALT.isTarget()) {
        Timer1.detachInterrupt();
        #ifdef DEBUG
            Serial.println("ROTATION DONE");
        #endif
        MS.setTimeOn((uint32_t)(0));
        Timer1.attachInterrupt(ISR_timer);
    }
}


        /*** Serial Comunications ***/
void ProcessGoto() {
    uint16_t tgt1, tgt2;
    tgt1 = MOD(atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD(atoi(sCmd.next()), USTEPS_REV);
    AZ.setTarget(tgt1);
    ALT.setTarget(tgt2);
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessMove() {
    uint16_t tgt1, tgt2;
    tgt1 = MOD((int16_t)AZ.getPosition() + atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD((int16_t)ALT.getPosition() + atoi(sCmd.next()), USTEPS_REV);
    AZ.setTarget(tgt1);
    ALT.setTarget(tgt2);
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessStop() {
    AZ.setTarget(AZ.getPosition());
    ALT.setTarget(ALT.getPosition());
    Serial.print("OK\r");
    Timer1.attachInterrupt(ISR_rotate);
}

void ProcessGetPos() {
    char buf[13];
    sprintf(buf, "P %04d %04d\r", AZ.getPosition(), ALT.getPosition());
    Serial.print(buf);
}

void ProcessHome() {

}

void ProcessLaser() {
    uint8_t enable = atoi(sCmd.next()) != 0;
    MS.laser(enable);
    Serial.print("OK\r");
    #ifdef DEBUG
        //if(enable) { Serial.println("LASER ON"); }
        //else { Serial.println("LASER OFF"); }
    #endif
}

void ProcessID() {
    Serial.print("SkyPointer 2.0\r");
}

void ProcessWriteCalib() {

}

void ProcessReadCalib() {

}

void ProcessQuit() {

}

void Unrecognized() {
    Serial.print("NK\r");
}


        /*** Main Program ***/
void setup () {
    MS.config();  // Configure pins

    // Protocol Commands
    sCmd.addCommand("G", ProcessGoto);      // G pos1 pos2\r    
    sCmd.addCommand("M", ProcessMove);      // M rel1 rel2\r
    sCmd.addCommand("S", ProcessStop);      // S\r
    sCmd.addCommand("P", ProcessGetPos);    // P\r
    sCmd.addCommand("H", ProcessHome);      // H\r
    sCmd.addCommand("L", ProcessLaser);     // L enable\r
    sCmd.addCommand("I", ProcessID);        // I\r
    sCmd.addCommand("W", ProcessWriteCalib);//
    sCmd.addCommand("R", ProcessReadCalib); //
    sCmd.addCommand("Q", ProcessQuit);      // Q\r
    sCmd.addDefaultHandler(Unrecognized);   // Unknown commands

    Serial.begin(115200);
    Timer1.initialize(DT);
}

void loop () {
    // Wait for commands in the serial port
    sCmd.readSerial();
}
