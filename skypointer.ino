#include <SerialCommand.h>
#include "SkyPointer_MotorShield.h"

MotorShield shield = MotorShield();

AccelStepper AZ(AccelStepper::DRIVER, XSTEP, XDIR);
AccelStepper ALT(AccelStepper::DRIVER, YSTEP, YDIR);

SerialCommand sCmd;

void ProcessGoto() {
    uint16_t tgt1, tgt2;
    tgt1 = MOD(atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD(atoi(sCmd.next()), USTEPS_REV);
    AZ.moveTo(tgt1);
    ALT.moveTo(tgt2);
    Serial.print("OK\r");
}

void Unrecognized() {
    Serial.print("NK\r");
}

void setup() {
    pinMode(XSTEP, OUTPUT);
    pinMode(XDIR, OUTPUT);
    //pinMode(ENABLE, OUTPUT);
    
    AZ.setPinsInverted(false, false, true);
    AZ.setEnablePin(ENABLE);
    AZ.setMaxSpeed(MAX_SPEED);
    AZ.setAcceleration(ACCEL);
    
    ALT.setPinsInverted(false, false, true);
    ALT.setEnablePin(ENABLE);
    ALT.setMaxSpeed(MAX_SPEED);
    ALT.setAcceleration(ACCEL);

    sCmd.addCommand("G", ProcessGoto);
    sCmd.addDefaultHandler(Unrecognized);
    
    Serial.begin(BAUDRATE);
}

void loop() {
    sCmd.readSerial();
    AZ.run();
    ALT.run();
}
