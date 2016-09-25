#include <SerialCommand.h>
#include "SkyPointer_MotorShield.h"


MotorShield shield = MotorShield();

AccelStepper AZ(AccelStepper::DRIVER, XSTEP, XDIR);
AccelStepper ALT(AccelStepper::DRIVER, YSTEP, YDIR);

SerialCommand sCmd;

void ProcessGoto() {
    uint16_t pos, simPos, tgt1, tgt2;
    int16_t delta1, delta2; // SIGNED

    // Get target positions
    tgt1 = MOD(atoi(sCmd.next()), USTEPS_REV);
    tgt2 = MOD(atoi(sCmd.next()), USTEPS_REV);

    // AZ motor
    // Get current position and calculate simmetric position
    pos = MOD(AZ.currentPosition(), USTEPS_REV);
    simPos = MOD(pos + USTEPS_REV/2, USTEPS_REV);
    // Distance to go (positive / negative)
    delta1 = tgt1 - pos;
    // Distance from simmetric position
    delta2 = simPos - tgt1;

    /* Correct delta1 if necessary
    The direction can be specified in the move() method by the sign of the
    relative distance to go. If positive, the motor turns forward, or
    backward otherwhise.
    */
    if (pos < USTEPS_REV/2) {
        // Transform delta to its (negative) simmetric.
        delta1 = (delta2 < 0) ? delta1 - USTEPS_REV : delta1;
    }
    else {
        // Transform delta to its (positive) simmetric.
        delta1 = (delta2 > 0) ? delta1 + USTEPS_REV : delta1;
    }
    // Rotate AZ motor 
    AZ.move(delta1);

    // ALT motor
    // Get current position and calculate simmetric position
    pos = MOD(ALT.currentPosition(), USTEPS_REV);
    simPos = MOD(pos + USTEPS_REV/2, USTEPS_REV);
    // Distance to go (positive / negative)
    delta1 = tgt2 - pos;
    // Distance from simmetric position
    delta2 = simPos - tgt2;

    /* Correct delta1 if necessary
    The direction can be specified in the move() method by the sign of the
    relative distance to go. If positive, the motor turns forward, or
    backward otherwhise.
    */
    if (pos < USTEPS_REV/2) {
        // Transform delta to its (negative) simmetric.
        delta1 = (delta2 < 0) ? delta1 - USTEPS_REV : delta1;
    }
    else {
        // Transform delta to its (positive) simmetric.
        //delta1 = (delta2 > 0) ? MOD(delta1 + USTEPS_REV, USTEPS_REV) : delta1;
        delta1 = (delta2 > 0) ? delta1 + USTEPS_REV : delta1;
    }
    // Rotate ALT motor 
    ALT.move(delta1);

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
    AZ.setMaxSpeed(MAX_SPEED);
    AZ.setAcceleration(ACCEL);
    AZ.setEnablePin(ENABLE);
    
    ALT.setPinsInverted(false, false, true);
    ALT.setMaxSpeed(MAX_SPEED);
    ALT.setAcceleration(ACCEL);
    ALT.setEnablePin(ENABLE);

    sCmd.addCommand("G", ProcessGoto);
    sCmd.addDefaultHandler(Unrecognized);
    
    Serial.begin(BAUDRATE);
}

void loop() {
    sCmd.readSerial();
    AZ.run();
    ALT.run();
    
}
