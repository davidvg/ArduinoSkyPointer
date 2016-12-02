#include <SerialCommand.h>
#include "SkyPointer_MotorShield.h"

MotorShield shield = MotorShield();

AccelStepper AZ(AccelStepper::DRIVER, XSTEP, XDIR);
AccelStepper ALT(AccelStepper::DRIVER, YSTEP, YDIR);

SerialCommand sCmd;

// Functions for SerialCommand
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

void ProcessMove() {
    int16_t tgt1, tgt2;
    tgt1 = atoi(sCmd.next());
    tgt2 = atoi(sCmd.next());

    AZ.move(tgt1);
    ALT.move(tgt2);
    Serial.print("OK\r");

}

void ProcessGetPos() {
    char buff[13];
    uint16_t pos1, pos2;
    pos1 = MOD(AZ.currentPosition(), USTEPS_REV);
    pos2 = MOD(ALT.currentPosition(), USTEPS_REV);
    sprintf(buff, "P %04d %04d\r", pos1, pos2);
    Serial.print(buff);
}

void ProcessLaser() {
    uint8_t enable = atoi(sCmd.next()) != 0;
    shield.laser(enable);
    Serial.print("OK\r");
}

void Unrecognized() {
    Serial.print("NK\r");
}

void setup() {
    // Configure pins and turn laser off
    shield.init();

    // Configure axes logic and dinamics 
    AZ.setPinsInverted(false, false, true);
    AZ.setMaxSpeed(MAX_SPEED);
    AZ.setAcceleration(ACCEL);
    AZ.setEnablePin(ENABLE);
    
    ALT.setPinsInverted(false, false, true);
    ALT.setMaxSpeed(MAX_SPEED);
    ALT.setAcceleration(ACCEL);
    ALT.setEnablePin(ENABLE);

    // SerialCommand
    sCmd.addCommand("G", ProcessGoto);    // G XXXX YYYY\r
    sCmd.addCommand("M", ProcessMove);    // M XXXX YYYY\r
    sCmd.addCommand("P", ProcessGetPos);  // P\r
    sCmd.addCommand("L", ProcessLaser);   // L enable\r
    sCmd.addDefaultHandler(Unrecognized);
    
    Serial.begin(BAUDRATE);
}

void loop() {
    sCmd.readSerial();
    AZ.run();
    ALT.run();
    //Serial.println(MOD(AZ.currentPosition(), USTEPS_REV), DEC);    
}
