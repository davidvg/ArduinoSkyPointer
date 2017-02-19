/*******************************************************************************
Author:    David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
           Juan Menendez Blanco    <juanmb@gmail.com>
Version:   2.0
Date:      2016/May/26

*******************************************************************************/
#include <Arduino.h>
#include "AccelStepper.h"
#include "sky_pointer.h"


// Calculate the relative movement of a motor for going from 'pos' to 'tgt'
// in the shortest time
int16_t calcSteps(uint16_t pos, uint16_t tgt) {
    uint16_t simPos;
    int16_t delta1, delta2;

    // Obtain the simetric position
    simPos = MOD(pos + USTEPS_REV/2, USTEPS_REV);
    // Distance to go (positive / negative)
    delta1 = tgt - pos;
    // Distance from simmetric position
    delta2 = simPos - tgt;

    /* Correct delta1 if necessary
    The direction can be specified in the move() method by the sign of the
    relative distance to go. If positive, the motor turns forward, or
    backward otherwhise.
    */
    if (pos < USTEPS_REV/2) {
        // Transform delta to its (negative) simmetric.
        return (delta2 < 0) ? delta1 - USTEPS_REV : delta1;
    }
    // Transform delta to its (positive) simmetric.
    return (delta2 > 0) ? delta1 + USTEPS_REV : delta1;
}

// Shield class
SkyPointer::SkyPointer(void) :
    azMotor(AccelStepper::DRIVER, XSTEP, XDIR),
    altMotor(AccelStepper::DRIVER, YSTEP, YDIR) {
    absPos = 0;
    laserOnTime = 0;
    homing = false;
    laserTimeout = LASER_TIMEOUT; // Default laser timeout
}

void SkyPointer::init(void) {
    // Enable pin
    pinMode(ENABLE, OUTPUT);
    digitalWrite(ENABLE, LOW);
    // Configure outputs
    pinMode(XSTEP, OUTPUT);
    pinMode(XDIR, OUTPUT);
    pinMode(YSTEP, OUTPUT);
    pinMode(YDIR, OUTPUT);
    // Photo diode pin
    pinMode(PHOTO_PIN, INPUT);
    // Laser pin
    pinMode(LASER_PIN, OUTPUT);

    // Switch laser off at start
    laser(false);

    // Configure axes logic and dinamics
    azMotor.setPinsInverted(false, false, true);
    azMotor.setAcceleration(ACCEL);
    azMotor.setEnablePin(ENABLE);

    altMotor.setPinsInverted(true, false, true);
    altMotor.setAcceleration(ACCEL);
    altMotor.setEnablePin(ENABLE);
}

void SkyPointer::setLaserTimeout(uint32_t t) {
    laserTimeout = t;
}

void SkyPointer::laser(uint8_t enable) {
    // Inverted logic: 0 switches laser on, 1 switches it off
    digitalWrite(LASER_PIN, !enable);
    if (enable) {
        laserOnTime = millis();
    }
}

uint8_t SkyPointer::isLaserOn(void) {
    return !digitalRead(LASER_PIN);
}

void SkyPointer::home() {
    digitalWrite(ENABLE, LOW);
    altMotor.setMaxSpeed(40);
    altMotor.move(-1000);
    homing = true;
}

void SkyPointer::move(int16_t az, int16_t alt) {
    digitalWrite(ENABLE, LOW);
    azMotor.setMaxSpeed(MOVE_SPEED);
    altMotor.setMaxSpeed(MOVE_SPEED);
    // Check movement is in the allowed range
    int16_t aux = absPos + az;
    if ((aux > MAX_RANGE) || (aux < -MAX_RANGE)) {
        az = (az > 0) ? az - USTEPS_REV : az + USTEPS_REV;
    }
    absPos += az;  // Update absolute azimuth
    Serial.println(az);
    azMotor.move(az);
    altMotor.move(alt);
    laser(true);
}

void SkyPointer::goTo(uint16_t az, uint16_t alt) {
    int16_t delta_az, delta_alt;
    uint16_t pos;
    float ratio;

    digitalWrite(ENABLE, LOW);
    // This assumes that max speed is never reached by any motor in order to
    // make a linear motion. For short rotations as the ones done here, it
    // is true.
    // If max speed was to be reached, the maximum speed for any motor should be
    // scaled too by the ratio.
    azMotor.setMaxSpeed(GOTO_SPEED);
    altMotor.setMaxSpeed(GOTO_SPEED);

    // AZ motor
    az = MOD(az, USTEPS_REV);
    pos = MOD(azMotor.currentPosition(), USTEPS_REV);
    delta_az = calcSteps(pos, az);

    // ALT motor
    alt = MOD(alt, USTEPS_REV);
    pos = MOD(altMotor.currentPosition(), USTEPS_REV);
    delta_alt = calcSteps(pos, alt);

    // Check movement is in the allowed range and revert rotation direction
    // if necessary
    int16_t aux = absPos + delta_az;
    if ((aux > MAX_RANGE) || (aux < -MAX_RANGE)) {
        delta_az = (delta_az>0) ? delta_az - USTEPS_REV : delta_az + USTEPS_REV;
    }

    // Assign different accelerations to make a linear motion by making both
    // motors arrive at target at the same time. Speeds are kept proportional.
    if ((delta_az != 0) && (delta_alt!=0)) {
        if (abs(delta_az) >= abs(delta_alt)) {
            ratio = (float)(abs(delta_alt)) / abs(delta_az); // ratio < 1
            azMotor.setAcceleration(ACCEL);
            altMotor.setAcceleration(ACCEL * ratio);
        }
        if (abs(delta_az) < abs(delta_alt)) {
            ratio = (float)(abs(delta_az)) / abs(delta_alt); // ratio < 1
            azMotor.setAcceleration(ACCEL * ratio);
            altMotor.setAcceleration(ACCEL);
        }
    }

    // Update absolute azimuth
    absPos += delta_az;
    Serial.println(az);
    // Move the motors
    azMotor.move(delta_az);
    altMotor.move(delta_alt);
    laser(true);
}

void SkyPointer::getPos(uint16_t *az, uint16_t *alt) {
    *az = MOD(azMotor.currentPosition(), USTEPS_REV);
    *alt = MOD(altMotor.currentPosition(), USTEPS_REV);
}

void SkyPointer::stop() {
    azMotor.stop();
    altMotor.stop();
}

void SkyPointer::releaseMotors() {
    digitalWrite(ENABLE, HIGH);
}

void SkyPointer::run() {
    azMotor.run();
    altMotor.run();

    if (isLaserOn()) {
        // Switch off the laser if timeout has been reached
        if (millis() - laserOnTime > laserTimeout) {
            laser(false);
        }
    } else if (!homing) {
        // Switch on the laser if the motors are running
        if (azMotor.isRunning() || (altMotor.isRunning())) {
            laser(true);
        }
    }

    if (homing && digitalRead(PHOTO_PIN)) {
        altMotor.stop();
        altMotor.setCurrentPosition(0);
        homing = false;
    }
}
