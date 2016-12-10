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

    // Obtain the simmetric position
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
    azMotor.move(az);
    altMotor.move(alt);
    laser(true);
}

void SkyPointer::goTo(uint16_t az, uint16_t alt) {
    int16_t delta;
    uint16_t pos;

    digitalWrite(ENABLE, LOW);
    azMotor.setMaxSpeed(GOTO_SPEED);
    altMotor.setMaxSpeed(GOTO_SPEED);

    // AZ motor
    az = MOD(az, USTEPS_REV);
    pos = MOD(azMotor.currentPosition(), USTEPS_REV);
    delta = calcSteps(pos, az);
    azMotor.move(delta);

    // ALT motor
    alt = MOD(alt, USTEPS_REV);
    pos = MOD(altMotor.currentPosition(), USTEPS_REV);
    delta = calcSteps(pos, alt);
    altMotor.move(delta);
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
