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
    home = false;
    laserOnTime = 0;
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
    // Laser pin
    pinMode(LASER_PIN, OUTPUT);
    // Photo diode pin
    pinMode(PHOTO_PIN, INPUT);
    // Switch laser off at start
    laser(0);

    // Configure axes logic and dinamics
    azMotor.setPinsInverted(false, false, true);
    azMotor.setMaxSpeed(MAX_SPEED);
    azMotor.setAcceleration(ACCEL);
    azMotor.setEnablePin(ENABLE);

    altMotor.setPinsInverted(false, false, true);
    altMotor.setMaxSpeed(MAX_SPEED);
    altMotor.setAcceleration(ACCEL);
    altMotor.setEnablePin(ENABLE);
}

void SkyPointer::setTimeOn(uint32_t t) {
    laserOnTime = t;
}

uint32_t SkyPointer::getTimeOn(void) {
    return laserOnTime;
}

void SkyPointer::laser(uint8_t enable) {
    // Inverted logic: 0 switches laser on, 1 switches it off
    digitalWrite(LASER_PIN, !enable);
}

void SkyPointer::move(int16_t az, int16_t alt) {
    digitalWrite(ENABLE, LOW);
    azMotor.move(az);
    altMotor.move(alt);
}

void SkyPointer::goTo(uint16_t az, uint16_t alt) {
    int16_t delta;
    uint16_t pos;

    digitalWrite(ENABLE, LOW);

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
}
