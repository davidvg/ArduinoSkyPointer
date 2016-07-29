/*******************************************************************************
Author:    David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
           Juan Menendez Blanco    <juanmb@gmail.com>
Version:   2.0
Date:      2016/May/26

*******************************************************************************/
#include "Arduino.h"

#include "SkyPointer_MotorShield.h"
#include <AccelStepper.h>

// Shield class
MotorShield::MotorShield(void) {
    home = false;
    laserOnTime = 0;
}

void MotorShield::config(void) {
    // Enable pin
    pinMode(ENABLE, OUTPUT); 
    digitalWrite(ENABLE, LOW); 
    // Laser pins
    pinMode(LASER_PIN_H, OUTPUT);
    pinMode(LASER_PIN_L, OUTPUT);
    laser(0);
}

void MotorShield::setTimeOn(uint32_t t) {
    laserOnTime = t;
}

uint32_t MotorShield::getTimeOn(void) {
    return laserOnTime;
}

void MotorShield::laser(uint8_t enable) {
    digitalWrite(LASER_PIN_H, enable);
    digitalWrite(LASER_PIN_L, !enable);
}


/*
// Motor Class
Motor::Motor(uint8_t port_) {
    port = port_;

    target = 0;
    position = 0;
    direction = FW;

    // Configure motor pins
    switch(port) {
        case 0:
            step_pin = XSTEP;
            dir_pin = XDIR;
            break;
        case 1:
            step_pin = YSTEP;
            dir_pin = YDIR;
            break;
        case 2:
            step_pin = ZSTEP;
            dir_pin = ZDIR;
            break;
        default:    // Use X port if any other
            step_pin = XSTEP;
            dir_pin = XDIR;
            break;
    }
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
}

void Motor::setPosition(uint16_t pos) {
    position = pos;
}

uint16_t Motor::getPosition(void) {
    return position;
}

void Motor::setTarget(uint16_t tgt) {
    target = tgt;
    // Guess the direction to rotate and store in the Motor class
    uint16_t sim_pos = MOD(position + USTEPS_REV/2, USTEPS_REV);
    if (position < USTEPS_REV/2) {
        direction = ((target > position) && (target < sim_pos)) ? FW : BW;
    }
    else {
        direction = ((target > position) || (target < sim_pos)) ? FW : BW;
    }
}

uint16_t Motor::getTarget(void) {
    return target;
}

bool Motor::isTarget(void) {
    return (position == target);
}

void Motor::setDirection(uint8_t dir) {
    digitalWrite(dir_pin, dir);
}

uint8_t Motor::getDirection(void) {
    return direction;
}

void Motor::microstep(uint8_t step_dir) {
    // Set direction pin
    setDirection(step_dir);
    // Rotate 1 microstep by sending a HIGH pulse to the STEP pin
    digitalWrite(step_pin, HIGH);
    digitalWrite(step_pin, LOW);
    // Update position calculations
    if (step_dir == FW) {
        position++;
    }
    else {
        if (position == 0){
            position = USTEPS_REV - 1;
        }
        else {
            position--;
        }
    }
    position = MOD(position, USTEPS_REV);
}
*/
