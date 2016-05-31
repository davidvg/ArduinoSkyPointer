/*******************************************************************************
Author:    David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
           Juan Menendez Blanco    <juanmb@gmail.com>
Version:   2.0
Date:      2016/May/26

*******************************************************************************/
#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "SkyPointer_MotorShield.h"

// Shield class
MotorShield::MotorShield(void) {
    home = false;
}

void MotorShield::config(void) {
    pinMode(ENABLE, OUTPUT); 
    digitalWrite(ENABLE, LOW); 
}


// Motor Class
Motor::Motor(uint8_t port_) {
    port = port_;
    step_pin = 0;
    dir_pin = 0;

    target = 0;
    currPosition = 0;

    // Configure motor pins
    init();
}

void Motor::init(void) {
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
    }
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
}

void Motor::setPosition(uint16_t pos) {
    currPosition = pos;
}

uint16_t Motor::getPosition(void) {
    return currPosition;
}

void Motor::setTarget(uint16_t tgt) {
    target = tgt;
}

uint16_t Motor::getTarget(void) {
    return target;
}

bool Motor::isTarget(void) {
    return (currPosition == target);
}

void Motor::set_direction(uint8_t dir) {
    digitalWrite(dir_pin, dir);
}

uint8_t Motor::guessDirection(void) {
    uint8_t dir;
    // Simetric position
    uint16_t sim_pos = (currPosition + USTEPS_REV/2) % USTEPS_REV;
    if (!isTarget()) {
        dir = ((target > currPosition) && (target < sim_pos)) ? FW : BW;
    }
    else {
        dir = ((target > currPosition) || (target < sim_pos)) ? FW : BW;
    }
    return dir;
}

void Motor::rotate(uint8_t step_dir) {
// Send a HIGH pulse to the port step pin.
    if (step_dir == 0) {     // Forward
        currPosition++;
        // Check range 
        currPosition += USTEPS_REV;
        currPosition %= USTEPS_REV;
    }
    else {
        if (currPosition == 0){
            currPosition = USTEPS_REV - 1;
        }
        else {
            currPosition--;
        }
        // Check range again
        currPosition += USTEPS_REV;
        currPosition %= USTEPS_REV;
    }
    // Perform rotation
    digitalWrite(step_pin, HIGH);
    digitalWrite(step_pin, LOW);
}
