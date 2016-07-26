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
#include <AccelStepper.h>

AccelStepper motor(1, XSTEP, XDIR);

uint8_t pos = 2000;

void setup () {
    motor.setPinsInverted(false, false, true);
    motor.setEnablePin(ENABLE);
    motor.setMaxSpeed(5000);
    motor.setAcceleration(3000);
    motor.moveTo(6400);
}

void loop () {
    /*
    if (motor.distanceToGo() == 0) {
        delay(5);
        pos = -pos;
        motor.moveTo(pos);
    }
    */
    motor.run();
}
