/*******************************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://githun.com/juanmb/skypointer

*******************************************************************************/

#include "SkyPointer_MotorShield.h"

// Motor for Azimut axis
Motor AZ = Motor(X, STEPS);

void setup () {
    AZ.init();
}

void loop () {
    //digitalWrite(XSTEP, HIGH);
    //digitalWrite(XSTEP, LOW);
    AZ.rotate(1);
    delayMicroseconds(100);

}
