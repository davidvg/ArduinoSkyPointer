/*******************************************************************************
Author:    David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
           Juan Menendez Blanco    <juanmb@gmail.com>
Version:   2.0
Date:      2016/Apr/26

*******************************************************************************/
#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "SkyPointer_MotorShield.h"


void config_shield(void) {
    pinMode(ENABLE, OUTPUT); 
    digitalWrite(ENABLE, LOW); 
    digitalWrite(XDIR, HIGH);
}

// Class for a motor
Motor::Motor(uint8_t port_, uint8_t steps_) {
    uint8_t port = port_;
}

void Motor::init(void) {
    uint8_t step_pin = 0;
    uint8_t dir_pin = 0;
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
//        case 3:
//            step_pin = WSTEP;
//            dir_pin = WDIR;
//            break;
    }
    
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
}

void Motor::set_direction(uint8_t dir) {
    digitalWrite(dir_pin, dir);
}

void Motor::rotate(uint8_t step_dir) {
// Send a HIGH pulse to the port step pin.
digitalWrite(step_pin, HIGH);
    digitalWrite(step_pin, LOW);
}

