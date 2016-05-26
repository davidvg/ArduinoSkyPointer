/******************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://githun.com/juanmb/skypointer

*******************************************************************************/
#ifndef _SkyPointer_MotorShield_h_
#define _SkyPointer_MotorShield_h_

#include <inttypes.h>

// Naming the axes (ports)
#define X 0
#define Y 1
#define Z 2
#define W 3
// Motor parameters
#define STEPS 200
#define MICROSTEPS 16
// Directions for rotations
#define FW 0
#define BW 1

// Arduino pins
#define XSTEP 2
#define YSTEP 3
#define ZSTEP 4
#define XDIR 5
#define YDIR 6
#define ZDIR 7
#define ENABLE 8
// Laser pins
#define LASER_PIN_L 12
#define LASER_PIN_H 13


void config_shield(void);   // Pin configuration

class Motor {
    public:
        Motor(uint8_t port_, uint8_t steps_);
        uint8_t port;
        uint8_t step_pin;
        uint8_t dir_pin;

        void init(void);
        void set_direction(uint8_t dir);
        void rotate(uint8_t dir);

    private:

};


#endif
