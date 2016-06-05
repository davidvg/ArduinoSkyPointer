/******************************************************************
Author:     David Vazquez Garcia    <davidvazquez.gijon@gmail.com>
            Juan Menendez Blanco    <juanmb@gmail.com>
Version:    2.0
Date:       2016/May/26

This library is implemented for its use in the SkyPointer project:
    https://github.com/juanmb/skypointer

*******************************************************************************/
#ifndef _SkyPointer_MotorShield_h_
#define _SkyPointer_MotorShield_h_

#include <inttypes.h>

// Speed for develop and normal behaviour
#ifdef DEBUG
    #define DT 100000   // 100 ms
#else
    #define DT 100      // 100 us
#endif

// Naming the axes (ports)
#define X 0
#define Y 1
#define Z 2
// Motor parameters
#define STEPS 200
#define MICROSTEPS 16
#define USTEPS_REV STEPS*MICROSTEPS
// Directions for rotations
#define FW 1            // Forward
#define BW 2            // Backward

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
// Pin for the photo diode
#define PHOTO_PIN A0

// Modulus operator
#define MOD(a, b) ((((a) % (b)) + (b)) % (b))


class MotorShield {
    public:
        MotorShield(void);
        uint8_t home;
        void config(void);   // Pin configuration
        
    //private:
};



class Motor {
    public:
        Motor(uint8_t port_); // Uses the STEPS value
        //friend class MotorShield;

        uint8_t port;
        uint8_t step_pin;
        uint8_t dir_pin;

        void init(void);
        void setPosition(uint16_t);
        uint16_t getPosition(void);
        void setTarget(uint16_t);
        uint16_t getTarget(void);
        bool isTarget();

        void set_direction(uint8_t);
        void microstep(uint8_t);

    //private:
        uint16_t position;
        uint16_t target;

        uint8_t guessDirection(void);

        //MotorShield shield; // MotorShield can now access Motor attributes
};


#endif
