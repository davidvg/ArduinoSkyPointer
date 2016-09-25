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
#include <AccelStepper.h>

// Speed 
#ifdef DEBUG
    #define MAX_SPEED 100
    #define ACCEL 300
#else
    #define MAX_SPEED 4000
    #define ACCEL 3000
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
#define FW 0            // Forward
#define BW 1            // Backward

// Arduino pins
#define XSTEP 2
#define YSTEP 3
#define ZSTEP 4
#define XDIR 5
#define YDIR 6
#define ZDIR 7
#define ENABLE 8
#define XSTOP 9
#define YSTOP 10
#define ZSTOP 11
// Laser pins
#define LASER_PIN_L 12
#define LASER_PIN_H 13
// Pin for the photo diode
#define PHOTO_PIN A0
// On time for laser
#define LASER_ON_TIME  5000000 // 5 seconds
// Baud Rate
#define BAUDRATE 115200

// Modulus operator
#define MOD(a, b) ((((a) % (b)) + (b)) % (b))


class MotorShield {
    public:
        MotorShield(void);
        uint8_t home;
        void config(void);          // Pin configuration
        void setTimeOn(uint32_t);   // Store elapsed ON time for laser
        uint32_t getTimeOn(void);   // Get elapsed ON time for laser
        void laser(uint8_t);        // Turn ON/OFF the laser
        
    private:
        uint32_t laserOnTime;       // Elapsed ON time for laser
};


/*
class Motor {
    public:
        Motor(uint8_t port_); 
        
        void setPosition(uint16_t);
        uint16_t getPosition(void);

        void setTarget(uint16_t);
        uint16_t getTarget(void);
        bool isTarget();

        void setDirection(uint8_t);
        uint8_t getDirection(void);

        void microstep(uint8_t);

    private:
        uint8_t port;
        uint8_t step_pin;
        uint8_t dir_pin;

        uint16_t position;
        uint16_t target;
        uint8_t direction;          // Direction for rotation. It allows to read
                                    // the direction from the class, or to
                                    // specify it for standalone microstep calls
};
*/

#endif
