# SkyPointer

## Description

This project implements the functions and methods needed to control the stepper
motors of the [SkyPointer](https://github.com/juanmb/skypointer) project with
an Arduino UNO and a [CNC Shield](http://blog.protoneer.co.nz/arduino-cnc-shield/).

The Arduino controls two 200 steps/revolution stepper motors (azimuth and
altitude) running at 16 microsteps per step. The azimuth motor is connected to
the X driver, and the altitude motor is connected to the Y driver.

The Z-axis driver in the CNC shield is substituded with a custom board that
controls a green laser pointer and a photodiode. The photodiode is used in the
*homing* process for locating a reference angle of the altitude axis.

The laser pointer can be enabled or disabled using the ZDIR pin of the CNC
shield. The photodiode can be read through the ZSTEP pin.

## Serial protocol

The SkyPointer is controlled with a simple ASCII serial protocol.

All the serial commands start with an uppercase letter and end with a carriage
return `\r`. Some of them require one or two arguments, wich are separated by
spaces.

 * Id: `I\r`. Read the version string.
 * Goto: `G XXXX YYYY\r`. Move both motors to an absolute position in microsteps.
 * Move: ` M XXXX YYYY\r`. Move both motors a relative number of microsteps.
 * GetPos: `P\r`. Get the position of the motors.
 * Stop: ` S\r`. Stop both motors.
 * Home: ` H\r`. Move the altitude axis until the photodiode is triggered.
 * Quit: `Q\r`. Release both motors and switch the laser off.
 * Laser: `L enable\r`. Enable or disable the laser.
 * ReadCalib: `R N\r`. Read the calibration value (32 bit hex integer) at position
   N (from 0 to 3).
 * WriteCalib: `W N XXXX\r`. Write the calibration value at position N.


## Makefile

The code can be compiled and uploaded to the Arduino using `make`.
The provided Makefile requires [Arduino-Makefile](https://github.com/sudar/Arduino-Makefile)
to work. In Debian/Ubuntu/Mint, you can install it with

    sudo apt-get install arduino-mk

## Needed libraries

You will need the following third-party Arduino libraries before compiling this
project:

 * [SerialCommand](https://github.com/scogswell/ArduinoSerialCommand)
 * [AccelStepper](http://www.airspayce.com/mikem/arduino/AccelStepper)
