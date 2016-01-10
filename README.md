# SkyPointer MotorShield
## Description
This library implements the functions and methods needed to control the stepper motors of the [SkyPointer](https://github.com/juanmb/skypointer) project with an [Adafruit MotorShield v2 shield](https://www.adafruit.com/products/1438).

It's a variation of the [Adafruit library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library) for DC, servo and stepper motors, and it's strongly based on it

The library focuses on stepper motors, and implements a slightly different method for microstepping.

As in the original Adafruit library, there is a `utility` folder, that includes some necessary code. It must be placed in the same folder than the `.cpp` and `.h` files for the library.
