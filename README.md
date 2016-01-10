# SkyPointer MotorShield
## Description
This library implements the functions and methods needed to control the stepper motors of the [SkyPointer](https://github.com/juanmb/skypointer) project with an [Adafruit MotorShield v2 shield](https://www.adafruit.com/products/1438).

It's a variation of the [Adafruit library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library) for DC, servo and stepper motors, and it's strongly based on it

The library focuses on stepper motors, and implements a slightly different method for microstepping.

As in the original Adafruit library, there is a `utility` folder, that includes some necessary code. It must be placed in the same folder than the `.cpp` and `.h` files for the library. In general, the code will be in the ```libraries``` folder of your Arduino installation.

## How to use the library
Here appears the code **specific to this library** to use it. Other necessary code is not shown.

First we are going to include the library:
```C++
#include <SkyPointer_MotorShield.h>
```

Now we define a new `SkyPointer_MotorShield` object and connect a 200 steps-per-revolution stepper motor to its port 1:
```C++
SkyPointer_MotorShield MS = SkyPointer_MotorShield();
SkyPointer_MicroStepper *motor1 =MS.getMicroStepper(200, 1);
```

In ```setup()``` we start the shield:

```C++
void setup () {
  MS.begin();
}
```
In ```loop()``` we can call the ```microstep()``` function to rotate the motor 1 microstep in ```FORWARD``` direction:
```C++
void loop () {
  motor1 -> microstep (1, FORWARD);
  delay (200); // 200 ms delay for aprox. speed of 5 microsteps / second
}
```
In this case, a ```delay()``` is used, but a better time control should be used.
