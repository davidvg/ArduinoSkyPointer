# SkyPointer MotorShield

**Note on the version:** currently a v2 version of the library is on development. This new version uses a different motor driver and the code is not compatible with the one from v1. This new version allows faster rotations and accelerations that make the rotation smoother.


## Description
This library implements the functions and methods needed to control the stepper motors of the [SkyPointer](https://github.com/juanmb/skypointer) project with an [Adafruit MotorShield v2 shield](https://www.adafruit.com/products/1438).

It's a variation of the [Adafruit library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library) for DC, servo and stepper motors, and it's strongly based on it.

The library focuses on stepper motors, and implements a slightly different method for microstepping. It uses 16 microsteps per step, but other values can be easily implemented.

As in the original Adafruit library, there is a `utility` folder, that includes some necessary code. It must be placed in the same folder than the `.cpp` and `.h` files for the library. In general, the code will be in the ```libraries``` folder of your Arduino installation.




## How to use the library
Here appears the code **specific to this library** to use it. Other necessary code is not shown.

#### Defining the elements

First thing to do is to include the library in the Arduino sketch:
```C++
#include <SkyPointer_MotorShield.h>
```

Now we define a new `SkyPointer_MotorShield`, a shield object, and connect a 200 steps-per-revolution stepper motor to its port 1:
```C++
SkyPointer_MotorShield MS = SkyPointer_MotorShield();
SkyPointer_MicroStepper *motor1 =MS.getMicroStepper(200, 1);
```

The library works with 16 microsteps per step.

In ```setup()``` we start the shield using the `begin()` method, which internally configures the driver's pins and frequencies:

```C++
void setup () {
	...
	MS.begin();
	...
}
```


#### Making it work

We can set a target position calling the `setTarget()` method:

```C++
motor -> setTarget(1000); // Set the motor to rotate to (absolute) microstep 1000
```

The current target position can be retrieved calling the class member `motor.target`. Also, we can check if the target has been reached with the `motor -> isTarget()` method, which returns `true` if the rotation is done and the motor is at its final position.

The rotation is done calling the ```microstep()``` function:
```C++
void loop () {
  motor1 -> microstep (1, FORWARD);
  delay (200); // 200 ms delay for aprox. speed of 5 microsteps / second
}
```

The rotation speed is controlled by calling the `microstep` member periodically. In this case, a ```delay()``` is used, but a better time control should be used. In the case of the SkyPointer project, speed temporization is done using the [Timer1](http://playground.arduino.cc/Code/Timer1) library after reading the target positions sent via Serial port using the [SerialCommand](https://github.com/scogswell/ArduinoSerialCommand) library.

Due to the limitations of the shield, the maximum speed that can be achieved is quite slow.

The current motor position can be accesed at any moment calling the `getPosition()` member:

```C++
curr_pos = motor -> getPosition();
```

#### More features...

The SkyPointer for which this library was written also has a laser that must be turned on and off. This can be done calling the `laser()` method, which takes as argument a `1` (`true`) to turn the laser on and a `0` (`false`) to turn it off. This can be used to handle any other hardware that meets the same specifications (more on this can be found in the code)
