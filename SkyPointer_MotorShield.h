/******************************************************************






 ******************************************************************/

/*
TO-DO
[ ] Custom number of microsteps per step, 8, 16, 32, 64...
[X] currStep is uint16_t and always increases -> constrain to [0, 3199]
*/
    


#ifndef _SkyPointer_MotorShield_h_
#define _SkyPointer_MotorShield_h_

#include <inttypes.h>
#include <Wire.h>
#include "utility/Adafruit_PWMServoDriver.h"

//#define MOTORDEBUG

#define MICROSTEPS 16         // 8 or 16

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

class SkyPointer_MotorShield;
///////////////////////////////////////
// New class for Microstepper motor
class SkyPointer_MicroStepper {
    public:
        SkyPointer_MicroStepper (void);
        friend class SkyPointer_MotorShield;
        // Parameters
        uint16_t target;
        uint32_t usecPerMicrostep;
        uint16_t currPos;
        //uint32_t steppingcounter;   // Not needed
        // Function members
        uint16_t getPosition (); // Returns current position in microsteps
        uint16_t microstep (uint16_t usteps, uint8_t dir);  // Rotates microsteps
        void setSpeed (float);   // Calculates interval in microseconds
                                    // from one call of microstep to the next
        void release (void);        // Frees the motor
        void setTarget (uint16_t);  // Sets the value of the target for the motor
        boolean isTarget(void);     // Checks if currPos == target
        // Debug
        uint32_t getSpeed();
        
        
    private:
        // Parameters
        uint8_t steppernum;
        uint16_t microstepsPerRev; // 'Whole' steps per revolution
        uint16_t currMicrostep;
        // Function members
        uint8_t PWMApin, AIN1pin, AIN2pin;
        uint8_t PWMBpin, BIN1pin, BIN2pin;
        
        SkyPointer_MotorShield *MC;
        
};


///////////////////////////////////////
class SkyPointer_StepperMotor {
 public:
  SkyPointer_StepperMotor(void);
  friend class SkyPointer_MotorShield;

  void step(uint16_t steps, uint8_t dir,  uint8_t style = SINGLE);
  
  void setSpeed(uint16_t);
  uint8_t onestep(uint8_t dir, uint8_t style);
  void release(void);
  uint32_t usperstep, steppingcounter;

 private:
  uint8_t PWMApin, AIN1pin, AIN2pin;
  uint8_t PWMBpin, BIN1pin, BIN2pin;
  uint16_t revsteps; // # steps per revolution
  uint8_t currentstep;
  SkyPointer_MotorShield *MC;
  uint8_t steppernum;
};

class SkyPointer_MotorShield
{
  public:
    SkyPointer_MotorShield(uint8_t addr = 0x60);

    void begin(uint16_t freq = 1600);

    void setPWM(uint8_t pin, uint16_t val);
    void setPin(uint8_t pin, boolean val);

    // Attach stepper motor in microstepper mode
    SkyPointer_MicroStepper *getMicroStepper(uint16_t steps, uint8_t num);
    // Attach stepper motor in normal mode
    SkyPointer_StepperMotor *getStepper(uint16_t steps, uint8_t n);
 private:
    uint8_t _addr;
    uint16_t _freq;

    SkyPointer_StepperMotor steppers[2];
    // Added...
    SkyPointer_MicroStepper microsteppers[2];
    
    Adafruit_PWMServoDriver _pwm;
};

#endif
