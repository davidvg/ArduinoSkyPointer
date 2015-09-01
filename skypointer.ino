/******************************************************************************
TO-DO
--------------------------
[ ] When target is negative rotation is along the long path
[ ] Rethink the calculation of speed for Timer1 interrupts
[ ] Function for turning laser on
[ ] Feedback to the server with rotated distance for alignment
[X] Implement rotation direction in ISR_rotate
[X] Calculate relative movement in the GOTO processor, equals target - currPos
[X] Function for remapping currPos to [0, 3200] in microstep() function
[X] Ensure arguments from parsing serial are in range [0, 3200]
*/

#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <Wire.h>
#include <SkyPointer_MotorShield.h>
#include <TimerOne.h>

// A modulo operator that handles negative numbers
#define MOD(a,b) ((((a)%(b))+(b))%(b))

// Motor parameters
#define STEPS 200
#define USTEPS 16
#define TOTAL_USTEPS (STEPS*USTEPS)

// Speed parameters
#define DT 10000 // Timer1 interrupt period
#define RPM 1   // Desired rotation speed in rpm

#define LASER_PIN 13


// Definition of the SerialCommand object, with delimiter ":"
SerialCommand sCmd;
// Definition of the motor shield
SkyPointer_MotorShield MS = SkyPointer_MotorShield();
// Motor 1 on port 1, 200 steps/rev
SkyPointer_MicroStepper *motor1 = MS.getMicroStepper(STEPS, 1);
// Motor 2 on port 2, 200 steps/rev
SkyPointer_MicroStepper *motor2 = MS.getMicroStepper(STEPS, 2);


// Interruption routine
void ISR_rotate() {
  uint16_t pos, sim_pos, tg;
  uint8_t dir;

  sei();  // Enable interrupts --> Serial, I2C (MotorShield)

  // MOTOR 1
  pos = motor1->getPosition();
  // Calculate simmetric point to current position
  sim_pos = (pos + TOTAL_USTEPS/2) % TOTAL_USTEPS;
  tg = motor1->target;

  if (!motor1->isTarget()) {  // Check target has not been reached
    if (pos < TOTAL_USTEPS/2) {
      dir = ((tg > pos) && (tg < sim_pos)) ? FORWARD : BACKWARD;
    } else {
      dir = ((tg > pos) || (tg < sim_pos)) ? FORWARD : BACKWARD;
    }
    motor1->microstep(1, dir);
  }

  // MOTOR 2
  pos = motor2->getPosition();
  // Calculate simmetric point to current position
  sim_pos = (pos + TOTAL_USTEPS/2) % TOTAL_USTEPS;
  tg = motor2->target;

  if (!motor2->isTarget()) {  // Check target has not been reached
    if (pos < TOTAL_USTEPS/2) {
      dir = ((tg > pos) && (tg < sim_pos)) ? FORWARD : BACKWARD;
    } else {
      dir = ((tg > pos) || (tg < sim_pos)) ? FORWARD : BACKWARD;
    }
    motor2->microstep(1, dir);
  }

  // Check if target is reached
  // This needs to be changed to invoke a new function to be created.
  // This function must turn the laser on.
  if ((motor1->isTarget()) && (motor2->isTarget())) {
    Timer1.detachInterrupt();
  }
}

/****************************************************************************
 * Functions for processing the commands received
 ***************************************************************************/

// Update the target positions of both motors
void ProcessGoto() {
  uint16_t tgt1, tgt2;

  tgt1 = MOD(atoi(sCmd.next()), TOTAL_USTEPS);
  tgt2 = MOD(atoi(sCmd.next()), TOTAL_USTEPS);

  motor1 -> setTarget(tgt1);
  motor2 -> setTarget(tgt2);

  Serial.print("OK\r");
  Timer1.attachInterrupt(ISR_rotate);  // Enable TimerOne interrupt
}


// Move both motors to a relative position
void ProcessMove() {
  uint16_t tgt1, tgt2;

  tgt1 = MOD((int16_t)motor1->getPosition() + atoi(sCmd.next()), TOTAL_USTEPS);
  tgt2 = MOD((int16_t)motor2->getPosition() + atoi(sCmd.next()), TOTAL_USTEPS);

  motor1 -> setTarget(tgt1);
  motor2 -> setTarget(tgt2);

  Serial.print("OK\r\n");
  Timer1.attachInterrupt(ISR_rotate);  // Enable TimerOne interrupt
}


// Stop both motors
void ProcessStop() {
  motor1 -> setTarget(motor1->getPosition());
  motor2 -> setTarget(motor2->getPosition());

  Serial.print("OK\r");
  Timer1.attachInterrupt(ISR_rotate);  // Enable TimerOne interrupt
}


// Get the current position of the motors
void ProcessGetPos() {
  char buf[13];
  uint16_t tgt1, tgt2;

  tgt1 = motor1->getPosition();
  tgt2 = motor2->getPosition();

  sprintf(buf, "P %04d %04d\r", tgt1, tgt2);
  Serial.print(buf);
}


// Enable/disable the laser module
void ProcessLaser() {
  uint8_t enable;

  enable = atoi(sCmd.next()) != 0;
  digitalWrite(LASER_PIN, enable);
  Serial.print("OK\r");
}


// Handles unknown commands
void Unrecognized() {
  Serial.print("NK\r");
}


void setup() {
  pinMode(LASER_PIN, OUTPUT);

  // Start the serial port
  Serial.begin(115200);

  // Add the commands to the SerialComnnand object
  sCmd.addCommand("G", ProcessGoto);    // G pos1 pos2\r
  sCmd.addCommand("M", ProcessMove);    // M steps1 steps2\r
  sCmd.addCommand("S", ProcessStop);    // S\r
  sCmd.addCommand("P", ProcessGetPos);  // P\r
  sCmd.addCommand("L", ProcessLaser);   // L enable\r
  sCmd.addDefaultHandler(Unrecognized);	// Unknown commands

  // Configure interrupt speed (microseconds)
  Timer1.initialize(DT);

  // Start motor shield
  MS.begin();
  motor1->setSpeed(RPM);
  motor2->setSpeed(RPM);

  Serial.print("Waiting for commands...");
}


void loop() {
  sCmd.readSerial();  // Read commands from serial port
}
