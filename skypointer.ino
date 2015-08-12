/*

*/
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


// DEBUG
uint8_t count = 0;
unsigned long avg = 0;
uint32_t dt = 10000;

// Target position [absolute steps]
uint16_t target[2];
// Desired rotation speed in rpm
uint8_t rpm = 1;
// Bool for controlling rotation status
bool moving = false;

// Motor Parameters
uint8_t steps = 200;
uint8_t usteps = 16;
uint16_t totalMicrosteps = (uint16_t) (steps * usteps); // Needs 16 bit

// Definition of the SerialCommand object, with delimiter ":"
SerialCommand sCmd;
// Definition of the motor shield
SkyPointer_MotorShield MS = SkyPointer_MotorShield ();
// Motor 1 on port 1, 200 steps/rev
SkyPointer_MicroStepper *motor1 = MS.getMicroStepper(steps, 1);
// Motor 2 on port 2, 200 steps/rev
SkyPointer_MicroStepper *motor2 = MS.getMicroStepper(steps, 2);
/******************************************************************************/
// Routine for interruption
void ISR_rotate() {

  // DEBUG
  count++;
  unsigned long t = micros(); // DEBUG  --  Time control per ISR callback

  // Enable interrupts --> Serial, I2C (MotorShield)
  sei();
  //digitalWrite (13 , digitalRead(13) ^ 1);                // DEBUG

  uint16_t pos1, pos2, posSim1, posSim2, t1, t2;   // Define some variables for easy handling
  // Get the current position for both motors
  pos1 = motor1->getPosition();
  pos2 = motor2->getPosition();
  // Calculate simmetric point to current position
  uint16_t delta = 0.5 * totalMicrosteps;
  posSim1 = (pos1 + delta) % totalMicrosteps;
  posSim2 = (pos2 + delta) % totalMicrosteps;
  // Get target for both motors
  t1 = motor1->target;
  t2 = motor2->target;

  // Rotating the motors in the right direction
  // MOTOR 1
  if (!motor1->isTarget()) {  // Check target has not been reached
    if (pos1 < totalMicrosteps / 2) {
      if ((t1 > pos1) && (t1 < posSim1)) {
        motor1->microstep(1, FORWARD);
      }
      else {
        motor1->microstep(1, BACKWARD);
      }
    }
    else {
      if ((t1 > pos1) || (t1 < posSim1)) {
        motor1->microstep(1, FORWARD);
      }
      else {
        motor1->microstep(1, BACKWARD);
      }
    }
  }
  // MOTOR 2
  if (!motor2->isTarget()) {  // Check target has not been reached
    if (pos2 < totalMicrosteps / 2) {
      if ((t2 > pos2) && (t2 < posSim2)) {
        motor2->microstep(1, FORWARD);
      }
      else {
        motor2->microstep(1, BACKWARD);
      }
    }
    else {
      if ((t2 > pos2) || (t2 < posSim2)) {
        motor2->microstep(1, FORWARD);
      }
      else {
        motor2->microstep(1, BACKWARD);
      }
    }
  }

/*
      //Serial.println(totalMicrosteps);
      Serial.print("M1 [pos, target]: [");
      Serial.print(motor1->getPosition()); Serial.print(", ");
      //Serial.print(posSim1); Serial.print(", ");
      Serial.print(motor1->target); Serial.print("]");

      Serial.print(" ## M2 [pos, target]: [" );
      Serial.print(motor2->getPosition()); Serial.print(", ");
      //Serial.print(posSim2); Serial.print(", ");
      Serial.print(motor2->target); Serial.print("] ## [done1, done2]: [");

      Serial.print(motor1->isTarget()); Serial.print(", "); Serial.print(motor2->isTarget());
      Serial.println("]");
*/

/*
  Serial.print("Position: ["); Serial.print(pos1); Serial.print(", ");
  Serial.print(pos2); Serial.print("] # Target: ["); Serial.print(t1);
  Serial.print(", "); Serial.print(t2); Serial.print("] # Done? [");
  Serial.print(motor1->isTarget()); Serial.print(", ");
  Serial.print(motor2->isTarget()); Serial.println("]");

*/
  // DEBUG -- Time control per ISR callback
  //Serial.print("Time per callback [us]: "); Serial.println(micros() - t);
  avg += micros() - t;

  // Check if target is achieved
  // This needs to be changed to invoke a new function to be created.
  // This function must turn the laser on.
  if ((motor1->isTarget()) && (motor2->isTarget())) {
    Timer1.detachInterrupt();

    // DEBUG -- Time control
    Serial.println("Done.");
    Serial.print("Delta t = "); Serial.print(dt);
    Serial.println(" microseconds.");
    Serial.print("Average time per callback ["); Serial.print(count);
    Serial.print(" rotations]: "); Serial.print(avg / count);
    Serial.println(" microseconds.");
    Serial.println("------------------------------------------------------------");
  }
}
/******************************************************************************/
// Functions for processing the commands received
void Goto_wrap (uint16_t* _target) {
  /* Wrapper for processing the GOTO command, as the SerialComand library doesn't
  allow arguments for the functions for processing commands.
  Updates de values of target with the received steps to run.
  */
  int buffer[10]; // Buffer fot incoming data
  unsigned int i = 0;  // Counter for the buffer
  char* arg = sCmd.next(); // Reads the Command (i.e., G, ...)
  while (arg != NULL) {   // Fill the buffer
    buffer[i] = atoi(arg);
    //        Serial.println(buffer[i]);
    arg = sCmd.next();
    i++;
  }
  if (i >= sizeof(_target)) {
    for (unsigned int n = 0; n < sizeof(_target); n++) {
      _target[n] = buffer[n];  // Update target steps values
      _target[n] = _target[n] % totalMicrosteps;
    }
  }
}
void process_Goto () {
  // Calls a wrapper function for updating the target steps.
  // Also passes the values to rotate to the motors configuration


    // DEBUG
    count = 0;
    avg = 0;

  // Call the wrapper for the goto processor
  Goto_wrap (target);
  // Set the targets for the motors
    motor1 -> setTarget(target[0]);
    motor2 -> setTarget(target[1]);
    moving = true;

  // Output for received GOTO
  if (moving) {
    Serial.print("\nNew location received: ["); Serial.print(target[0]);
    Serial.print(", "); Serial.print(target[1]); Serial.println("]");
    Serial.println("Rotating...\n");

    // Configure interrupt speed (microseconds); speed is equal for both motors
    // Located here to allow for changes in speed if needed
    //Timer1.initialize(motor1->getSpeed());
    Timer1.initialize(dt);
    // Enable TimerOne interrupt
    Timer1.attachInterrupt(ISR_rotate);
  }
}

// Handles unknown commands
void unrecognized () {
  Serial.println("What?");
}
/******************************************************************************/
void setup () {
  // Start the serial port
  Serial.begin (115200);
  // Add the commands to the SerialComnnand object
  sCmd.addCommand ("G", process_Goto); // G:steps1:steps2
  // Unknown commands
  sCmd.addDefaultHandler (unrecognized);

  // Start motor shield
  MS.begin();
  motor1->setSpeed(rpm);
  motor2->setSpeed(rpm);

  pinMode(13, OUTPUT);                // DEBUG
  //Timer1.initialize(500000);
  //Timer1.attachInterrupt(ISR_rotate);

  // Feedback
  Serial.println("Waiting for commands...");
}
/******************************************************************************/
void loop () {
  // Read commands from serial port
  sCmd.readSerial();

  //Serial.println(motor1->getPosition());                // DEBUG

  //Serial.println(motor1->target);                // DEBUG
  // Run the motors to the targets defined in the goto processor

  /*
      if (moving == true && m1.distanceToGo() == 0 && m2.distanceToGo() == 0) {
          Serial.println("Location achieved.");
          Serial.println("Waiting for commands...");
          moving = false; // Set moving status to false
      }
  */
}
