#include <SerialCommand.h>
#include <EEPROM.h>
#include "sky_pointer.h"

#define VERSION "2.1"

#if MAXSERIALCOMMANDS < 11
#error "Please increase MAXSERIALCOMMANDS to 11 in SerialCommand library"
#endif

SkyPointer sp = SkyPointer();
SerialCommand sCmd;


// Move both motors to an absolute position
void ProcessGoto() {
    int16_t az, alt;

    az = atoi(sCmd.next());
    alt = atoi(sCmd.next());
    sp.goTo(az, alt);
    Serial.print("OK\r");
}

// Move both motors to a relative position
void ProcessMove() {
    int16_t az, alt;

    az = atoi(sCmd.next());
    alt = atoi(sCmd.next());
    sp.move(az, alt);
    Serial.print("OK\r");
}

// Stop the motors
void ProcessStop() {
    sp.stop();
    Serial.print("OK\r");
}

// Move the ALT motor to the home position
void ProcessHome() {
    sp.home();
    Serial.print("OK\r");
}

// Get the current position of the motors
void ProcessGetPos() {
    char buff[13];
    uint16_t az, alt;

    sp.getPos(&az, &alt);
    sprintf(buff, "P %04d %04d\r", az, alt);
    Serial.print(buff);
}

// Get ID
void ProcessId() {
    Serial.print("SkyPointer v"VERSION"\r");
}

// Release both motors and shut down the laser
void ProcessQuit() {
    sp.laser(0);
    sp.releaseMotors();
    Serial.print("OK\r");
}

// Enable or disable the laser
void ProcessLaser() {
    uint8_t enable = atoi(sCmd.next()) != 0;
    sp.laser(enable);
    Serial.print("OK\r");
}

// Set the timeout of the laser pointer in ms
void ProcessTimeout() {
    int32_t t = atoi(sCmd.next());
    sp.setLaserTimeout(t);
    Serial.print("OK\r");
}

// Read a calibration value (4 bytes) from EEPROM
void ProcessReadCalib () {
  uint8_t n = atoi(sCmd.next());
  if (n > 3) {
      Serial.print("NK\r");
      return;
  }
  char buf[12], data[4];
  for (int i = 0; i < 4; i++) {
    data[i] = EEPROM.read(4*n + i);
  }
  sprintf(buf, "R %08lx\r", *(uint32_t *)data);
  Serial.print(buf);
}

// Write a calibration value (4 bytes) to EEPROM
void ProcessWriteCalib () {
  uint8_t n = atoi(sCmd.next());
  if (n > 3) {
      Serial.print("NK\r");
      return;
  }
  char data[4];
  sscanf(sCmd.next(), "%lx", (uint32_t *)data);
  for (int i = 0; i < 4; i++) {
    EEPROM.write(4*n + i, data[i]);
  }
  Serial.print("OK\r");
}

void Unrecognized() {
    Serial.print("NK\r");
}

void setup() {
    // Configure the SkyPointer hardware
    sp.init();

    // Map serial commands to functions
    sCmd.addCommand("G", ProcessGoto);    // G XXXX YYYY\r
    sCmd.addCommand("M", ProcessMove);    // M XXXX YYYY\r
    sCmd.addCommand("P", ProcessGetPos);  // P\r
    sCmd.addCommand("S", ProcessStop);    // S\r
    sCmd.addCommand("H", ProcessHome);    // H\r
    sCmd.addCommand("I", ProcessId);      // I\r
    sCmd.addCommand("Q", ProcessQuit);    // Q\r
    sCmd.addCommand("L", ProcessLaser);   // L enable\r
    sCmd.addCommand("T", ProcessTimeout);   // T XXXX\r
    sCmd.addCommand("R", ProcessReadCalib);  // R N\r
    sCmd.addCommand("W", ProcessWriteCalib); // W N XXXX\r
    sCmd.addDefaultHandler(Unrecognized);

    Serial.begin(BAUDRATE);
}

void loop() {
    sCmd.readSerial();
    sp.run();
}
