/*******************************************************************************
 * 
 *  Ground Robot Firmware, developed for IARC Mission 7
 *
 *  Original Author: Sterling Lewis Peet <sterling.peet@gatech.edu>
 *  Modified for use with Create 2 by Christian Michelson 
 *  <christian.michelson@gatech.edu>
 * 
 *  All Rights Reserved, please contact the IARC Competition Committee for
 *  permissions requests.
 *
 ******************************************************************************/
 
 #include <SoftwareSerial.h>

void coiSend(int val);
void coiSetBaud(byte baud);
void coiSafeMode();
static boolean isTimeUp(unsigned long *previous, unsigned int *interval);

////////////////////////////////////////////////////////////////////////////////
// iRobot Create Open Interface Opcodes

// Getting Started Opcodes
const byte COI_OP_GS_START = 128;
const byte COI_OP_GS_BAUD = 129;

// Mode Change Opcodes
const byte COI_OP_MODE_SAFE = 131;
const byte COI_OP_MODE_FULL = 132;
const byte COI_OP_MODE_PASSIVE = COI_OP_GS_START;

// Actuator Opcodes
const byte COI_OP_ACT_DRIVE = 137;
const byte COI_OP_ACT_DRIVEDIRECT = 145;
const byte COI_OP_ACT_LED = 139;
const byte COI_OP_ACT_DIGITAL_OUTPUTS = 147;
const byte COI_OP_ACT_PWM_LSD = 144;
const byte COI_OP_ACT_LSD = 138;
const byte COI_OP_ACT_SND_IR = 151;
const byte COI_OP_ACT_SONG = 140;
const byte COI_OP_ACT_PLAY_SONG = 141;

// Input Command Opcodes
const byte COI_OP_IN_SENSORS = 142;
const byte COI_OP_IN_QUERY_LIST = 149;
const byte COI_OP_IN_STREAM = 148;
const byte COI_OP_IN_STREAM_PAUSE = 150;

// Wait Opcodes (mainly for scripting)
const byte COI_OP_WAIT_TIME = 155;
const byte COI_OP_WAIT_DIST = 156;
const byte COI_OP_WAIT_ANGLE = 157;
const byte COI_OP_WAIT_EVENT = 158;

// Baud Rate Codes
const byte COI_BAUD_300    = 0;
const byte COI_BAUD_600    = 1;
const byte COI_BAUD_1200   = 2;
const byte COI_BAUD_2400   = 3;
const byte COI_BAUD_4800   = 4;
const byte COI_BAUD_9600   = 5;
const byte COI_BAUD_14400  = 6;
const byte COI_BAUD_19200  = 7;
const byte COI_BAUD_28800  = 8;
const byte COI_BAUD_38400  = 9;
const byte COI_BAUD_57600  = 10;
const byte COI_BAUD_115200 = 11;

// Timing constants
unsigned int COI_SERIAL_TIMEOUT = 500; // milliseconds
const int COI_START_MESSAGE = 4000; // millisec

// Global Variables
//const long int defaultBaud = 57600;
//const long int defaultBaud = 115200;
const long int defaultBaud = 19200;
unsigned long sciSerialBegin = 0;

////////////////////////////////////////////////////////////////////////////////
// iRobot Create Open Interface Functions

// Initialize the interface
void coiInit()
{
  Serial1.begin(defaultBaud);      // default UART baud rate
  delay(COI_START_MESSAGE);             // Wait for startup message to clear
  Serial1.flush();
  coiSend(COI_OP_GS_START); // Announce our intent to send commands
 //coiSetBaud(COI_BAUD_38400); // Back down the baud rate for reliability
 //Serial1.begin(38400);
 //coiSetBaud(COI_BAUD_115200); // Use default Create 2 Baud Rate
 //Serial1.begin(115200);
 Serial.println("Serial communications opened successfully!");
 coiSafeMode();
 Serial.println("Set to safe mode.  Remember to turn off!");
 
  // Print out the Sketch and Version Info
  Serial.println("IARC Mission 7 Ground Robot Firmware");
  Serial.print("Release Version: ");
  Serial.print(VERSION_MAJOR);
  Serial.print(".");
  Serial.println(VERSION_MINOR);
  
  delay(200);
}

// Drive the Create wheels, set velocity in mm/s
void coiDriveDirect(int right, int left)
{
  coiSend(COI_OP_ACT_DRIVEDIRECT);
  coiSend(right);
  coiSend(left);
}

// Stop the Create wheels!
void coiStopMoving()
{
  coiDriveDirect(0, 0);
}

// Get bump and wheel drop sensor data
byte coiCheckBump()
{
  boolean flag = true;
  byte bump = 0;
  byte bumpCode = 7;
  int whileCount = 0;
  Serial1.flush();
  //coiSend(142);
  // This magic number represents the desired sensor packet number
  //coiSend(7);
  Serial1.write(142); //Hard-coding these for simplicity
  Serial1.write(7);
  sciSerialBegin = millis();

  // Wait for the create to send the byte
  while (flag)
  {
    if(isTimeUp(&sciSerialBegin, &COI_SERIAL_TIMEOUT))
    {
      flag = false;
      Serial.println("WARN: Serial Read Timeout in coiCheckBump()");
    }
    if(Serial1.available()) flag = false;
    whileCount++;
  }

  if(Serial1.available()) {
    bump = Serial1.read();
    Serial.print("Bump: ");
    Serial.println(bump);
  }
  // In case we need to check what the create is sending back
  //Serial.print("Bump Sensor Code: "); 
  //Serial.println(bump, HEX);
  return bump;
}

////////////////////////////////////////////////////////////////////////////////
// iRobot Create Open Utilities

// Change the Baud Rate
void coiSetBaud(byte baud)
{
  coiSend(COI_OP_GS_BAUD);
  coiSend(baud);
  delay(100);  // let interface settle at new rate
}

//Switch to Passive Mode
void coiPassiveMode()
{
  coiSend(COI_OP_MODE_PASSIVE);
}

// Switch to Safe Mode
void coiSafeMode()
{
  coiSend(COI_OP_MODE_SAFE);
  delay(100);
}

// Switch to Full Mode
void coiFullMode()
{
  coiSend(COI_OP_MODE_FULL);
}

// This utility function takes pointers to the previous time something has occured, and the
// desired interval for it to recur.  Returns true if the interval has expired, and updates
// the variable in the previous pointer to the current system time in milliseconds.  Returns
// false if the time interval has not expired.
static boolean isTimeUp(unsigned long *previous, unsigned int *interval)
{
  unsigned long currentTime = millis();
  if(currentTime - *previous > *interval)
  {
    // save the last time back to the location provided by *previous
    *previous = currentTime;
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// iRobot Create Open Interface Helper Functions

// Send the Create a byte
void coiSend(byte val)
{
  Serial1.write(val);
   //Serial.println(val); // DEBUG
}

// Send the Create 2 Bytes
void coiSend(int val)
{
  //Serial1.write(val >> 8); // Send the high byte
  //Serial1.write(val & 0xFF);  // Send the low byte
  Serial1.write(val);
   //Serial.println(val); // DEBUG
}



