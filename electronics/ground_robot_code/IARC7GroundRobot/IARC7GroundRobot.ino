/*******************************************************************************
 * 
 *  Ground Robot Firmware, developed for IARC Mission 7
 *
 *  Original Author: Sterling Lewis Peet <sterling.peet@gatech.edu>
 *  Modified for use with Create 2 by Christian Michelson 
 *  <christian.michelson@gatech.edu>
 * 
 *  All Rights Reser\]ed, please contact the IARC Competition Committee for
 *  permissions requests.
 *
 ******************************************************************************/

#include <FiniteStateMachine.h>

////////////////////////////////////////////////////////////////////////////////
// Version Information
const byte VERSION_MAJOR = 2;
const byte VERSION_MINOR = 2;

////////////////////////////////////////////////////////////////////////////////
// Hardware Constants

//const byte rxPin = 0;
//const byte txPin = 1;
const byte brcPin = 6;
long last_minute = 0;
long minute = 0;

const byte targetSwitchPin = 7;
const byte runSigPin = 8;
const byte waitSigPin = 2;
const byte topTouchPin = 3;

const byte greenLed = 9;
const byte redLed = 10;

// Design Configuration Constants
const int robotSpeed = 330; // mm/s


// Global Variables

void chooseType();
void obsWaitStart();
void obsWait();
void obsRunStart();
void obsCrashStart();
void trgtWaitStart();
void touchStart();
void touch();
void nullFunc();
void obsRunStart(); void obsRun();
void obsCrashStart(); void obsCrash();
void trgtWaitStart(); void trgtWait();
void coiStopMoving();
void KeepAwakeExit(); void trgtWaitExit();
void trgtRunStart();
void obsKeepAwake(); void trgtRun(); void obsKeepAwakeStart();
void vNoiseStart(); void trgtKeepAwake();
void vNoise(); void trgtKeepAwakeStart();
void vReverseStart(); void trgtCrash();
void vReverse(); void trgtCrashStart();

// Robot Behavior State Machine
State Start =             State(chooseType);
State ObstacleWait =      State(obsWaitStart, obsWait, nullFunc);
State ObstacleRun =       State(obsRunStart, obsRun, coiStopMoving);
State ObstacleCollision = State(obsCrashStart, obsCrash, nullFunc);
State TargetWait =        State(trgtWaitStart, trgtWait, trgtWaitExit);
State TargetRun =         State(trgtRunStart, trgtRun, nullFunc);
State TrajectoryNoise =   State(vNoiseStart, vNoise, nullFunc);
State Reverse =           State(vReverseStart, vReverse, nullFunc);
State TargetCollision =   State(trgtCrashStart, trgtCrash, nullFunc);
State TopTouch =          State(touchStart, touch, nullFunc);
State TargetKeepAwake =   State(trgtKeepAwakeStart, trgtKeepAwake, KeepAwakeExit);
State ObstacleKeepAwake = State(obsKeepAwakeStart, obsKeepAwake, KeepAwakeExit);

FSM fsm = FSM(Start);     // Initialize state machine, set start state

#include "iRobotCreate.h"
#include "RemoteControl.h"
#include "StateFunctions.h"

void setup() {
  //Serial.begin(115200);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  
  initRC();

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  // Indicate initialization mode
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, HIGH);

  coiInit();
  coiSafeMode();
  delay(200);

  // Stop indication
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, LOW);
}

void loop() {
  fsm.update();
}




