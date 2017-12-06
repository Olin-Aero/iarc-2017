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

boolean runSigState = true;
boolean waitSigState = true;
boolean topTouchState = true;

void initRC()
{
  pinMode(targetSwitchPin, INPUT);
  pinMode(runSigPin, INPUT_PULLUP);
  pinMode(waitSigPin, INPUT_PULLUP);
  pinMode(topTouchPin, INPUT_PULLUP);

  digitalWrite(runSigPin, HIGH);
  digitalWrite(waitSigPin, HIGH);
  digitalWrite(topTouchPin, HIGH);
}

boolean isRunSig()
{
  boolean sig = false;
  boolean pin = digitalRead(runSigPin);
  // Only return true if there is a change from low to high
  if(pin && !runSigState)
  {
    sig = true;
  }
  runSigState = pin;
  return sig;
}

boolean isWaitSig()
{
  boolean sig = false;
  boolean pin = digitalRead(waitSigPin);
  // Only return true if there is a change from low to high
  if(pin && !waitSigState)
  {
    sig = true;
  }
  waitSigState = pin;
  return sig;
}

boolean isTopTouch()
{
  boolean sig = false;
  boolean pin = digitalRead(topTouchPin);
  // Only return true if there is a change from high to low
  if(!pin && topTouchState)
  {
    sig = true;
  }
  topTouchState = pin;
  return sig;
}

// Set the Robot's type, Obstacle or Target
//  This is the magic that get all the functionality in one firmware load
void chooseType()
{
  if(digitalRead(targetSwitchPin))
  {
    Serial.println("Choosing Target Profile");
    fsm.transitionTo(TargetWait);
  } 
  else
  {
    Serial.println("Choosing Obstacle Profile");
    fsm.transitionTo(ObstacleWait);
  }
}


