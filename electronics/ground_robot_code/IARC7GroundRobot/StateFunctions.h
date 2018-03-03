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
 
// Set to true to force only the maximum noise rather than random
const boolean MAX_NOISE_TEST = false;

const byte randomMax = 64;
// Time between trajectory noise injections
unsigned int noiseInterval = 5000;
unsigned long lastNoise = 0;
// Time between BRC toggle to keep robot awake
unsigned int KeepAwakeInterval = 60000;
unsigned long lastKeepAwake = 0;
// Time between auto-reverse
unsigned int reverseInterval = 20000;
unsigned long lastReverse = 0;
// Time needed to affect trajectory
unsigned int noiseLength = 850;
unsigned long beginNoise = 0;
// Time needed to reverse trajectory
unsigned int reverseLength = 2150; // .33/2 m/s * pi * wheelbase / 2
unsigned long beginReverse = 0;
// Time needed to spin 45 degrees
unsigned int topTouchTime = reverseLength/4;
unsigned long beginTopTouch = 0;

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Entering a State

void obsWaitStart()
{
  Serial.println("State Change: ObstacleWait");
  coiPassiveMode();
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
}

void obsRunStart()
{
  Serial.println("State Change: ObstacleRun");
  coiSafeMode();
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, HIGH);
  // This wheel spped offset gets close to 5m radius circle trajectory, but this
  // value may need to be modified depending on surface friction conditions.
  // Higher values result in a tighter turn radius
  coiDriveDirect(robotSpeed - 30, robotSpeed + 30);
}

void obsCrashStart()
{
  Serial.println("State Change: ObstacleCollision");
  coiStopMoving();
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, HIGH);
}

void trgtWaitStart()
{
  Serial.println("State Change: TargetWait");
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, LOW);
  coiPassiveMode();
}

void trgtRunStart()
{
  Serial.println("State Change: TargetRun");
  coiSafeMode();
  coiDriveDirect(robotSpeed, robotSpeed);
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, HIGH);
  //lastNoise = millis();
  //lastReverse = millis();
}

void vNoiseStart()
{
  Serial.println("State Change: TrajectoryNoise");
  long rand = random(randomMax);
  int offset = (int) rand - randomMax / 2;
  if(MAX_NOISE_TEST)
    offset = 127; // force the maximum noise for testing
  coiDriveDirect(robotSpeed - offset, robotSpeed + offset);
  digitalWrite(greenLed, HIGH);  
  digitalWrite(redLed, LOW);
  beginNoise = millis();
}

void trgtKeepAwakeStart()
{
  Serial.println("Toggling BRC line to keep robot awake");
  digitalWrite(brcPin, LOW);
  Serial.println("BRC low");
  delay(750);
  coiSafeMode();
  digitalWrite(brcPin, HIGH);
  Serial.println("BRC high");
  delay (750);
  coiPassiveMode();
}

void obsKeepAwakeStart()
{
  Serial.println("Toggling BRC line to keep robot awake");
  digitalWrite(brcPin, LOW);
  Serial.println("BRC low");
  delay(750);
  coiSafeMode();
  digitalWrite(brcPin, HIGH);
  Serial.println("BRC high");
  delay (750);
  coiPassiveMode();
}

void vReverseStart()
{
  Serial.println("State Change: Reverse");
  coiDriveDirect(-robotSpeed/2, robotSpeed/2);
  digitalWrite(greenLed, HIGH);  
  digitalWrite(redLed, LOW);
  beginReverse = millis();
}

void trgtCrashStart()
{
  Serial.println("State Change: TargetCollision");
  coiStopMoving();
}

void touchStart()
{
  Serial.println("State Change: TopTouch");
  coiDriveDirect(-robotSpeed/2, robotSpeed/2);
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);  
  beginTopTouch = millis();
}

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Updating a State

void obsWait()
{
  if(isRunSig())
  {
    fsm.transitionTo(ObstacleRun);
  }
  else
  {
      if (isTimeUp(&lastKeepAwake, &KeepAwakeInterval))
      {
        fsm.transitionTo(ObstacleKeepAwake);
      }
      else
      {
        delay(15);
      }
  }
}

void obsRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(ObstacleWait);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(ObstacleCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void obsCrash()
{
  if(isWaitSig())
  {
    fsm.transitionTo(ObstacleWait);
  } 
  else
  {
    byte bump = coiCheckBump();
    if(bump == 0)
    {
      fsm.transitionTo(ObstacleRun);
    } 
    else
    {
      delay(15);
    }
  }
}

void trgtWait()
{
  if(isRunSig())
  {
    fsm.transitionTo(TargetRun);
  }
  else
  {
      if (isTimeUp(&lastKeepAwake, &KeepAwakeInterval))
      {
        fsm.transitionTo(TargetKeepAwake);
      }
      else
      {
        delay(15);
      }
  }
}

void trgtRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  } 
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  } 
  else if (isTimeUp(&lastReverse, &reverseInterval))
  {
    fsm.transitionTo(Reverse);
  }
  else if (isTimeUp(&lastNoise, &noiseInterval))
  {
    fsm.transitionTo(TrajectoryNoise);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void vNoise()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  }  
  else if (isTimeUp(&beginNoise, &noiseLength))
  {
    fsm.transitionTo(TargetRun);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void vReverse()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  } 
  else if (isTimeUp(&beginReverse, &reverseLength))
  {
    fsm.transitionTo(TargetRun);
  }
}

void trgtCrash()
{
  fsm.transitionTo(Reverse);
}

void trgtKeepAwake()
{
  fsm.transitionTo(TargetWait);
}

void obsKeepAwake()
{
  fsm.transitionTo(ObstacleWait);
}

void touch()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTimeUp(&beginTopTouch, &topTouchTime))
  {
    fsm.transitionTo(TargetRun);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }  
}

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Exiting a State

void trgtWaitExit()
{
  Serial.println("Resetting Noise and Reverse timers");
  lastNoise = millis();
  lastReverse = millis();
}

void KeepAwakeExit()
{
  Serial.println("Resetting BRC Keep Awake timer");
  lastKeepAwake = millis();
}

void nullFunc(){ /* Do Nothing, but act as a placeholder */
}


