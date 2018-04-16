int inByte = 0;         // incoming serial byte
int baud =19200;

/*************************************************************
SETUP
*************************************************************/
void setup()
{
  delay(2000); // Needed to let the robot initialize

  Serial.begin(baud);
  Serial1.begin(baud);

  Serial1.write(128); // This command starts the OI. 
  Serial1.write(131); // set mode to safe (see p.7 of OI manual)

}
/*************************************************************
LOOP
*************************************************************/
void loop()
{
  Serial1.write(142);  // requests the OI to send a packet of sensor data bytes
  Serial1.write(7);  // request cliff sensor value specifically
  delay(250); // poll sensor 40 times a second
  if (Serial1.available() > 0) {
    inByte = Serial1.read();
    Serial.println(inByte);  
  } 
}

