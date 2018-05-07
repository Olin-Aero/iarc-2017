/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  // print out the value you read:
  String output =  String(analogRead(A0)) + " " 
  + String(analogRead(A1)) + " " 
  + String(analogRead(A2)) + " " 
  + String(analogRead(A3)) + " " 
  + String(analogRead(A4)) + " " 
  + String(analogRead(A5)); 
  Serial.println(output);
  delay(1);        // delay in between reads for stability
}
