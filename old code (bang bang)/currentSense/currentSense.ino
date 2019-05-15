/*
  Analog input, analog output, serial output
*/

// These constants won't change. They're used to give names to the pins used:
#include <MegaMotoHB.h>
#include <math.h>
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

MegaMotoHB motor(11, 10, 8);
int counter = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
motor.Enable();
motor.SetStepDelay(0);
}

void loop() {

  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  // change the analog out value:
    motor.Fwd(100);
    /*delay(800);*/
    /*motor.Rev(100);*/
    /*delay(800);*/

  // print the results to the Serial Monitor:
  Serial.print("sensor = ");
  Serial.println(sensorValue);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}
