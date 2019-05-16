/********************************************//**
 * \file     H-Bridge.pde
 * \author   Alexander Hogen
 *           https://github.com/ahogen/MegaMoto
 * \date     1/3/2016
 * \version  0.1
 * \brief    A basic example showing how to get
 *           started with the MegaMoto Arduino
 *           library.
 *
 * The "MegaMogo" name is trademark of Robot Power.
 * You can find and purchase their hardware by
 * visiting their website: www.robotpower.com
 * I have been granted written permission by
 * Robot Power to use this name in the library and
 * associated code as long as I point people to
 * their company name and website. I encourage
 * you to go check out their stuff! 
 *
 * This project demonstrates how to include and 
 * instantiate a new MegaMoto object with the 
 * chosen pinassignments and then start up the 
 * motor/device in various speeds and directions.
 *
 *
 * [1] "MegaMoto & MegaMoto Plus User Manual," Robot Power,
 *     Version 1.6, May 28, 2016.
 *     Avaliable: http://www.robotpower.com/downloads/MegaMoto-user-manual.pdf
 ***********************************************/

// Include the MegaMoto library
#include <MegaMotoHB.h>

// Instantiate a (global) MegaMoto object.
// For this example, it is assumed that your jumpers are connected in
// the following fashion. Please read page 3 in the MegaMoto user
// manual to understand your options. ALSO, the motor itself is wired
// in an H-bridge fashion. Please see Figure 4 on page 5 of the user
// manual to understand what this means.
// 
// -- Jumper configuration --
// PWM A ---> D11
// PWM B ---> D3
// Enable --> 5V
// Sensor --> (none. remove jumper)

// Format is:  <yourObjectName>( <pin_for_pwm_a>, <pin_for_pwm_b> );
/*MegaMotoHB motor(11, 10);*/

// If you *do* want to use the Enable line, instantiate your motor
// object like this...
// Format is:  <yourObjectName>( <pin_for_pwm_a>, <pin_for_pwm_b>, <pin_for_enable> );

MegaMotoHB motor(11, 10, 8);

const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to
const int buttonPin = 7;     // the number of the pushbutton pin
const int buttonStopPin = 6;     // the number of the pushbutton pin
const int buttonEStopPin = 5;     // the number of the pushbutton pin

const int ledPin =  12;      // the number of the LED pin
const int ledFwdPin =  9;      // the number of the LED pin

int buttonState = 0;         // variable for reading the pushbutton status
int buttonStopState = 0;         // variable for reading the pushbutton status
//int buttonEStopState = 0;         // variable for reading the pushbutton status
int EStopFlag = 0;
int lastButtonState = LOW;   // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() 
{
	// Initialize the serial port
	Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(ledFwdPin, OUTPUT);

  pinMode(buttonPin, INPUT);
  pinMode(buttonStopPin, INPUT);


	// Print a message indicating the code is running
	Serial.println("Basic MegaMoto test is running...");
	
	// Enable the MegaMoto controller, if you *did* choose to
	// control the Enable pin. If you didn't use the Enable pin,
	// this function won't do anything. It's just wasting your
	// CPU time, so comment it out! ;-)
	motor.Enable();

}


void loop()
{
    // put your main code here, to run repeatedly:

    // Move forward at full speed
    buttonState = digitalRead(buttonPin);
    buttonStopState = digitalRead(buttonStopPin);
    int buttonEStopState = digitalRead(buttonEStopPin);
    digitalWrite(ledPin, LOW);
    digitalWrite(ledFwdPin, LOW);

  if (buttonEStopState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonEStopState == HIGH) {
      EStopFlag = not EStopFlag;
      digitalWrite(ledPin, HIGH);
    }
    }
    else if (not EStopFlag) {
      if (buttonStopState == HIGH){
        motor.Stop();
      }
      else {
        motor.Enable();
        if (buttonState == HIGH) {
          motor.Fwd(255);
          digitalWrite(ledFwdPin, HIGH);
        }
        else if (buttonState == LOW) {
        // Move backward at half speed
        motor.Rev(255);
        }
      }
      
    }
    else if (EStopFlag) {
      motor.Stop();
    
    }
    lastButtonState = buttonEStopState;


}
