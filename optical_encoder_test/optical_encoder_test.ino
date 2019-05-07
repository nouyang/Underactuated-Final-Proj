//#include <KeyboardController.h>
int val;
int encoder0PinA = 8;
int encoder0PinB = 9;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;

//USBHost usb;

// Attach Keyboard controller to USB
//KeyboardController keyboard(usb);


void setup() {
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  Serial.begin (9600);
}

void loop() {
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos--;
      t
    } else {
      encoder0Pos++;
    }
    //    Serial.print (encoder0Pos);
    Serial.print((double(encoder0Pos) / 1250) * 360 * 5);
    Serial.print ("\n");
    Serial.print ("/");
  }
  encoder0PinALast = n;
}
//
//void keyPressed() {
//  Serial.print(keyboard.getKey());
//}
