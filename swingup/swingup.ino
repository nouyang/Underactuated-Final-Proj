#include <Rotary.h>

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

// --------Lever Encoder--------
int val;
int encoder0Pos = 0;

Rotary r = Rotary(2, 3);
int n = LOW;
const byte CPin = 0;  // analog input channel
int CRaw;      // raw A/D value
float CVal;    // adjusted Amps value

// --------Motor--------
int EnablePin = 8;
int duty;
int PWMPin = 11;  // Timer2
int PWMPin2 = 10;


void setup() {
    Serial.begin (9600);
    pinMode(EnablePin, OUTPUT);     
    pinMode(PWMPin, OUTPUT);
    pinMode(PWMPin2, OUTPUT);
    setPwmFrequency(PWMPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
    r.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    sei();
}
    

void loop() {

}


// TODO: flag to tell motor direction
int swingup_controller(t1, t1dot, t2dot){
    u = 0;
    maxEncoder = 20;
    minEncoder = -20;
    maxMotorSpeed = 255;
    k = 10;

    currentEnergy = 0.5 * M * t1dot**2 + mglcost1
    maxEnergy = 0.5 * M * t1dot**2 * xfixedpoint

    errorEnergy = currentEnergy - maxEnergy 

    if (errorEnergy < maxEnergy/2 and abs(t1dot) < 0.1) {
        // low energy, get out of it
        u = maxSpeed;
    }
    else if (t2dot > maxEncoder) {
        // spinning too fast
        u = -maxSpeed;
    }

    else if (t2dot < maxEncoder)  {
        // spinning too slow
        u = maxSpeed;
    }

    else{
        u = 2 * tauG / M[0,1] + k * t1dot * errorEnergy;
    }

    u = constrain(u, minMotorSpeed, maxMotorSpeed);
    return u;
}
    
}
   

// ---- Set interrupt to read encoder

ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoder0Pos--;
        Serial.println((double(encoder0Pos) / 1250) * 360);
    }
    else if (result == DIR_CCW) {
        encoder0Pos++;
        Serial.println((double(encoder0Pos) / 1250) * 360);
    }
}



// ---- Set clock frequency for motor controller

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) { // Timer0 or Timer1
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) { 
      TCCR0B = TCCR0B & 0b11111000 | mode; // Timer0
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode; // Timer1
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode; // Timer2
  }
}

