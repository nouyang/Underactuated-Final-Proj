// Simple energy swingup controller
// 12 May 2019

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

#include <Rotary.h>
#include <MegaMotoHB.h>
#include <math.h>

// --------Lever Encoder--------
int val;
volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
Rotary rMotor = Rotary(2, 3); // motor (theta2)
Rotary rStick = Rotary(A5, A4);  // stick (theta1)
int n = LOW;
const byte CPin = 0;  // analog input channel
int CRaw;      // raw A/D value
float CVal;    // adjusted Amps value

// --------Motor--------
int EnablePin = 8;
int duty;
int PWMPin = 11;  // Timer2
int PWMPin2 = 10;
MegaMotoHB motor(11, 10, 8);
int motor_output = 0; // command to motor

// --------P-Controller--------
double theta1 = 0.0; // get_from_encoder()
double theta2 = 0.0; // get_from_encoder()
double prev_theta1 = 0.0; // get_from_encoder()
double prev_theta2 = 0.0; // get_from_encoder()

double theta1dot = 0.0; // get_from_encoder()
double delta_theta1 = 0.0; // get_from_encoder()
double theta2dot = 0.0; // get_from_encoder()
double delta_theta2 = 0.0; // get_from_encoder()

double theta1_desired = 0.0;

unsigned long curr_time;
unsigned long time_elapsed;
unsigned long prev_time;

double state[4];

double k = 5; // P constant

//---- print help
int aprintf(char *str, ...) {
  int i, j, count = 0;

  va_list argv;
  va_start(argv, str);
  for(i = 0, j = 0; str[i] != '\0'; i++) {
    if (str[i] == '%') {
      count++;

      Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);

      switch (str[++i]) {
        case 'd': Serial.print(va_arg(argv, int)); // int  
          break;
        case 'l': Serial.print(va_arg(argv, long)); // long 
          break;
        case 'f': Serial.print(va_arg(argv, double)); // float
          break;
        case 'c': Serial.print((char) va_arg(argv, int)); // char
          break;
        case 's': Serial.print(va_arg(argv, char *)); // string
          break;
        case '%': Serial.print("%");
          break;
        default:;
      };

      j = i+1;
    }
  };
  va_end(argv);

  if(i > j) {
    Serial.write(reinterpret_cast<const uint8_t*>(str+j), i-j);
  }

  return count;
}


void setup() {
    Serial.begin(230400);
    rMotor.begin();
    rStick.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT13) | (1 << PCINT12);
    sei();
    motor.Enable();
}

void loop() {

}


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
        u = maxSpeed; }
    else if (t2dot > maxEncoder) { // spinning too fast
        u = -maxSpeed; }

    else if (t2dot < maxEncoder)  { // spinning too slow
        u = maxSpeed; }

    else {
    // our actual energy-shaping controller
        u = 2 * tauG / M[0,1] + k * t1dot * errorEnergy; }

    u = constrain(u, minMotorSpeed, maxMotorSpeed);
    return u;
}
    
   
// -------- Angle Stuff --------
double getCurrentTheta1() {
    return (double(encoder1Pos) / 1250) * 360;
}

double getCurrentTheta2() { 
    return (double(encoder2Pos) / 1250) * 360;
}


// ---- Set interrupt to read encoder ----

ISR(PCINT2_vect) { // motor, on D2 and D3
    unsigned char result = rMotor.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoder2Pos--;
        //        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoder2Pos++;
        //        Serial.println(getCurrentTheta());
    }
}

ISR(PCINT1_vect) { // stick, on A5 and A4
    unsigned char result = rStick.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoder1Pos--;
        //        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoder1Pos++;
        //        Serial.println(getCurrentTheta());
    }
}
