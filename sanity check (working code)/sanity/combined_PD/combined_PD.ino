// Sanity check that encoder works
// 11 May 2019

#include <Rotary.h>
#include <MegaMotoHB.h>
#include <math.h>

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

// --------Lever Encoder--------
int val;
volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
Rotary rMotor = Rotary(2, 3); // motor (theta2)
Rotary rStick = Rotary(A5, A4);  // stick (theta1)
int n = LOW;
/*const byte CPin = 0;  // analog input channel*/
/*int CRaw;      // raw A/D value*/
/*float CVal;    // adjusted Amps value*/

// --------Motor--------
int EnablePin = 8;
int duty;
int PWMPin = 11;  // Timer2
int PWMPin2 = 10;
MegaMotoHB motor(11, 10, 8);
int motor_output = 0; // command to motor

// --------P-Controller--------
int sample_time = 0; // 15 msec

double theta1 = 0.0; // get_from_encoder()
double theta2 = 0.0; // get_from_encoder()
double prev_theta1 = 0.0; // get_from_encoder()
double prev_theta2 = 0.0; // get_from_encoder()

double theta1dot = 0.0; // get_from_encoder()
double delta_theta1 = 0.0; // get_from_encoder()
double theta2dot = 0.0; // get_from_encoder()
double delta_theta2 = 0.0; // get_from_encoder()

double theta1_desired = 0.0;
double theta1dot_desired = 0.0;

double err_theta = 0.0;
double err_thetadot = 0.0;

int delta_motor = 0;
int prev_motor = 0;

bool theta_CW;
bool motor_CW;

unsigned long now = 0;
unsigned long time_elapsed;
unsigned long prev_time = 0;

double state[4];

double k = 4; // theta constant 
double kdot = -1000; // thetadot 

void setup() {
    /*Serial.begin(230400);*/
    Serial.begin(115200); // for use with plotter tool 

    rMotor.begin();
    rStick.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT13) | (1 << PCINT12);
    sei();

    /*motorOn();*/
    motor.Enable();
    motor.SetStepDelay(0);
}

void loop(){
    // -------- update time --------
    unsigned long now = millis();
    time_elapsed = (now - prev_time);
    if (time_elapsed >= sample_time)
    {

    // -------- update theta --------
    // issue: prev_theta is the same as theta
    
    theta2 = getCurrentTheta2();
    theta1 = getCurrentTheta1();
    delta_theta2 = prev_theta2 - theta2;
    delta_theta1 = prev_theta1 - theta1;
    theta1dot = delta_theta1 / time_elapsed;
    theta2dot = delta_theta2 / time_elapsed;
    prev_theta2 = theta2;
    prev_theta1 = theta1;
    state[0] = theta1;
    state[1] = theta2;
    state[2] = theta1dot;
    state[3] = theta2dot;

    // -------- determine motor input --------
    /*Serial.println(motor_speed);*/
    
    err_theta = theta1 - theta1_desired; 
    err_thetadot = theta1dot - theta1dot_desired;
    motor_output = - ceil( k * (theta1 - theta1_desired) - kdot * (theta1dot - theta1dot_desired));
    delta_motor = motor_output - prev_motor;
    prev_motor = motor_output;
//    Serial.print(motor_output);
    /*aprintf("\ntheta1 %f, t2 %f, t1dot %f, t2dot %f, out %d, deltath %f, cw ", */
            /*theta1, theta2, theta1dot, theta2dot, motor_output, delta_theta1);*/
    /*aprintf("\n %d %f %d %f %f ", delta_motor, theta1, motor_output, err_theta, err_thetadot);*/
    /*aprintf("\n %f %f %f %f ", theta1, err_theta, theta1dot, err_thetadot);*/
    /*aprintf("\n %f %f ", theta1dot, err_thetadot);*/
    aprintf("\n %f %f %d", theta1, err_theta, motor_output);
    /*Serial.println(theta1);*/
    /*Serial.print(delta_motor);*/
    
    // -------- write appropriate motor input --------
    motor_output  = constrain(motor_output, -255, 255);
    
    if (abs(theta1) > 10){
        if (motor_output > 0){
            motor.Fwd(abs(motor_output));
        }
        else {
            motor.Rev(abs(motor_output));
        }
    }
    else {
        Serial.println("stop");
        motor.Stop();
    }
/*
    motor.Fwd(200);
    Serial.println("Fwd");
    delay(500);
    motor.Rev(200);
    Serial.println("Rev");
    delay(500);
    motor.Stop();
    Serial.println("Stop");
    delay(1000);
    */


    /*motorCounterCW(200); */
    /*motor.Rev(100); // keeps going */
    /*delay(500);*/
    /*motor.Stop();*/
    /*delay(500);*/
//    if (abs(theta1) < 2.0) {
/*//    }*/
    prev_time = now;
    }
}

// --------- Helper Functions -------

// -------- Motor Funcs --------
void motorOn(){
    digitalWrite(EnablePin, HIGH);
    analogWrite(PWMPin2, 0);
// requires continuously writing it as fast as possible
}
void motorCW(int input_speed) {
    PWMPin = 10;
    PWMPin2 = 11;
    motorWrite(input_speed);
    motor_CW = true;
}
void motorCounterCW(int input_speed) {
    PWMPin = 11;
    PWMPin2 = 10;
    motorWrite(input_speed);
    motor_CW = false;
}
void motorWrite(int input_speed) {
    for(duty = 0; duty <= input_speed; duty += 50){
        analogWrite(PWMPin, duty);
        delay(5);
    }
  analogWrite(PWMPin, input_speed);
}
    
// -------- Angle Conversion --------
double getCurrentTheta1() { // calibration for stick encoder = 1250
    double val = (double(encoder1Pos) / 1250) * 360;
    val = fmod(val, 360);
    if (val > 180) {
        val = val - 360;
    }
    return val;
}

double getCurrentTheta2() {  // 500 ticks / rev, for motor encoder
    double val = (double(encoder2Pos) / 500 ) * 360;
    val = fmod(val, 360);
    if (val > 180) {
        val = val - 360;
    }
    return val;
}


// ---- Set interrupt to read encoder ----

ISR(PCINT2_vect) { // motor, on D2 and D3
    unsigned char result = rMotor.process();
    if (result == DIR_NONE) {
    }

    else if (result == DIR_CW) {
        encoder2Pos--;
    }
    else if (result == DIR_CCW) {
        encoder2Pos++;
    }
}

ISR(PCINT1_vect) { // stick, on A5 and A4
    unsigned char result = rStick.process();
    if (result == DIR_NONE) {
    }

    else if (result == DIR_CW) {
        encoder1Pos--;
        //        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoder1Pos++;
    }
}



//---- print help ---------
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

