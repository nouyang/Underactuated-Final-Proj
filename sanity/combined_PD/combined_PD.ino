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
/*int EnablePin = 8;*/
/*int duty;*/
/*int PWMPin = 11;  // Timer2*/
/*int PWMPin2 = 10;*/
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

bool theta_CW;
bool motor_CW;

unsigned long curr_time;
unsigned long time_elapsed;
unsigned long prev_time = 0;

double state[4];

double k = 2; // P constant


void setup() {
    /*Serial.begin(230400);*/
    Serial.begin(9600); // for use with plotter tool 

    rMotor.begin();
    rStick.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT13) | (1 << PCINT12);
    sei();

    motor.Enable();
    motor.SetStepDelay(0);
}

void loop(){
    // -------- update time --------

    curr_time = millis();
    time_elapsed = curr_time - prev_time;
    prev_time = curr_time;

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

//    if (theta1dot > 0) {
//        theta_CW = true;
//    }
//    else if (theta1dot < 0){
//        theta_CW = false;
//    }
    if (delta_theta1 < 0) {
      theta_CW = true;
    }
    else {
      theta_CW = false;
    }

    // -------- determine motor input --------
    /*thetadot = delta_theta / time_elapsed;*/
    /*motor_speed = k * (thetadot - thetadot_desired);*/
    /*Serial.println(motor_speed);*/
    
    /*motor_output = - ceil(k * (theta1 - theta1_desired));*/
    motor_output = - 30;
//    Serial.print("Motor output: ");
//    Serial.print(motor_output);
    /*aprintf("\ntheta1 %f, t2 %f, t1dot %f, t2dot %f, out %d, deltath %f, cw ", */
            /*theta1, theta2, theta1dot, theta2dot, motor_output, delta_theta1);*/
    /*Serial.println(motor_output*10);*/
    Serial.print(theta2);
    Serial.print(" ");
    Serial.print(motor_output);
    Serial.print(" ");
    Serial.println(theta1);
//    Serial.println();/*}*/
    
//    delay(100);
    // -------- write appropriate motor input --------
    motor_output  = abs(constrain(motor_output, -255, 255));
//    if (abs(theta1) < 2.0) {
//        
/*//    }*/
    if (theta1 > 0) { //to the left 
        if (theta_CW == true) {
            motor.Fwd(motor_output);
            /*Serial.print("running CCW");*/
            /*Serial.print(abs(motor_output));*/
        }
        else {
            motor.Rev(motor_output);
            /*motorCW(abs(motor_output));*/
            /*Serial.print("running CW");*/
            /*Serial.print(abs(motor_output));*/
        }
    }
    else if (theta1 < 0) {
        if (theta_CW == true) {
            motor.Rev(motor_output);
            /*motorCCW(abs(motor_output));*/
            /*Serial.print("running CCW");*/
            /*Serial.print(abs(motor_output));*/
        }
        else {
            motor.Fwd(motor_output);
            /*motorCW(abs(motor_output));*/
            /*Serial.print("running CW");*/
            /*Serial.print(abs(motor_output));*/
        }
    }




}
// ----- End main loop

// --------- Helper Functions -------

/*// -------- Motor Funcs --------*/
/*void motorOn(){*/
    /*digitalWrite(EnablePin, HIGH);*/
    /*analogWrite(PWMPin2, 0);*/
/*}*/
/*void motorCW(int input_speed) {*/
    /*PWMPin = 10;*/
    /*PWMPin2 = 11;*/
    /*motorWrite(input_speed);*/
    /*motor_CW = true;*/
/*}*/
/*void motorCCW(int input_speed) {*/
    /*PWMPin = 11;*/
    /*PWMPin2 = 10;*/
    /*motorWrite(input_speed);*/
    /*motor_CW = false;*/
/*}*/
/*void motorWrite(int input_speed) {*/
/*//    for(duty = 0; duty <= input_speed; duty += 50){*/
/*//        analogWrite(PWMPin, duty);*/
/*//        delay(5);*/
/*//    }*/
  /*analogWrite(PWMPin, input_speed);*/
/*}*/
    
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

