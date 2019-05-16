// PD  controller for upright stabilization
// Remove deadband
// Add e-stop button and potentiometers for 
// 15 May 2019
// Test

#include <Rotary.h>
#include <MegaMotoHB.h>
#include <math.h>

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

// --------Lever Encoder--------
int val;
volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
// Rotary rMotor = Rotary(2, 3); // motor (theta2)
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
int motor_state = 0;

// --------P-Controller--------
double thetadot_deadband = 0.2;
double theta_deadband = 5;

int sample_time = 0; // 15 msec

double theta1 = 0.0; // get_from_encoder()
double theta2 = 0.0; // get_from_encoder()
double prev_theta1 = 0.0; // get_from_encoder()
double prev_theta2 = 0.0; // get_from_encoder()

double theta1dot = 0.0; // get_from_encoder()
double delta_theta1 = 0.0; // get_from_encoder()
double theta2dot = 0.0; // get_from_encoder()
double delta_theta2 = 0.0; // get_from_encoder()

//double theta1_desired = 0.0;
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
double kdot = -80; // thetadot 

// PIN 5 SOMEHOW RELATED TO MOTOR !  ! ! 

const int ledPin =  12;      // the number of the LED pin



void setup() {
    Serial.begin(9600);
    /*Serial.begin(9600); // for use with plotter tool */

    // rMotor.begin();
    rStick.begin();



    // Enable interrupts for the two encoders
    // PCICR |= (1 << PCIE2); 
    // PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // Pins D2 and D3 
    PCICR |= (1 << PCIE1); //RIP changed motors, no motor encoder now
    PCMSK1 |= (1 << PCINT13) | (1 << PCINT12); // Pins A5 and A4 
    sei();

    /*motorOn();*/
    motor.Enable();
    motor.SetStepDelay(0);
    /*setPwmFrequency(PWMPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq*/
}

void loop(){
    // -------- update time --------
        now = millis();
        time_elapsed = (now - prev_time);
        if (time_elapsed >= sample_time)
            {
                prev_time = now;

                // -------- update theta --------

                // theta2 = getCurrentTheta2();
                theta1 = getCurrentTheta1();
                delta_theta1 = theta1 - prev_theta1;
                // delta_theta2 = theta2 - prev_theta2;
                theta1dot = delta_theta1 / time_elapsed;
            // theta2dot = delta_theta2 / time_elapsed;
                prev_theta1 = theta1;
                state[0] = theta1;
                // state[1] = theta2;
                state[2] = theta1dot;
                // state[3] = theta2dot;

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
                aprintf("\n t1 %f errtheta %f, errdot %f, motor out %d, t1dot %f", theta1, err_theta, err_thetadot, motor_output, theta1dot);
                /*Serial.println(theta1);*/
                /*Serial.print(delta_motor);*/

                /*// -------- write appropriate motor input --------*/
                //motor_output = 75 * err_thetadot;
                /*motor_output = 5 * theta1;*/
                motor_output  = abs(constrain(motor_output, -200, 200));
                motor_output = 70;

                int someFlag = -1;
                k = 10;
                kdot = 10;
                if (abs(theta1) < 30){

                    if ((theta1 > 2+1)) {
                        if (theta1dot > 0.01) { // going away... slow it down -- FIGHT!
                            /*motorWrite(someFlag * motor_output);*/
                            motor_output = 20 + (k * abs((theta1)) + kdot * abs(theta1dot));
                            motor_output  = abs(constrain(motor_output, -200, 200));
                            motorWrite(+60); // fight gravity harder
                        }

                        // if (theta1dot < 0.1) { // going towrad .. 
                        // /*motorWrite(-someFlag * motor_output);*/
                        // motor_output = 60;
                        // motor_output  = abs(constrain(motor_output, -200, 200));
                        // motor.Rev(motor_output);
                        // Serial.print("\nRev");
                        // Serial.print(motor_output);
                        // }

                        // else {
                        // if (motor_state < 0){
                        // // motor.Rev(motor_state -= 20);
                        // }
                        // else{
                        // motor.Fwd(motor_state -= 20);
                        // }
                        // }
                    }

                    else if ((theta1 < 2-1)) {
                        // if (theta1dot > 0.1) { // going toward
                        // motor_output  = abs(constrain(motor_output, -200, 200));
                        // motor.Rev(motor_output);
                        // /*motorCCW(abs(motor_output));*/
                        // Serial.print("\nRev");
                        // Serial.print(motor_output);

                        // }

                        if (theta1dot < -0.01) { // going away -- fight!!
                            motor_output = 20 + (k * abs(theta1) + kdot * abs(theta1dot));
                            motor_output  = abs(constrain(motor_output, -200, 200));
                            motorWrite(-60);
                            // if (theta1dot < 0.1) { // going towrad .. 
                            // /*motorWrite(-someFlag * motor_output);*/
                            // motor_output = 60;
                            // motor_output  = abs(constrain(motor_output, -200, 200));
                            // motor.Rev(motor_output);
                            // Serial.print("\nRev");
                            // Serial.print(motor_output);
                            // }

                            // else {
                            // if (motor_state < 0){
                            // // motor.Rev(motor_state -= 20);
                            // }
                            // else{
                            // motor.Fwd(motor_state -= 20);
                            // }
                            // }
                        }                        
                        Serial.print("\nRev");
                        Serial.print(motor_output);

                        /*motorWrite(someFlag * motor_output);*/
                        /*motorCW(abs(motor_output));*/
                    }

                    else {
                    if (motor_state < 0){
                    motor.Rev(motor_state -= 20);
                    }
                    else{
                    motor.Fwd(motor_state -= 20);
                    }
                    }
                }
                else{
                    motor.Stop();
                }
            }
        }

            // SANITY CHECK
            /*
               motor.Rev(200);
               delay(500);
               motor.Fwd(200);
               delay(500);
               motor.Stop();
               delay(500);
             */


    // --------- Helper Functions -------

    // -------- Motor Funcs --------


    // Implement bang bang control on theta2 dot dot 
    // -- This is PID loop to control actual motor speed to desired speed 
    void motorWrite(int some_value) {
        int new_state = motor_state + some_value; 
        new_state = constrain(new_state, -200, 200);

        if (new_state > 0) {
            motor.Fwd(new_state);
        }
        else if (new_state < 0) { // < 0
            motor.Rev(-new_state);
        }
        else {
            // motorWrite(1);
            //do nothing
        }
        motor_state = new_state;
    }

    // -------- Angle Conversion --------
    double getCurrentTheta1() { // calibration for stick encoder = 1250
        double val = (double(encoder1Pos) / 1250) * 360 + 180;
        val = fmod(val, 360);
        if (val > 180) {
            val = val - 360;
        }
        return val;
    }


    // RIP motor encoder
    // double getCurrentTheta2() {  // 500 ticks / rev, for motor encoder
    // double val = (double(encoder2Pos) / 500 ) * 360 + 180;
    // val = fmod(val, 360);
    // if (val > 180) {
    // val = val - 360;
    // }
    // return val;
    // }

    // ---- Set interrupt to read encoder ----

    ISR(PCINT1_vect) { // stick, on D2 and D3
        unsigned char result = rStick.process();
        if (result == DIR_NONE) {
        }

        else if (result == DIR_CW) {
            encoder1Pos--;
        }
        else if (result == DIR_CCW) {
            encoder1Pos++;
        }
    }

    // ISR(PCINT2_vect) { // RIP, previously motor, on A5 and A4
    // unsigned char result = rStick.process();
    // if (result == DIR_NONE) {
    // }

    // else if (result == DIR_CW) {
    // encoder1Pos--;
    // //        Serial.println(getCurrentTheta());
    // }
    // else if (result == DIR_CCW) {
    // encoder1Pos++;
    // }
    // }



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

    // For motor controller 
    void setPwmFrequency(int pin, int divisor) {
        byte mode = 0;
        if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
            switch(divisor) {
                case 1: mode = 0x01; break;
                case 8: mode = 0x02; break;
                case 64: mode = 0x03; break;
                case 256: mode = 0x04; break;
                case 1024: mode = 0x05; break;
                default: return;
            }
            if(pin == 5 || pin == 6) {
                TCCR0B = TCCR0B & 0b11111000 | mode;
            } else {
                TCCR1B = TCCR1B & 0b11111000 | mode;
            }
        } else if(pin == 3 || pin == 11) {
            switch(divisor) {
                case 1: mode = 0x01; break;
                case 8: mode = 0x02; break;
                case 32: mode = 0x03; break;
                case 64: mode = 0x04; break;
                case 128: mode = 0x05; break;
                case 256: mode = 0x06; break;
                case 1024: mode = 0x07; break;
                default: return;
            }
            TCCR2B = TCCR2B & 0b11111000 | mode;
        }
        // motorWrite(1);
    }
