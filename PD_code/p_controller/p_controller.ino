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

// --------P-Controller--------
double theta = 0.0; // get_from_encoder()
double thetadot = 0.0;
double thetadot_desired = 0.0; // [x_fixed point]
double delta_theta = 0.0;
double prev_theta = 0.0;
bool theta_CW;
bool motor_CW;
unsigned long curr_time;
unsigned long time_elapsed;
unsigned long prev_time;
double k = 10; // Proportional Constant
int motor_speed = 0.0;

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
    motorOn();
}

void loop(){
    // -------- update time --------
    prev_time = curr_time;
    curr_time = millis();
//    Serial.print("Time: ");
//    Serial.println(curr_time);
    time_elapsed =  prev_time - curr_time;
//    Serial.println(prev_time);
    
//    if read_encoder == True:
//        encoder++;
        
    // -------- update theta --------
    prev_theta = theta;
    theta = getCurrentTheta();
    delta_theta = prev_theta - theta;

//    Serial.println(delta_theta);
    if (delta_theta < 0) {
      theta_CW = false;
    }
    else {
      theta_CW = true;
    }
//    Serial.println(theta_CW);
    // -------- determine motor input --------
    thetadot = delta_theta / time_elapsed;
    motor_speed = k * (thetadot - thetadot_desired);
//    Serial.println(motor_speed);

    // -------- write appropriate motor input --------
    if (motor_speed < 0) {
      motorCCW(-motor_speed);
//      Serial.println("calling motorCCW");
      delay(10);
    }
    else if (motor_speed > 0) {
      motorCW(motor_speed);
//      Serial.println("calling motorCW");
      delay(10);
    }

    // just for testing (delete later)
//    if (theta_CW == true) {
//      motorCW(255);
//    }
//    else {
//      motorCCW(255);
//    }

//    motorCW(200);
//    Serial.println(motor_CW);
//    delay(300);
//    motorCCW(200);
//    Serial.println(motor_CW);
//    delay(300);

   


/////        TODO: IMPLEMENT

// -------- Read State --------
//    t1dot = encoder / time_elapsed;
//    theta1 = encoder1
//    t2dot = endoer2
//    theta2 = encoder2 / time_elapsed;


//    # calc error from fixed point
//    error = [desired state] - [theta1 t1dot theta2 t2dot]
//
//    # consider angle wrapping!   see elizabeth's code
//    if error[0] > pi etc.
//
//    # calc torque to motor, which is just k * error
//    u = K1 * (err[0])
//    u += K2 * (err[1])
//    u += K3 * (err[2])
//    u += K4 * (err[3]) 
//
//    int speed = calcMotorSpeed(u);
//    motorWrite(speed);
}



//// Convert our u in torque units, to u in "analogwrite" units
//int calcMotorSpeed(torque){
//  
//// torque = Iwr
//    motorSpeed = ?? torque /  
//    return motorSpeed
//// Check that torque is less than max torque
//}


// -------- Angle Stuff --------
double getCurrentTheta() {
  return (double(encoder0Pos) / 1250) * 360;
}


// -------- Motor Functions --------
void motorOn(){
    digitalWrite(EnablePin, HIGH);
    analogWrite(PWMPin2, 0);
}

// arg requirement: speed <= 255
void motorCW(int input_speed) {
    PWMPin = 10;
    PWMPin2 = 11;
    motorWrite(input_speed);
    motor_CW = true;
}

// arg requirement: speed <= 255
void motorCCW(int input_speed) {
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
//  analogWrite(PWMPin, input_speed);
}


// -------- Error Stuff --------

//# Our code should also do similar things -- 
//  u = np.zeros((1, 1))
//    error = x-xf;
//    error[1] = (error[1])%(math.pi*2)
//    if(error[1]>math.pi):
//        error[1] = error[1]-2*math.pi
//
//    error[0] = (error[0])%(math.pi*2)
//    if(error[0]>math.pi):
//        error[0] = error[0]-2*math.pi
//
//    #print error
//
//    ideal_control = -np.dot(K,error)
//
//    if(ideal_control< -input_max ):
//        u=np.array([-input_max])
//    elif(ideal_control>input_max):
//        u=np.array([input_max])
//    else:
//        u=ideal_control


// ---- Set interrupt to read encoder ----

ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoder0Pos--;
//        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoder0Pos++;
//        Serial.println(getCurrentTheta());
    }
}



// ---- Set clock frequency for motor controller ----

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
