#include <Rotary.h>
#include <MegaMotoHB.h>
#include <math.h>

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
// PWMPin (timer22), PWMPin2, EnablePin
MegaMotoHB motor(11, 10, 8);
int duty;

// --------P-Controller--------
double curr_t1 = 0.0; // get_from_encoder()
double curr_t1dot = 0.0;

double t1_desired = 0.0; // [x_fixed point]
double t1dot_desired = 0.0; // [x_fixed point]

double delta_t1 = 0.0;
double prev_t1 = 0.0;

unsigned long curr_micros;
unsigned long time_elapsed;
unsigned long prev_micros;

double k = 10; // Proportional Constant

int t2dot = 0; // store motor speed
int motor_output = 0; // command to motor


// ---- fancy arduino printf
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
    Serial.begin (230400);
    r.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    sei();
}

void loop(){
    // -------- update time --------
    curr_micros = micros();
    //    Serial.print("Time: ");
    //    Serial.println(curr_micros);
    time_elapsed =  curr_micros - prev_micros;
    /*aprintf("\ntime prev %l, curr %l, delta %l ", prev_micros, curr_micros, time_elapsed);*/
    prev_micros = curr_micros;
    /*delay(1000);*/
    //    Serial.println(prev_micros);

    // -------- update theta 1 --------
    curr_t1 = getCurrentTheta();
    delta_t1 = prev_t1 - curr_t1;
    prev_t1 = curr_t1;

    // -------- determine motor input --------
    motor_output = - ceil(k * (curr_t1 - t1_desired));
    // curr_t1dot = delta_t1 / double(time_elapsed);
    // motor_output = k * (t1dot - t1dot_desired);


    /*aprintf("\n motorout %d, error t1 %f", motor_output, (curr_t1 - t1_desired));*/
    aprintf("\n t1 %f, error t1 %f", curr_t1, (curr_t1 - t1_desired));
    /*aprintf("\nth %f, time el %l, tdot %f, err %f, v_m %d", theta, time_elapsed, t1dot, t1 - t1_desired, motor_output);*/
    /*aprintf("\nth %f, err t1 %f, curr_m, motor_out %d", theta, time_elapsed, t1dot, t1 - t1_desired, motor_output);*/
    

    // -------- write appropriate motor input --------
    if (motor_output < 0){
        /*motor.Rev( abs(motor_output) );*/
    }
    else {
        /*motor.Fwd(motor_output);*/
    }
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
    double val = (double(encoder0Pos) / 1250) * 360;
    val = fmod(val, 360);
    if (val > 180) {
        val = val - 360;
    }
    return val;
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
