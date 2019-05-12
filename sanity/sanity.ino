// Sanity check that encoder works
// 11 May 2019

#include <Rotary.h>

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

// --------Lever Encoder--------
int val;
volatile int encoderMotorPos = 0;
volatile int encoderStickPos = 0;
//Rotary r = Rotary(2, 3);
Rotary rMotor = Rotary(2, 3);
Rotary rStick = Rotary(4, 5);
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
double prev_theta = 0.0; // get_from_encoder()
double thetadot = 0.0; // get_from_encoder()
double delta_theta = 0.0; // get_from_encoder()

unsigned long curr_time;
unsigned long time_elapsed;
unsigned long prev_time;

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
    Serial.begin (230400);
    delay(50);
    Serial.println("hi");
    rMotor.begin();
    rStick.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); //D 2 and 3
    PCICR |= (1 << PCIE0);
    PCMSK2 |= (1 << PCINT20) | (1 << PCINT21); // D4 and D5
    sei();
}

void loop(){
    // -------- update time --------
    /*prev_time = curr_time;*/
    /*curr_time = millis();*/
    //    Serial.print("Time: ");
    //    Serial.println(curr_time);
    /*time_elapsed =  prev_time - curr_time;*/
    //    Serial.println(prev_time);

    //    if read_encoder == True:
    //        encoder++;

    // -------- update theta --------
    /*prev_theta = theta;*/
    theta = getCurrentTheta();
    /*delta_theta = prev_theta - theta;*/

    /*//    Serial.println(delta_theta);*/
    /*if (delta_theta < 0) {*/
        /*theta_CW = false;*/
    /*}*/
    /*else {*/
        /*theta_CW = true;*/
    /*}*/
    //    Serial.println(theta_CW);
    // -------- determine motor input --------
    /*thetadot = delta_theta / time_elapsed;*/
    /*motor_speed = k * (thetadot - thetadot_desired);*/
    /*Serial.println(motor_speed);*/
    /*aprintf("\nth %f, v_motor %d", theta, motor_speed);*/
    /*aprintf("\ntheta %d", theta);*/
    Serial.println(getCurrentTheta());
    Serial.println(theta);
    /*}*/
    delay(100);


}

// -------- Angle Stuff --------
double getCurrentTheta() {
    return (double(encoderMotorPos) / 1250) * 360;
}



// ---- Set interrupt to read encoder ----

ISR(PCINT2_vect) {
    unsigned char result = rMotor.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoderMotorPos--;
        //        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoderMotorPos++;
        //        Serial.println(getCurrentTheta());
    }
}

ISR(PCINT0_vect) {
    unsigned char result = rStick.process();
    if (result == DIR_NONE) {
        // do nothing
    }

    else if (result == DIR_CW) {
        encoderStickPos--;
        //        Serial.println(getCurrentTheta());
    }
    else if (result == DIR_CCW) {
        encoderStickPos++;
        //        Serial.println(getCurrentTheta());
    }
}
