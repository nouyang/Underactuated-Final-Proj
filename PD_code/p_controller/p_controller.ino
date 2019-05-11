double theta = 0.0; // get_from_encoder()
double theta_desired = 0.0; // [x_fixed point]
double delta_theta = 0.0;
unsigned long time;
unsigned long time_elapsed;
unsigned long prevTime;

void setup() {
    Serial.begin(9600);
}

void loop(){
    time = millis();
    Serial.print("Time: ");
    Serial.print(time);
    time_elapsed =  prevTime - time;
    if read_encoder == True:
        encoder++;
    delta_theta = prevEncoder - theta;

    thetadot = theta / time_elapsed;
    speed = k * (theta_desired - thetadot);
    motorLeft(speed);

    
// read state
//    t1dot = encoder / time_elapsed;
//    theta1 = encoder1
//    t2dot = endoer2
//    theta2 = encoder2 / time_elapsed;

    # calc error from fixed point
    error = [desired state] - [theta1 t1dot theta2 t2dot]

    # consider angle wrapping!   see elizabeth's code
    if error[0] > pi etc.

    # calc torque to motor, which is just k * error
    u = K1 * (err[0])
    u += K2 * (err[1])
    u += K3 * (err[2])
    u += K4 * (err[3]) 

    int speed = calcMotorSpeed(u);
    motorWrite(speed);
}

// convert our u in torque units, to u in "analogwrite" units
int calcMotorSpeed(torque){
// torque = Iwr
    motorSpeed = ?? torque /  
    return motorSpeed
// Check that torque is less than max torque
}

void motorLeft(speed){
    digitalWrtie 12 LOW
    digital write 9 Low
    analogWrite 3 123
}
void motorRight(speed){
    blah
}

void motorOn(){
    blah
}



# Our code should also do similar things -- 
  u = np.zeros((1, 1))
    error = x-xf;
    error[1] = (error[1])%(math.pi*2)
    if(error[1]>math.pi):
        error[1] = error[1]-2*math.pi

    error[0] = (error[0])%(math.pi*2)
    if(error[0]>math.pi):
        error[0] = error[0]-2*math.pi

    #print error

    ideal_control = -np.dot(K,error)

    if(ideal_control< -input_max ):
        u=np.array([-input_max])
    elif(ideal_control>input_max):
        u=np.array([input_max])
    else:
        u=ideal_control
