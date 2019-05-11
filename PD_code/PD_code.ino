#include <Rotary.h>

//https://cdn.usdigital.com/assets/datasheets/H5_datasheet.pdf?k=636931248608523021

int val;
//int encoder0PinA = 8;
//int encoder0PinB = 9;
int encoder0Pos = 0;
//int encoder0PinALast = LOW;
//int n = LOW;

Rotary r = Rotary(2, 3);

void setup() {
    //  pinMode (encoder0PinA, INPUT);
    //  pinMode (encoder0PinB, INPUT);
    Serial.begin (9600);
    r.begin();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
    sei();
}

void loop() {
    delay(100);
    Serial.println((double(encoder0Pos) / 1250) * 360);
}
    /*char ch = Serial.read();*/
    /*if (ch == 'z') {*/
    /*encoder0Pos = 0;*/
    /*Serial.print("zeroed ... /0.00");*/
    /*}*/
    /*unsigned char result = r.process();*/
    /*if (result) {*/
    /*if (result == DIR_CW) {*/
    /*encoder0Pos++;*/
    /*}*/
    /*else {*/
    /*encoder0Pos--;*/
    /*}*/
    /*Serial.println(result == DIR_CW ? "Right" : "Left");*/

    /*Serial.print((double(encoder0Pos) / 1250) * 360);*/
    /*Serial.print ("\n");*/
    /*Serial.print ("/");*/
//    encoder0PinALast = n;
//  }


ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if (result == DIR_NONE) {
        // do nothing
    }
    else if (result == DIR_CW) {
        encoder0Pos++;
        /*Serial.println("ClockWise");*/
    }
    else if (result == DIR_CCW) {
        encoder0Pos--;
        /*Serial.println((double(encoder0Pos) / 1250) * 360);*/
        /*Serial.println("CounterClockWise");*/
    }
}
