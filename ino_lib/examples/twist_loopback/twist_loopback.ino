#include "plane_serial.h"

PlainSerial uart(&Serial);

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600);
}

double x = 0;
double y = 0;
double theta = 0;

void loop() {
    volatile int id = -1;
    double result = uart.read(&id);
    if(id >= 0){
        digitalWrite(13,HIGH);
        switch(id){
            case 1:
                x = result;
                break;
            case 2:
                y = result;
                break;
            case 3:
                theta = result;
                break;
            default:
                break; //nop
        };
    }else{
        digitalWrite(13,LOW);
    }

    uart.write_float(1, x);     //linear x
    uart.write_float(2, y);     //linear y
    uart.write_float(3, theta); //angular z
}
