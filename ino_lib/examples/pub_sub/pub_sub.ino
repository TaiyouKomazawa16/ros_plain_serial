#include "plane_serial.h"

PlainSerial uart(&Serial);

void setup() {
    pinMode(13, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    volatile int id = -1;
    double result = uart.read(&id);
    if(id >= 0){
        digitalWrite(13,HIGH);
    }
    uart.write_float(1, millis() * 0.001);  //linear x
    uart.write_float(2, millis() * 0.002);  //linear y
    uart.write_float(3, 0.5);               //angular z
}
