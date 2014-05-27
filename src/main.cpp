#include <Arduino.h> // Arduino Stuffs
#include <Servo.h> // Servo lib for controlling motor controllers
#include <PID_v1.h> // The Arduino PID lib

#include "main.h"
#include "Avionics.h"

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200
/*****************************************************************/

char serialBuffer[512];

Avionics *avionics = 0;

void setup() {    
    // Init serial output
    Serial.begin(OUTPUT_BAUD_RATE);

    avionics = new Avionics();
    avionics->setup();
}

// Main loop
void loop() {
    avionics->update();

    // int head;
    // head = 0;
    // while ((serialBuffer[head++] = Serial.read()) != -1) {
    // }
    // if (head > 1) {
    //     serialBuffer[head] = 0;
    //     int code,p,i,d;
    //     sscanf(serialBuffer, "%d:%d,%d,%d\n", &code, &p, &i, &d);
    //     if (code == 99) {
    //         // Serial.print(p);
    //         // Serial.print(",");
    //         // Serial.print(i);
    //         // Serial.print(",");
    //         // Serial.println(d);
    //         // avionics->pitchPID->SetTunings((double)p/100.0, (double)i/100.0, (double)d/100.0);
    //     }
    // }
    
    delay(20);
}

