#include <Arduino.h> // Arduino Stuffs
#include <Servo.h> // Servo lib for controlling motor controllers
#include <PID_v1.h> // The Arduino PID lib

#include "main.h"
#include "sensors.h"

Servo *motor_one=0;

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 20    // in milliseconds

// Quick macros to make some conversions
#define TO_RAD(x) (x * 0.01745329252)    // *pi/180
#define TO_DEG(x) (x * 57.2957795131)    // *180/pi
/*****************************************************************/

double Setpoint, Input, Output;
PID *myPID=0;


void setup() {
    // Init serial output
    Serial.begin(OUTPUT_BAUD_RATE);
    
    // Init sensors
    delay(50);    // Give sensors enough time to start
    I2C_Init();
    Accel_Init();
    Magn_Init();
    Gyro_Init();
    Pressure_Init();
    
    // Read sensors, init DCM algorithm
    delay(20);    // Give sensors enough time to collect data
    reset_sensor_fusion();

    // Set the PID to seek pitch 0
    Setpoint = 0;

    // Make the pid                       P, I, D
    myPID = new PID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

    //Make motor one
    motor_one = new Servo();

    // Turn the pid on
    myPID->SetMode(AUTOMATIC);
    // Set the PIds limits
    myPID->SetOutputLimits(40, 120);
 
    motor_one->attach(10);
    
    motor_one->write(20);
    delay(5000);
}

// Main loop
void loop() {    
    // Time to read the sensors again?
    if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
        timestamp_old = timestamp;
        timestamp = millis();
        if (timestamp > timestamp_old)
            G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        else
            G_Dt = 0;

        ReadSensors();
        
        // Run DCM algorithm
        Compass_Heading(); // Calculate magnetic heading

        Matrix_update();
        Normalize();
        Drift_correction();
        Euler_angles();

        // Serial.print("Acc: "); Serial.print(gyro[0], DEC);        Serial.print(";     ");
        // Serial.print("yaw: "); Serial.print(TO_DEG(yaw));        Serial.print(";     ");
        Serial.print("pitch: "); Serial.print(TO_DEG(pitch)); Serial.print(" ");
        // Serial.print("roll: "); Serial.print(TO_DEG(roll));     Serial.print(";     ");
        // Serial.print("temp: "); Serial.print(temperature);        Serial.print(";     ");
        // Serial.print("pressure: "); Serial.print(pressure);             Serial.print(";     ");
        // Serial.print("altitude: "); Serial.print(altitude);             
        
        
        Input = TO_DEG(pitch);
        myPID->Compute();
        Serial.print("speed: "); Serial.print(Output/2); Serial.print(";    "); Serial.println();
        motor_one->write(Output/2);
        
        delay(20);
    }
}

