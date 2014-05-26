#include "Avionics.h"
#include <Arduino.h>
#include <math.h>

Avionics::Avionics(){

}

Avionics::~Avionics() {

}

void Avionics::setup() {
	sensors = new KalmanFilter();
	sensors->SetupSensors();

    Serial.println("Starting ESCs...");
    motor_pitch_plus = new Servo();
    motor_pitch_plus->attach(9);

    motor_pitch_minus = new Servo();
    motor_pitch_minus->attach(12);

    motor_roll_plus = new Servo();
    motor_roll_plus->attach(8);

    motor_roll_minus = new Servo();
    motor_roll_minus->attach(11);
    
    motor_pitch_plus->write(MOTOR_ZERO);
    motor_pitch_minus->write(MOTOR_ZERO);

    motor_roll_plus->write(MOTOR_ZERO);
    motor_roll_minus->write(MOTOR_ZERO);
    delay(2000);

    Serial.println("ESCs Started");

    // Setup PIDs
    pitchPID = new PID(&pitch_Input, &pitch_Output, &pitch_Setpoint, 4.0, 1.0, 0.5, DIRECT);
    pitchPID->SetSampleTime(5);
    pitchPID->SetMode(AUTOMATIC);
    pitchPID->SetOutputLimits(-10.0,10.0);
    pitch_Output = 0;
	pitch_Setpoint = 0;
}

void Avionics::update() {
	vAcc_Output = 47;
	sensors->Compute();
	pitch_Input = sensors->pitch;
	roll_Input = sensors->roll;


	// Slow correction

	if (pitchPID->Compute()) {
		if (fabs(pitch_Input - pitch_Setpoint) < TO_RAD(5)) { // 
			pitch_Output *= 0.5;
		}
		Serial.print("In: "); Serial.print(pitch_Input);
		Serial.print(";    Out: ");Serial.println(pitch_Output);
		// motor_pitch_plus->write(vAcc_Output + pitch_Output);
		// motor_pitch_minus->write(vAcc_Output - pitch_Output);

		// motor_roll_minus->write(vAcc_Output);
		// motor_roll_plus->write(vAcc_Output);
	}
}