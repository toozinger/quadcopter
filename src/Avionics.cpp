#include "Avionics.h"
#include <Arduino.h>
#include <math.h>

#include "sensors/sensors.h"
#include "sensors/kalman/Kalman.h"

Avionics::Avionics(){

}

Avionics::~Avionics() {

}

void Avionics::setup() {
    delay(50);
	sensors = new Sensors();
	sensors->setupSensors();

    imu = new FreeIMU(sensors);
    imu->init();

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
    pitchPID = new PID(&pitch_Input, &pitch_Output, &pitch_Setpoint, 8.0, 2.0, 0, DIRECT);
    pitchPID->SetSampleTime(5);
    pitchPID->SetMode(AUTOMATIC);
    pitchPID->SetOutputLimits(-20,20);
    pitch_Output = 0;
    pitch_Setpoint = 0;

}

float euler[3];

void Avionics::update() {
	imu->getYawPitchRollRad(euler);
    // Yaw, roll, pitch

    // Try control by nested PID
    // [euler angle, target angle] -> pitchPID
    // [gyro_rate, pitchPIDOutput] -> pitchRatePID -> motor (one per motor)

	 pitch_Input = euler[2];
     pitch_Setpoint = 0;

     Serial.print(euler[0]);
     Serial.print(" , ");
     Serial.print(euler[1]);
     Serial.print(" , ");
     Serial.print(euler[2]);
     Serial.println();
 //    pitch_Output = 0;


 //    if (pitchPID->Compute()) {
 //        pitchMinus_Setpoint = pitch_Output;
 //        pitchPlus_Setpoint = pitch_Output;

 //        pitchMinus_Input = sensors->gyro.x;
 //        pitchPlus_Input = -sensors->gyro.x;

 //        if (pitchMinusPID->Compute()){
 //            motor_pitch_minus->write(pitchMinus_Output);
 //        }

 //        if (pitchPlusPID->Compute()){
 //            motor_pitch_plus->write(pitchPlus_Output);
 //        }
 //        // Serial.print(pitchMinus_Output);
 //        // Serial.print(":");
 //        // Serial.println(pitchPlus_Output);
 //    }


	// Slow correction

	// if (pitchPID->Compute()) {
	// 	Serial.print("In: "); Serial.print(pitch_Input);
	// 	Serial.print(";    Out: ");Serial.println(pitch_Output);
	// 	motor_pitch_plus->write(47 + pitch_Output);
	// 	motor_pitch_minus->write(47 - pitch_Output);

	// 	// motor_roll_minus->write(vAcc_Output);
	// 	// motor_roll_plus->write(vAcc_Output);
	// }
}