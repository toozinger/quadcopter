#ifndef __AVIONICS_H
#define __AVIONICS_H

#include <Servo.h>
#include <PID_v1.h>

// Quick macros to make some conversions
#define TO_RAD(x) (x * 0.01745329252)    // *pi/180
#define TO_DEG(x) (x * 57.2957795131)    // *180/pi

#define MOTOR_ZERO (20)

class Sensors;
class FreeIMU;

class Avionics
{
public:
	Servo *motor_pitch_plus;
	Servo *motor_pitch_minus;
	Servo *motor_roll_plus;
	Servo *motor_roll_minus;

	PID *pitchPID;
	double pitch_Input;
	double pitch_Output;

	PID *rollPID;
	double roll_Input;
	double roll_Output;

	PID *vAccPID;
	double vAcc_Input;
	double vAcc_Output;

	Sensors *sensors;
	FreeIMU *imu;
	
public:
	double pitch_Setpoint;
	double roll_Setpoint;
	double vAcc_Setpoint;

	Avionics();
	~Avionics();

	void setup();
	void update();
};

#endif