#ifndef __SENSORS_H
#define __SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <I2C.h> // I2C for the sensor libs
#include <L3G4200D.h> // The gyro lib
#include <Adafruit_BMP085.h> // The barometric pressure lib

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 20    // in milliseconds

class KalmanFilter {
private:
	Adafruit_BMP085 press;
	L3G4200D gyroscope;
	// RAW sensor data
	float accel[3];    // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
	//float accel_min[3];
	//float accel_max[3];

	float magnetom[3];
	//float magnetom_min[3];
	//float magnetom_max[3];

	float gyro[3];
	//float gyro_average[3];
	//int gyro_num_samples = 0;

	float temperature;
	float pressure;
	float altitude;

	// DCM variables
	float MAG_Heading;
	float Magn_Vector[3]= {0, 0, 0}; // Store the magnetometer turn rate in a vector
	float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
	float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
	float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
	float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
	float Omega_I[3]= {0, 0, 0}; // Omega Integrator
	float Omega[3]= {0, 0, 0};
	float errorRollPitch[3] = {0, 0, 0};
	float errorYaw[3] = {0, 0, 0};
	float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
	float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

	// More output-state variables
	int num_accel_errors = 0;
	int num_magn_errors = 0;
	int num_gyro_errors = 0;

	long sensorTimestamp;
	long timestamp;
	long timestamp_old;
	float G_Dt;
public:
	KalmanFilter();
	~KalmanFilter();
	void SetupSensors();
	void Compute();
private:
	void reset_sensor_fusion();
	void ReadSensors();
	static float Vector_Dot_Product(float vector1[3], float vector2[3]);
	static void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
	static void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
	static void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
	static void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
	void Normalize(void);
	void Drift_correction(void);
	void Matrix_update(void);
	void Euler_angles(void);
	void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
	void I2C_Init();
	void Accel_Init();
	void Read_Accel();
	void Magn_Init();
	void Read_Magn();
	void Gyro_Init();
	void Read_Gyro();
	void Pressure_Init();
	void Read_Pressure();
	void ApplySensorMapping();
	void Compass_Heading();
public:
	float yaw;
	float pitch;
	float roll;
};

#endif