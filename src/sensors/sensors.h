#ifndef __SENSORS_H
#define __SENSORS_H

// Sensor data output interval in milliseconds
#define OUTPUT_DATA_INTERVAL 20    // in milliseconds

#include <Arduino.h>
#include <Wire.h>
#include <I2C.h>
#include <L3G4200D.h>
#include <Adafruit_BMP085.h>

class Sensors {
private:
	typedef struct vector
		{
			float x, y, z;
		} vector;
	vector gyroZero;
private:
	unsigned long lastRead;
	Adafruit_BMP085 press;
	L3G4200D gyroscope;

public:
	vector gyro, accel, magn;
	float temperature, pressure, altitude;

	void setupSensors();
	void readSensors();
};
#endif