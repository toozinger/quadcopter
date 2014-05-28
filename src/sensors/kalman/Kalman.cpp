/*
 FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
 Copyright (C) 2011-2012 Fabio Varesano <fabio at varesano dot net>

 Development of this code has been supported by the Department of Computer Science,
 Universita' degli Studi di Torino, Italy within the Piemonte Project
 http://www.piemonte.di.unito.it/


 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */
#include "Kalman.h"

#include <math.h>

#include "../sensors.h"

//#include "vector_math.h"

FreeIMU::FreeIMU(Sensors *sensors) {
	this->sensors = sensors;
	// initialize quaternion
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
	lastUpdate = 0;
	now = 0;

#ifndef CALIBRATION_H
	// initialize scale factors to neutral values
	acc_scale_x = 1;
	acc_scale_y = 1;
	acc_scale_z = 1;
	magn_scale_x = 1;
	magn_scale_y = 1;
	magn_scale_z = 1;
#else
	// get values from global variables of same name defined in calibration.h
	acc_off_x = ::acc_off_x;
	acc_off_y = ::acc_off_y;
	acc_off_z = ::acc_off_z;
	acc_scale_x = ::acc_scale_x;
	acc_scale_y = ::acc_scale_y;
	acc_scale_z = ::acc_scale_z;
	magn_off_x = ::magn_off_x;
	magn_off_y = ::magn_off_y;
	magn_off_z = ::magn_off_z;
	magn_scale_x = ::magn_scale_x;
	magn_scale_y = ::magn_scale_y;
	magn_scale_z = ::magn_scale_z;
#endif
}

void FreeIMU::init() {
	init(0, false);
}

void FreeIMU::init(bool fastmode) {
	init(0, false);
}

void FreeIMU::init(int acc_addr, bool fastmode) {
	// zero gyro
	zeroGyro();
	Serial.println("IMU Zero");

	// place quaternion.  10 times since I couldn't get it working quite right
	for (int i = 0; i < 10; i++) {
		sensors->readSensors();
		float ax = sensors->accel.x;
		float ay = sensors->accel.y;
		float az = sensors->accel.z;

		float mx = sensors->magn.x;
		float my = sensors->magn.y;
		float mz = sensors->magn.z;

		float recipNorm;
		float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
		float qa, qb, qc;

		float errorCount = 0;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalization)
		if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
			float hx, hy, bx, bz;
			float halfwx, halfwy, halfwz;

			// Normalize magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Reference direction of Earth's magnetic field
			hx = 2.0f
					* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
							+ mz * (q1q3 + q0q2));
			hy = 2.0f
					* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
							+ mz * (q2q3 - q0q1));
			bx = sqrt(hx * hx + hy * hy);
			bz = 2.0f
					* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
							+ mz * (0.5f - q1q1 - q2q2));

			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex = (my * halfwz - mz * halfwy);
			halfey = (mz * halfwx - mx * halfwz);
			halfez = (mx * halfwy - my * halfwx);
			errorCount++;
		}

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
		if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
			float halfvx, halfvy, halfvz;

			// Normalize accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Estimated direction of gravity
			halfvx = q1q3 - q0q2;
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;

			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (ay * halfvz - az * halfvy);
			halfey += (az * halfvx - ax * halfvz);
			halfez += (ax * halfvy - ay * halfvx);
			errorCount++;
		}

		// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
		if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
			float gx = halfex / errorCount;
			float gy = halfey / errorCount;
			float gz = halfez / errorCount;
			if (i > 3) {
				gx /= (float) (i - 3);
				gy /= (float) (i - 3);
				gz /= (float) (i - 3);
			}
			qa = q0;
			qb = q1;
			qc = q2;
			q0 += (-qb * gx - qc * gy - q3 * gz);
			q1 += (qa * gx + qc * gz - q3 * gy);
			q2 += (qa * gy - qb * gz + q3 * gx);
			q3 += (qa * gz + qb * gy - qc * gx);
		}

		// Normalize quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		delayMicroseconds(5000);
	}

	eulerLastX = eulerLastY = eulerLastZ = 0;
	eulerRotX = eulerRotY = eulerRotZ = 0;
}

/**
 * Populates raw_values with the raw_values from the sensors
 */
void FreeIMU::getRawValues(float * raw_values) {
	sensors->readSensors();
	raw_values[0] = sensors->accel.x;
	raw_values[1] = sensors->accel.y;
	raw_values[2] = sensors->accel.z;

	raw_values[3] = sensors->gyro.x;
	raw_values[4] = sensors->gyro.y;
	raw_values[5] = sensors->gyro.z;

	raw_values[6] = sensors->magn.x;
	raw_values[7] = sensors->magn.y;
	raw_values[8] = sensors->magn.z;

	// raw_values[9] = sensors->temperature;
	// raw_values[10] = sensors->pressure;  NOT USED
}

/**
 * Populates values with calibrated readings from the sensors
 */
void FreeIMU::getValues(float * values) {
	getRawValues(values);

	// remove offsets from the gyroscope
	values[3] = values[3] - gyro_off_x;
	values[4] = values[4] - gyro_off_y;
	values[5] = values[5] - gyro_off_z;

	// remove offsets and scale accelerometer (calibration)
	values[0] = (values[0] - acc_off_x) / acc_scale_x;
	values[1] = (values[1] - acc_off_y) / acc_scale_y;
	values[2] = (values[2] - acc_off_z) / acc_scale_z;

	values[6] = (values[6] - magn_off_x) / magn_scale_x;
	values[7] = (values[7] - magn_off_y) / magn_scale_y;
	values[8] = (values[8] - magn_off_z) / magn_scale_z;
}

/**
 * Computes gyro offsets
 */
void FreeIMU::zeroGyro() {
	const int totSamples = 3;
	float raw[11];
	float tmpOffsets[] = { 0, 0, 0 };

	for (int i = 0; i < totSamples; i++) {
		getRawValues(raw);
		tmpOffsets[0] += raw[3];
		tmpOffsets[1] += raw[4];
		tmpOffsets[2] += raw[5];
		delayMicroseconds(1000);
	}

	gyro_off_x = tmpOffsets[0] / (float) totSamples;
	gyro_off_y = tmpOffsets[1] / (float) totSamples;
	gyro_off_z = tmpOffsets[2] / (float) totSamples;
}

/**
 * Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
 * compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
 * axis only.
 *
 * @see: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 */
void FreeIMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay,
		float az, float mx, float my, float mz) {
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float qa, qb, qc;

// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalization)
	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f) && !isnan(mx) && !isnan(my) && !isnan(mz)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;

		// Normalize magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f
				* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
						+ mz * (q1q3 + q0q2));
		hy = 2.0f
				* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
						+ mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f
				* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
						+ mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		// halfex = (my * halfwz - mz * halfwy);
		// halfey = (mz * halfwx - mx * halfwz);
		// halfez = (mx * halfwy - my * halfwx);
	}

// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
	if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f) && !isnan(ax) && !isnan(ay) && !isnan(az)) {
		float halfvx, halfvy, halfvz;

		// Normalize accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f && !isnan(halfex) && !isnan(halfey) && !isnan(halfez)) {
		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx; // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

// Normalize quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/**
 * Populates array q with a quaternion representing the IMU orientation with respect to the Earth
 *
 * @param q the quaternion to populate
 */
void FreeIMU::getQ(float * q) {
	float val[9];
	getValues(val);

	now = millis();
	if (lastUpdate > 0 && now != lastUpdate) {
		sampleFreq = 1.0 / ((now - lastUpdate) / 1000.0);

// gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
		AHRSupdate(val[3] * M_PI / 180.0, val[4] * M_PI / 180.0,
				val[5] * M_PI / 180.0, val[0], val[1], val[2], val[6], val[7],
				val[8]);
	}
	lastUpdate = now;

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

/**
 * Compensates the accelerometer readings in the 3D vector acc expressed in the sensor frame for gravity
 * @param acc the accelerometer readings to compensate for gravity
 * @param q the quaternion orientation of the sensor board with respect to the world
 */
void FreeIMU::gravityCompensateAcc(float * acc, float * q) {
	float g[3];

// get expected direction of gravity in the sensor frame
	g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
	g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

// compensate accelerometer readings with the expected direction of gravity
	acc[0] = acc[0] - g[0];
	acc[1] = acc[1] - g[1];
	acc[2] = acc[2] - g[2];
}

/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 *
 * @param angles three floats array which will be populated by the Euler angles in radians
 */
void FreeIMU::getEulerRad(float * angles) {
	float q[4]; // quaternion
	getQ(q);
	angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1); // psi
	angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
	angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1],
			2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 *
 * @param angles three floats array which will be populated by the Euler angles in degrees
 */
void FreeIMU::getEuler(float * angles) {
	getEulerRad(angles);
	arr3_rad_to_deg(angles);
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 *
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 *
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
 */
void FreeIMU::getYawPitchRollRad(float * ypr) {
	float q[4]; // quaternion
	float gx, gy, gz; // estimated gravity direction
	getQ(q);

	gx = 2 * (q[1] * q[3] - q[0] * q[2]);
	gy = 2 * (q[0] * q[1] + q[2] * q[3]);
	gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
	ypr[1] = atan(gx / sqrt(gy * gy + gz * gz));
	ypr[2] = atan(gy / sqrt(gx * gx + gz * gz));

	if (fabs(eulerLastX-ypr[0]) > 6) {
		eulerRotX += (eulerLastX > 0 ? 1 : -1);
	}
	if (fabs(eulerLastY-ypr[1]) > 6) {
		eulerRotY += (eulerLastY > 0 ? 1 : -1);
	}
	if (fabs(eulerLastZ-ypr[2]) > 6) {
		eulerRotZ += (eulerLastZ > 0 ? 1 : -1);
	}
	eulerLastX = ypr[0];
	eulerLastY = ypr[1];
	eulerLastZ = ypr[2];
	ypr[0] += 6.28*eulerRotX;
	ypr[1] += 6.28*eulerRotY;
	ypr[2] += 6.28*eulerRotZ;
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 *
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 *
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
 */
void FreeIMU::getYawPitchRoll(float * ypr) {
	getYawPitchRollRad(ypr);
	arr3_rad_to_deg(ypr);
}

/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
 */
void arr3_rad_to_deg(float * arr) {
	arr[0] *= 180 / M_PI;
	arr[1] *= 180 / M_PI;
	arr[2] *= 180 / M_PI;
}

/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float invSqrt(float number) {
	return pow(number, -0.5);
	// volatile long i;
	// volatile float x, y;
	// volatile const float f = 1.5F;

	// x = number * 0.5F;
	// y = number;
	// i = *(long *) &y;
	// i = 0x5f375a86 - (i >> 1);
	// y = *(float *) &i;
	// y = y * (f - (x * y * y));
	// return y;
}

