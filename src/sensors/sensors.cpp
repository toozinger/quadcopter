#include "sensors.h"

#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS    ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS    ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 

// Altimeter
#define ALT_SEA_LEVEL_PRESSURE 102133
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration

// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX    Y_MIN/Y_MAX    Z_MIN/Z_MAX"
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX    Y_MIN/Y_MAX    Z_MIN/Z_MAX"
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

void Sensors::setupSensors() {
	{	// Init I2C
		Wire.begin();
	}

	{	// Init Accel
		Wire.beginTransmission(ACCEL_ADDRESS);
	    WIRE_SEND(0x2D);    // Power register
	    WIRE_SEND(0x08);    // Measurement mode
	    Wire.endTransmission();
	    delay(5);
	    Wire.beginTransmission(ACCEL_ADDRESS);
	    WIRE_SEND(0x31);    // Data format register
	    WIRE_SEND(0x08);    // Set to full resolution
	    Wire.endTransmission();
	    delay(5);
	    
	    // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
	    Wire.beginTransmission(ACCEL_ADDRESS);
	    WIRE_SEND(0x2C);    // Rate
	    WIRE_SEND(0x09);    // Set to 50Hz, normal operation
	    Wire.endTransmission();
	    delay(5);
	}

	{	// Init Magn
		Wire.beginTransmission(MAGN_ADDRESS);
	    WIRE_SEND(0x02); 
	    WIRE_SEND(0x00);    // Set continuous mode (default 10Hz)
	    Wire.endTransmission();
	    delay(5);

	    Wire.beginTransmission(MAGN_ADDRESS);
	    WIRE_SEND(0x00);
	    WIRE_SEND(0b00011000);    // Set 50Hz
	    Wire.endTransmission();
	    delay(5);
	}

	{	// Init gyro
	    gyroscope.enableDefault();
	    gyroscope.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
	    gyroscope.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
	}

	{	// Init pressure
		press.begin(BMP085_ULTRALOWPOWER);
	}

	gyroZero.x = gyroZero.y = gyroZero.z = 0;
	for (float i = 0; i<25; i++) {
		gyroscope.read();
		gyroZero.x += (float) gyroscope.g.x * (float) (2000.0 / 32768.0);
		gyroZero.y += (float) gyroscope.g.y * (float) (2000.0 / 32768.0);
		gyroZero.z += (float) gyroscope.g.z * (float) (2000.0 / 32768.0);
		delay(OUTPUT_DATA_INTERVAL);
	}
	gyroZero.x /= 25.0;
	gyroZero.y /= 25.0;
	gyroZero.z /= 25.0;
	Serial.println("Zeroed gyro");

	lastRead = millis();
}

void Sensors::readSensors() {
	if (millis() - lastRead > OUTPUT_DATA_INTERVAL) {
		{	// Read gyro
			gyroscope.read();
			gyro.x = (float) gyroscope.g.x * (float) (2000.0 / 32768.0);
			gyro.x -= gyroZero.x;
			gyro.y = (float) gyroscope.g.y * (float) (2000.0 / 32768.0);
			gyro.y -= gyroZero.y;
			gyro.z = (float) gyroscope.g.z * (float) (2000.0 / 32768.0);
			gyro.z -= gyroZero.z;
		}
		{	// Read pressure
			temperature = press.readTemperature();
		    pressure = 0.01f * press.readPressure();
		    altitude = press.readAltitude(ALT_SEA_LEVEL_PRESSURE);
		}
		{
		    int i = 0;
		    byte buff[6];
		 
		    Wire.beginTransmission(MAGN_ADDRESS); 
		    WIRE_SEND(0x03);    // Send address to read from
		    Wire.endTransmission();
		    
		    Wire.beginTransmission(MAGN_ADDRESS); 
		    Wire.requestFrom(MAGN_ADDRESS, 6);    // Request 6 bytes
		    while(Wire.available())    // ((Wire.available())&&(i<6))
		    { 
		        buff[i] = WIRE_RECEIVE();    // Read one byte
		        i++;
		    }
		    Wire.endTransmission();
		    
		    if (i == 6)    // All bytes received?
		    {
		        // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		        magn.x = (((int16_t) buff[0]) << 8) | buff[1];
		        magn.x -= MAGN_X_OFFSET;
		        magn.x *= MAGN_X_SCALE;
		        
		        magn.y = (((int16_t) buff[4]) << 8) | buff[5];
		        magn.y -= MAGN_Y_OFFSET;
		        magn.y *= MAGN_Y_SCALE;

		        magn.z = (((int16_t) buff[2]) << 8) | buff[3];
		        magn.z -= MAGN_Z_OFFSET;
		        magn.z *= MAGN_Z_SCALE;
		    } else {
		    	Serial.println("Magn Error");
		    }
		}
		{
			    int i = 0;
			    byte buff[6];
			    
			    Wire.beginTransmission(ACCEL_ADDRESS); 
			    WIRE_SEND(0x32);    // Send address to read from
			    Wire.endTransmission();
			    
			    Wire.beginTransmission(ACCEL_ADDRESS);
			    Wire.requestFrom(ACCEL_ADDRESS, 6);    // Request 6 bytes
			    while(Wire.available())    // ((Wire.available())&&(i<6))
			    { 
			        buff[i] = WIRE_RECEIVE();    // Read one byte
			        i++;
			    }
			    Wire.endTransmission();
			    
			    if (i == 6)    // All bytes received?
			    {
			        accel.x = (((int16_t) buff[1]) << 8) | buff[0];    // Y axis (internal sensor x axis)
			        accel.x -= ACCEL_X_OFFSET;
			        accel.x *= ACCEL_X_SCALE;

			        accel.y = (((int16_t) buff[3]) << 8) | buff[2];    // X axis (internal sensor y axis)
			        accel.y -= ACCEL_Y_OFFSET;
			        accel.y *= ACCEL_Y_SCALE;
			        
			        accel.z = (((int16_t) buff[5]) << 8) | buff[4];    // Z axis (internal sensor z axis)
			        accel.z -= ACCEL_Z_OFFSET;
			        accel.z *= ACCEL_Z_SCALE;
			        
			    } else {
		    		Serial.println("Accel Error");
		    	}
		}
		lastRead = millis();
	}
}