#include <Wire.h>
#include <Servo.h> 
#include <I2C.h>
#include <L3G4200D.h>
#include <PID_v1.h>

Servo motor_one;

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT_BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 20    // in milliseconds

// SENSOR CALIBRATION
/*****************************************************************/
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

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX    Y_MIN/Y_MAX    Z_MIN/Z_MAX"
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X    .../OFFSET_Y    .../OFFSET_Z
#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)


// Altymeter
#define ALT_SEA_LEVEL_PRESSURE 102133

/*
// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00    -254.00/284.00    -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00    -516.00/568.00    -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82    102.00/100.41    -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)
*/

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)    // *pi/180
#define TO_DEG(x) (x * 57.2957795131)    // *180/pi

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

// Euler angles
float yaw, pitch, roll;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void ReadSensors() {
    Read_Pressure();
    Read_Gyro(); // Read gyroscope
    Read_Accel(); // Read accelerometer
    Read_Magn(); // Read magnetometer
    ApplySensorMapping();    
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f};

    ReadSensors();
    
    timestamp = millis();
    
    // GET PITCH
    // Using y-z-plane-component/x-component of gravity vector
    pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
                
    // GET ROLL
    // Compensate pitch of gravity vector 
    Vector_Cross_Product(temp1, Accel_Vector, xAxis);
    Vector_Cross_Product(temp2, xAxis, temp1);
    // Normally using x-z-plane-component/y-component of compensated gravity vector
    // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);
    
    // GET YAW
    Compass_Heading();
    yaw = MAG_Heading;
    
    // Init rotation matrix
    init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

/* This file is part of the Razor AHRS Firmware */

// DCM algorithm

/**************************************************/
void Normalize(void)
{
    float error=0;
    float temporary[3][3];
    float renorm=0;
    
    error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

    Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
    
    Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
    
    Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
    Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
    Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
    Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift. 
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;
    
    
    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);    //    

    Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
    Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
    
    Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);         
    
    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading
 
    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);    //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding    Proportional.
    
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void Matrix_update(void)
{    
    Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);    //adding proportional term
    Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
    
#if DEBUG_NO_DRIFT_CORRECTION
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
    Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
    Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
    Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
    Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
    Update_Matrix[2][2]=0;
#else // Use drift correction
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
    Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
    Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
    Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
    Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
    Update_Matrix[2][2]=0;
#endif

    Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

    for(int x=0; x<3; x++) //Matrix Addition (update)
    {
        for(int y=0; y<3; y++)
        {
            DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
        } 
    }
}

void Euler_angles(void)
{
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

/* This file is part of the Razor AHRS Firmware */

// Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3], float vector2[3])
{
    float op=0;
    
    for(int c=0; c<3; c++)
    {
        op+=vector1[c]*vector2[c];
    }
    
    return op; 
}

// Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
    vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
    vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
    vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

// Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn[c]*scale2; 
    }
}

// Adds two vectors
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
    for(int c=0; c<3; c++)
    {
        vectorOut[c]=vectorIn1[c]+vectorIn2[c];
    }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
    float op[3]; 
    for(int x=0; x<3; x++)
    {
        for(int y=0; y<3; y++)
        {
            for(int w=0; w<3; w++)
            {
             op[w]=a[x][w]*b[w][y];
            } 
            mat[x][y]=0;
            mat[x][y]=op[0]+op[1]+op[2];
            
            float test=mat[x][y];
        }
    }
}

// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
    float c1 = cos(roll);
    float s1 = sin(roll);
    float c2 = cos(pitch);
    float s2 = sin(pitch);
    float c3 = cos(yaw);
    float s3 = sin(yaw);

    // Euler angles, right-handed, intrinsic, XYZ convention
    // (which means: rotate around body axes Z, Y', X'') 
    m[0][0] = c2 * c3;
    m[0][1] = c3 * s1 * s2 - c1 * s3;
    m[0][2] = s1 * s3 + c1 * c3 * s2;

    m[1][0] = c2 * s3;
    m[1][1] = c1 * c3 + s1 * s2 * s3;
    m[1][2] = c1 * s2 * s3 - c3 * s1;

    m[2][0] = -s2;
    m[2][1] = c2 * s1;
    m[2][2] = c1 * c2;
}

/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS    ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS    ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 

void I2C_Init()
{
    Wire.begin();
}

void Accel_Init()
{
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

// Reads x, y and z accelerometer registers
void Read_Accel()
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
        accel[0] = (((int16_t) buff[1]) << 8) | buff[0];    // Y axis (internal sensor x axis)
        accel[1] = (((int16_t) buff[3]) << 8) | buff[2];    // X axis (internal sensor y axis)
        accel[2] = (((int16_t) buff[5]) << 8) | buff[4];    // Z axis (internal sensor z axis)
    }
    else
    {
        num_accel_errors++;
    }
}

void Magn_Init()
{
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

void Read_Magn()
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
        magnetom[0] = (((int16_t) buff[0]) << 8) | buff[1];
        magnetom[1] = (((int16_t) buff[4]) << 8) | buff[5];
        magnetom[2] = (((int16_t) buff[2]) << 8) | buff[3];
    }
    else
    {
        num_magn_errors++;
    }
}

#include <L3G4200D.h>

L3G4200D gyroscope;

void Gyro_Init()
{
    gyroscope.enableDefault();
    gyroscope.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
    gyroscope.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
    gyroscope.read();
    gyro[0] = gyroscope.g.x;
    gyro[1] = gyroscope.g.y;
    gyro[2] = gyroscope.g.z;
}

#include <Adafruit_BMP085.h>

Adafruit_BMP085 press;

void Pressure_Init() {
    press.begin(BMP085_ULTRALOWPOWER);
}

void Read_Pressure() {
    //press.update();
    temperature = press.readTemperature();
    pressure = 0.01f * press.readPressure();
    altitude = press.readAltitude(ALT_SEA_LEVEL_PRESSURE);
}



// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
        // Magnetometer axis mapping
        Magn_Vector[1] = -magnetom[0];
        Magn_Vector[0] = -magnetom[1];
        Magn_Vector[2] = -magnetom[2];

        // Magnetometer values mapping
        Magn_Vector[0] -= MAGN_X_OFFSET;
        Magn_Vector[0] *= MAGN_X_SCALE;
        Magn_Vector[1] -= MAGN_Y_OFFSET;
        Magn_Vector[1] *= MAGN_Y_SCALE;
        Magn_Vector[2] -= MAGN_Z_OFFSET;
        Magn_Vector[2] *= MAGN_Z_SCALE;
    
        // Accelerometer axis mapping
        Accel_Vector[1] = accel[0];
        Accel_Vector[0] = accel[1];
        Accel_Vector[2] = accel[2];

        // Accelerometer values mapping
        Accel_Vector[0] -= ACCEL_X_OFFSET;
        Accel_Vector[0] *= ACCEL_X_SCALE;
        Accel_Vector[1] -= ACCEL_Y_OFFSET;
        Accel_Vector[1] *= ACCEL_Y_SCALE;
        Accel_Vector[2] -= ACCEL_Z_OFFSET;
        Accel_Vector[2] *= ACCEL_Z_SCALE;
        
        // Gyroscope axis mapping
        Gyro_Vector[1] = -gyro[0];
        Gyro_Vector[0] = -gyro[1];
        Gyro_Vector[2] = -gyro[2];

        // Gyroscope values mapping
        Gyro_Vector[0] -= GYRO_X_OFFSET;
        Gyro_Vector[0] *= GYRO_X_SCALE;
        Gyro_Vector[1] -= GYRO_Y_OFFSET;
        Gyro_Vector[1] *= GYRO_Y_SCALE;
        Gyro_Vector[2] -= GYRO_Z_OFFSET;
        Gyro_Vector[2] *= GYRO_Z_SCALE;
}

void Compass_Heading()
{
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    
    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);
    
    // Tilt compensated magnetic field X
    mag_x = Magn_Vector[0]*cos_pitch + Magn_Vector[1]*sin_roll*sin_pitch + Magn_Vector[2]*cos_roll*sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = Magn_Vector[1]*cos_roll - Magn_Vector[2]*sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-mag_y, mag_x);
}

double Setpoint, Input, Output;
//                                    P, I, D
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

void setup()
{
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

    // Turn the pid on
    myPID.SetMode(AUTOMATIC);
    // Set the PIds limits
    myPID.SetOutputLimits(40, 120);
 
    motor_one.attach(10);
    
    motor_one.write(20);
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
        myPID.Compute();
        Serial.print("speed: "); Serial.print(Output/2); Serial.print(";    "); Serial.println();
        motor_one.write(Output/2);
        
        delay(20);
    }
}

