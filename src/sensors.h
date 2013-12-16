void ReadSensors();
void reset_sensor_fusion();
void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);
float Vector_Dot_Product(float vector1[3], float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
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
extern float yaw;
extern float pitch;
extern float roll;
extern long timestamp;
extern long timestamp_old;
extern float G_Dt;
