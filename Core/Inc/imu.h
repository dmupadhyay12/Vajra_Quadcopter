#ifndef IMU_H
#define IMU_H

// Driver for MPU6050 6-axis IMU, to be used for attitude determination and control systems
// of quadcopter

#include "i2c.h"
#include "stdint.h"
#include "stdbool.h"

// Define all required registers for IMU read/write
/* Default I2C address */
#define MPU6050_I2C_ADDR			(0x68 << 1)

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

typedef enum init_status {
    POWER_CONFIG_FAILED,
    SAMPLE_RATE_CONFIG_FAILED,
} init_status_t;

typedef struct {

    double Accel_X_OFFSET;
    double Accel_Y_OFFSET;
    double Accel_Z_OFFSET;
    double Ax;
    double Ay;
    double Az;

    double Gyro_X_OFFSET;
    double Gyro_Y_OFFSET;
    double Gyro_Z_OFFSET;
    double Gx;
    double Gy;
    double Gz;

    float gyroXCumulative;

    float updatePeriod; // period, in seconds at which the update function is called at

    float temperature;

    double accelRoll;
    double accelPitch;

    double gyroRoll;
    double gyroPitch;

    float  alpha;
    double filteredRoll;
    double filteredPitch;
} MPU6050_t;

typedef enum {
    X = 0,
    Y = 1,
    Z = 2,
} axis_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef* handle, MPU6050_t* data);

void MPU6050_Read_Accel_Raw(I2C_HandleTypeDef* handle, MPU6050_t* data);
void MPU6050_Read_Gyro_Raw(I2C_HandleTypeDef* handle, MPU6050_t* data);
void MPU6050_Read_Temperature(I2C_HandleTypeDef* handle, MPU6050_t* data);

int16_t MPU6050_Get_Accel_Raw(I2C_HandleTypeDef* handle, axis_t axis);
int16_t MPU6050_Get_Gyro_Raw(I2C_HandleTypeDef* handle, axis_t axis);
void MPU6050_Get_All_Raw(I2C_HandleTypeDef* handle, int16_t* sensorReadingsArray);

void MPU6050_Update_All(I2C_HandleTypeDef* handle, MPU6050_t* data);
void MPU6050_Calibrate_IMU(I2C_HandleTypeDef* handle, MPU6050_t* data);

void MPU6050_Apply_Filter(MPU6050_t* data);
#endif
