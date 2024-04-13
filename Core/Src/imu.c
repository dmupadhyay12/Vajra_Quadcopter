#include "imu.h"
#include <stdint.h>
#include <math.h>

uint8_t MPU6050_Init(I2C_HandleTypeDef* handle, MPU6050_t* data) {
    while (HAL_I2C_IsDeviceReady(handle, (uint8_t)MPU6050_I2C_ADDR, 10, 10) == HAL_ERROR) {
        // Wait for IMU to become ready and return HAL_OK
    }

    // Verify identity of device
    uint16_t WHO_AM_I = (uint16_t) MPU6050_WHO_AM_I;
    uint8_t readRes = 0;
    uint8_t checkDev = 0; 

    HAL_I2C_Mem_Read(handle, (uint8_t)MPU6050_I2C_ADDR, WHO_AM_I, 1, &checkDev, 1, 10);

    if (checkDev == (uint8_t)MPU6050_I_AM) {
        // Device validated to be MPU6050 IMU - now to run a variety of configurations

        /* 
        The following configurations are performed for now: 
         - Data rate set to 1 kHz
         - Accelerometer range set from -2g to +2g
         - Gyro speed set to max -250 deg/s to +250 deg/s
         - Digital low pass filter set, 94 Hz for Accelerometer, 98 Hz for Gyroscope
        */ 
        HAL_StatusTypeDef status = HAL_ERROR;
        uint8_t powerManagementState = 0;
        status = HAL_I2C_Mem_Write(handle, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_PWR_MGMT_1, 1, &powerManagementState, 1, 10);

        uint8_t sampleDivider = 8; // divide 8 kHz by 8 yields 1 kHz rate
        status = HAL_I2C_Mem_Write(handle, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_SMPLRT_DIV, 1, &sampleDivider, 1, 10); 

        uint8_t accel2GValue = 0;
        status = HAL_I2C_Mem_Write(handle, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_ACCEL_CONFIG, 1, &accel2GValue, 1, 10);

        uint8_t gyro250DegPerSec = 0;
        status = HAL_I2C_Mem_Write(handle, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_GYRO_CONFIG, 1, &gyro250DegPerSec, 1, 10);

        // Set Digital low pass (DLPF) filter settings
        uint8_t dlpfSetting = 0x02;
        status = HAL_I2C_Mem_Write(handle, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_CONFIG, 1, &dlpfSetting, 1, 10);

        // Delay 10ms and then run calibration process
        HAL_Delay(10);
        // MPU6050_Calibrate_IMU(handle, data);
        return 1;
    }
    return 0;
}

void MPU6050_Calibrate_IMU(I2C_HandleTypeDef* handle, MPU6050_t* data) {
    // Takes 1000 raw values for gyro and accelerometer and finds offsets

    int16_t accelZExpected = -1 * MPU6050_ACCE_SENS_2; // should be -g
    double accelXCumulative = 0;
    double accelYCumulative = 0;
    double accelZCumulative = 0;

    double gyroXCumulative = 0;
    double gyroYCumulative = 0;
    double gyroZCumulative = 0;

    int16_t sensorRaw[7] = {0};
    for (int i = 0; i < 1000; i++) {
        MPU6050_Get_All_Raw(handle, sensorRaw);

        accelXCumulative += sensorRaw[0] / (double)MPU6050_ACCE_SENS_2;
        accelYCumulative += sensorRaw[1] / (double)MPU6050_ACCE_SENS_2;
        accelZCumulative += sensorRaw[2] / (double)MPU6050_ACCE_SENS_2;
        
        gyroXCumulative += sensorRaw[4] / (double)MPU6050_GYRO_SENS_250;
        gyroYCumulative += sensorRaw[5] / (double)MPU6050_GYRO_SENS_250;
        gyroZCumulative += sensorRaw[6] / (double)MPU6050_GYRO_SENS_250;
    }


    double accelXAverage = accelXCumulative / 1000;
    double accelYAverage = accelYCumulative / 1000;
    double accelZAverage = accelZCumulative / 1000;

    double gyroXAverage = gyroXCumulative / 1000;
    double gyroYAverage = gyroYCumulative / 1000;
    double gyroZAverage = gyroZCumulative / 1000;

    data -> gyroXCumulative = gyroXAverage;


    // Find and update the offsets for each sensor and axis
    data -> Accel_X_OFFSET = accelXAverage;
    data -> Accel_Y_OFFSET = accelYAverage;
    data -> Accel_Z_OFFSET = accelZAverage - accelZExpected;

    data -> Gyro_X_OFFSET = gyroXAverage;
    data -> Gyro_Y_OFFSET = gyroYAverage;
    data -> Gyro_Z_OFFSET = gyroZAverage;
    
}

void MPU6050_Update_All(I2C_HandleTypeDef* handle, MPU6050_t* data) {
    // This function is called at a fixed, preset frequency in order to have an 
    // accurate computation of the gyroscopic estimate of the roll and pitch
    // Updates all accelerometer, gyro and temperature values
    // Updates accelerometer and gyroscope angle predictions

    int16_t sensorRaw[7] = {0};
    MPU6050_Get_All_Raw(handle, sensorRaw);

    data -> Ax = (sensorRaw[0]/* - data -> Accel_X_OFFSET*/) / (double)MPU6050_ACCE_SENS_2;
    data -> Ay = (sensorRaw[1]/* - data -> Accel_X_OFFSET*/) / (double)MPU6050_ACCE_SENS_2;
    data -> Az = (sensorRaw[2]/* - data -> Accel_Z_OFFSET*/) / (double)MPU6050_ACCE_SENS_2;
    
    data -> Gx = (sensorRaw[4]) / (double)MPU6050_GYRO_SENS_250 - data -> Gyro_X_OFFSET;
    data -> Gy = (sensorRaw[5]) / (double)MPU6050_GYRO_SENS_250 - data -> Gyro_Y_OFFSET;
    data -> Gz = (sensorRaw[6]) / (double)MPU6050_GYRO_SENS_250 - data -> Gyro_Z_OFFSET;

    // data -> sensorRawGx = sensorRaw[4];
    // Calculate roll and pitch estimation via gyroscope
    data -> gyroPitch += (data -> updatePeriod) * (data -> Gx);
    data -> gyroRoll += (data -> updatePeriod) * (data -> Gy);

    // Estimate roll and pitch via accelerometer
    data -> accelRoll = atan2f((data -> Ay), (data -> Az));
    data -> accelPitch = atanf(-(data -> Ax) / sqrtf(powf(data -> Ay, 2) + powf(data -> Az, 2)));

    // Apply sensor fusion algorithm (currently only complementary filter)
    MPU6050_Apply_Filter(data);

}

int16_t MPU6050_Get_Accel_Raw(I2C_HandleTypeDef* handle, axis_t axis) {
    uint8_t accelData[2] = {0};
    uint16_t accelToReturn = 0;
    if (axis == X) {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_XOUT_H, 1, accelData, 2, 10);
        accelToReturn = (accelData[0] << 8 | accelData[1]);
    } else if (axis == Y) {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_YOUT_H, 1, accelData, 2, 10);
        accelToReturn = (accelData[0] << 8 | accelData[1]); 
    } else {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_ZOUT_H, 1, accelData, 2, 10);
        accelToReturn = (accelData[0] << 8 | accelData[1]);
    }

    return accelToReturn;
}

int16_t MPU6050_Get_Gyro_Raw(I2C_HandleTypeDef* handle, axis_t axis) {
    uint8_t gyroData[2] = {0};
    int16_t gyroToReturn = 0;
    if (axis == X) {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_XOUT_H, 1, gyroData, 2, 10);
        gyroToReturn = (gyroData[0] << 8 | gyroData[1]);
    } else if (axis == Y) {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_YOUT_H, 1, gyroData, 2, 10);
        gyroToReturn = (gyroData[0] << 8 | gyroData[1]); 
    } else {
        HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_WHO_AM_I, (uint16_t)MPU6050_ACCEL_ZOUT_H, 1, gyroData, 2, 10);
        gyroToReturn = (gyroData[0] << 8 | gyroData[1]);
    }
    
    return gyroToReturn;
}

void MPU6050_Get_All_Raw(I2C_HandleTypeDef* handle, int16_t* sensorReadingsArray) {
    // write 14 bytes to byte array passed into the function
    uint8_t bytes[14] = {0};
    uint16_t address = (uint16_t) MPU6050_ACCEL_XOUT_H;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, (uint16_t)MPU6050_I2C_ADDR, address, 1, bytes, 14, 1000);
    // Write data from bytes array into sensor readings array
    // Mapped as seven element array - AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ

    sensorReadingsArray[0] = (int16_t) ((bytes[0] << 8) | (bytes[1]));
    sensorReadingsArray[1] = (int16_t) ((bytes[2] << 8) | (bytes[3]));
    sensorReadingsArray[2] = (int16_t) ((bytes[4] << 8) | (bytes[5]));
    sensorReadingsArray[3] = (int16_t) ((bytes[6] << 8) | (bytes[7]));
    sensorReadingsArray[4] = (int16_t) ((bytes[8] << 8) | (bytes[9]));
    sensorReadingsArray[5] = (int16_t) ((bytes[10] << 8) | (bytes[11]));
    sensorReadingsArray[6] = (int16_t) ((bytes[12] << 8) | (bytes[13]));
}

void MPU6050_Apply_Filter(MPU6050_t* data) {
    // Implementing basic complementary filter for now, will implement more complex
    // filtering methods in the future based on the performance of state estimator
    // and requirements (e.g. EKF, Mahoney, Madgwick, etc.)

    data -> filteredRoll = (data -> alpha) * (data -> gyroRoll) + (1 - data -> alpha) * (data -> accelRoll);
    data -> filteredPitch = (data -> alpha) * (data -> gyroPitch) + (1 - data -> alpha) * (data -> accelPitch);
} 
