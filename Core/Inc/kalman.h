#include "stdio.h"
#include "stdlib.h"
#include "imu.h"
#include <math.h>

// Define struct to hold all related kalman state information

typedef struct kalman {
    // Struct representing sensor measurements from MPU6050 6-axis IMU
    MPU6050_t* sensor_data;

    // Initialize variables representing state estimates, pitch and roll respectively, in radians
    float phi_rad;
    float theta_rad;

    // Kalman filter parameters and matrices
    float sampling_rate;
    float gain;
    // Covariance matrix can be represented as an array of four floats as it is a 2x2
    // matrix, should be initialized as a diagonalized matrix
    float P[4]; 
    
    // define system noise matrix Q
    float Q[2]; // Initialize noise as 0

    // define measurement noise matrix R
    float R[3];

} kalman_t;

void kalman_initialize(kalman_t* filter, float covariance_init, float* Q, float* R, MPU6050_t* sensor);

void kalman_predict(kalman_t* filter);

void kalman_update(kalman_t* filter);