#include "kalman.h"

void kalman_initialize(kalman_t *filter, float covariance_init, float *Q, float *R, MPU6050_t *sensor)
{
    filter->sensor_data = sensor;

    // indices 0 and 3 of the 4 element array represent the top-left and bottom-right
    // elements of the matrix
    filter->P[0] = covariance_init;
    filter->P[3] = covariance_init;

    for (int i = 0; i < 3; i++)
    {
        filter->Q[i] = Q[i];
        if (i != 2)
        {
            filter->R[i] = R[i];
        }
    }
}

void kalman_predict(kalman_t *filter)
{
    // use existing pitch and roll predictions to create trigonometric expressions
    // used in the calculation of the Jacobian (to linearize about the operating point)
    float sin_p = sin(filter->phi_rad);
    float cos_p = cos(filter->phi_rad);
    float tan_th = tan(filter->theta_rad);

    float p = filter->sensor_data->Gx;
    float q = filter->sensor_data->Gy;
    float r = filter->sensor_data->Gz;

    float rate = filter->sampling_rate;

    // Update the phi and theta predictions through the relationships between p, q, r
    // and the trig expressions for phi and theta
    filter->phi_rad = filter->phi_rad + rate * (p + q * sin_p * tan_th + r * cos_p * tan_th);
    filter->theta_rad = filter->theta_rad + rate * (q * cos_p - r * sin_p);

    // Recompute trig expressions using new phi_rad and theta_rad values, for use
    // during the computation of the Jacobian
    sin_p = sin(filter->phi_rad);
    cos_p = cos(filter->phi_rad);
    float tan_p = tan(filter->phi_rad);

    float sin_th = sin(filter->theta_rad);
    float cos_th = cos(filter->theta_rad);
    tan_th = tan(filter->theta_rad);

    // Compute Jacobian to linearize the movement of the system about that operating point
    float A[4] = {0};

    // d(theta_dot)/dphi
    A[0] = -q * sin_p - r * cos_p;

    // d(theta_dot)/dtheta
    A[1] = 0;

    // d(phi_dot) / dphi
    A[2] = q * cos_p * tan_th - r * sin_p * tan_th;

    // d(phi_dot) / dtheta
    A[3] = (q * sin_p + r * cos_p) / (cos_th * cos_th);

    // Update the covariance matrix by using the formula: P = P + T * (AP + PA^T + Q)

    float P0 = filter->P[0];
    float P1 = filter->P[1];
    float P2 = filter->P[2];
    float P3 = filter->P[3];

    float covariance_differential[4];
    covariance_differential[0] = (2 * A[0] * P0 + A[1] * P2 + A[1] * P1) * rate;
    covariance_differential[1] = (A[0] * P1 + A[1] * P3 + A[2] * P0 + A[3] * P1) * rate;
    covariance_differential[2] = (A[2] * P0 + A[3] * P2 + P2 * A[0] + P3 * A[1]) * rate;
    covariance_differential[3] = (A[2] * P1 + A[3] * P3 + P2 * A[2] + P3 * A[3]) * rate;

    // Update the covariance matrix using the covariance differential matrix derived above

    filter->P[0] = P0 + covariance_differential[0];
    filter->P[1] = P1 + covariance_differential[1];
    filter->P[2] = P2 + covariance_differential[2];
    filter->P[3] = P3 + covariance_differential[3];
}

void kalman_update(kalman_t *filter)
{
    // This step is where the accelerometer values are used as a reference
    // for the sensor model

    float ax = filter->sensor_data->Ax;
    float ay = filter->sensor_data->Ay;
    float az = filter->sensor_data->Az;

    float sin_p = sin(filter->phi_rad);
    float cos_p = cos(filter->phi_rad);

    float sin_th = sin(filter->theta_rad);
    float cos_th = cos(filter->theta_rad);

    // output function for sensor model:
    float h[3] = {0};
    h[0] = 9.81 * sin_th;
    h[1] = -9.81 * cos_th * sin_p;
    h[2] = -9.81 * cos_th * cos_p;

    // Jacobian of h(x, u)
    float C[6] = {0};
    C[0] = 0.0f;
    C[1] = 9.81 * cos_th;
    C[2] = -9.81 * cos_p * cos_th;
    C[3] = 9.81 * sin_p * sin_th;
    C[4] = 9.81 * sin_p * cos_th;
    C[5] = 9.81 * sin_th * cos_p;

    // Now that we have the Jacobian, need to find the Kalman gain matrix by using the formula:
    // K = P * C' * (C*P*C' + R)^-1

    // Develop an expression for C*P*C' + R, and then invert it
    // G = C*P*C' + R, Matlab symbolic toolbox was used to make the calculation easier

    float G[9] = {0};
    G[0] = filter->R[0] + C[0] * (C[0] * filter->P[0] + C[1] * filter->P[3] + C[2] * filter->P[6]) + C[1] * (C[0] * filter->P[1] + C[1] * filter->P[4] + C[2] * filter->P[7]) + C[2] * (C[0] * filter->P[2] + C[1] * filter->P[5] + C[2] * filter->P[8]);
    G[1] = C[3] * (C[0] * filter->P[0] + C[1] * filter->P[3] + C[2] * filter->P[6]) + C[4] * (C[0] * filter->P[1] + C[1] * filter->P[4] + C[2] * filter->P[7]) + C[5] * (C[0] * filter->P[2] + C[1] * filter->P[5] + C[2] * filter->P[8]);
    G[2] = C[6] * (C[0] * filter->P[0] + C[1] * filter->P[3] + C[2] * filter->P[6]) + C[7] * (C[0] * filter->P[1] + C[1] * filter->P[4] + C[2] * filter->P[7]) + C[8] * (C[0] * filter->P[2] + C[1] * filter->P[5] + C[2] * filter->P[8]);

    G[3] = C[0] * (C[3] * filter->P[0] + C[4] * filter->P[3] + C[5] * filter->P[6]) + C[1] * (C[3] * filter->P[1] + C[4] * filter->P[4] + C[5] * filter->P[7]) + C[2] * (C[3] * filter->P[2] + C[4] * filter->P[5] + C[5] * filter->P[8]);
    G[4] = filter->R[1] + C[3] * (C[3] * filter->P[0] + C[4] * filter->P[3] + C[5] * filter->P[6]) + C[4] * (C[3] * filter->P[1] + C[4] * filter->P[4] + C[5] * filter->P[7]) + C[5] * (C[3] * filter->P[2] + C[4] * filter->P[5] + C[5] * filter->P[8]);
    G[5] = C[6] * (C[3] * filter->P[0] + C[4] * filter->P[3] + C[5] * filter->P[6]) + C[7] * (C[3] * filter->P[1] + C[4] * filter->P[4] + C[5] * filter->P[7]) + C[8] * (C[3] * filter->P[2] + C[4] * filter->P[5] + C[5] * filter->P[8]);

    G[6] = C[0] * (C[6] * filter->P[0] + C[07] * filter->P[3] + C[8] * filter->P[6]) + C[1] * (C[6] * filter->P[1] + C[7] * filter->P[4] + C[8] * filter->P[7]) + C[2] * (C[6] * filter->P[2] + C[7] * filter->P[5] + C[8] * filter->P[8]);
    G[7] = C[3] * (C[6] * filter->P[0] + C[7] * filter->P[3] + C[8] * filter->P[6]) + C[4] * (C[6] * filter->P[1] + C[7] * filter->P[4] + C[8] * filter->P[7]) + C[5] * (C[6] * filter->P[2] + C[7] * filter->P[5] + C[8] * filter->P[8]);
    G[8] = filter->R[2] + C[6] * (C[6] * filter->P[0] + C[7] * filter->P[3] + C[8] * filter->P[6]) + C[7] * (C[6] * filter->P[1] + C[7] * filter->P[4] + C[8] * filter->P[7]) + C[8] * (C[6] * filter->P[2] + C[7] * filter->P[5] + C[8] * filter->P[8]);

    // Invert G

    float G_inv[9] = {0.0};
    float det = (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8] + G[1] * G[5] * G[6] + G[2] * G[3] * G[7] - G[2] * G[4] * G[6]);

    G_inv[0] = (G[4] * G[8] - G[5] * G[7]) / det;
    G_inv[1] = -(G[1] * G[8] - G[2] * G[7]) / det;
    G_inv[2] = (G[1] * G[5] - G[2] * G[4]) / det;

    G_inv[3] = -(G[3] * G[8] - G[5] * G[6]) / det;
    G_inv[4] = (G[0] * G[8] - G[2] * G[6]) / det;
    G_inv[5] = -(G[0] * G[5] - G[2] * G[3]) / det;

    G_inv[6] = (G[3] * G[7] - G[4] * G[6]) / det;
    G_inv[7] = -(G[0] * G[7] - G[1] * G[6]) / det;
    G_inv[8] = (G[0] * G[4] - G[1] * G[3]) / det;

    // Multiply PC' by inverse of G to update the kalman gain
}