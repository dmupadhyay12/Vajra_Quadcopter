#include "pid.h"

void initialize_controller(pid_controller_t* controller, float output_max, float output_min, float kp_in, float ki_in, float kd_in, float setpoint) {
    controller -> output_max = output_max;
    controller -> output_min = output_min;
    controller -> kp = kp_in;
    controller -> ki = ki_in;
    controller -> kd = kd_in;
    controller -> setpoint = setpoint;
    controller -> error_accumulation = 0;
}

void update_controller(pid_controller_t* controller, float setpoint_new, float updated_input) {
    controller -> setpoint = setpoint_new; 
    controller -> curr_state = updated_input;
}

float compute_controller(pid_controller_t* controller) {
    // Based on the setpoint and input, compute the error, apply the PID constants, and 
    // update the error accumulation, previous differential

    float curr_error = controller -> setpoint - controller -> curr_state;

    float p_gain = curr_error * (controller -> kp);
    controller -> error_accumulation += curr_error * (controller -> ki); 
    float d_term = (curr_error - controller -> prev_error) * controller -> kd;

    controller -> prev_error = curr_error;

    return p_gain + controller -> error_accumulation + d_term;
}