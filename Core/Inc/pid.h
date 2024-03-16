// #include "stdint.h"
// #include "stdbool.h"

// typedef struct pid_controller {

//     float *input; // current process value
//     float *output; // corrected output
//     float *setpoint; // target value for this particular controller 

//     float kp;
//     float ki;
//     float kd;

//     // Minimum and maximum outputs
//     float output_max;
//     float output_min;

//     // Terms for I and D
//     float error_accumulation; // For integral term
//     float input_previous; // For derivative term

//     float control_loop_period; // since loop is run at constant frequency

//     bool is_enabled; // flag to enable or disable controller

// } pid_controller_t;

// // Function definitions

// /**
//  * @brief Initializes PID controller
//  * 
//  * @param controller --> Pointer to PID struct, will populate required fields
//  * @param output_max  --> output maximum
//  * @param output_min --> output minimum
//  * @param kp_in --> proportional term
//  * @param ki_in --> integral term
//  * @param kd_in --> derivative term
//  * @param setpoint --> initial setpoint for controller
//  *  
// */
// void initialize_controller(pid_t* controller, float output_max, float output_min, float kp_in, float ki_in, float kd_in, float setpoint);


// /**
//  * @brief update PID setpoint based on new changes
//  * 
//  * @param controller --> Pointer to PID struct, whose setpoint to be updated
//  * @param setpoint_new --> New setpoint to be updated
//  * @param input_new --> New input/state of system
//  * 
// */
// void update_controller(pid_t* controller, float setpoint_new, float input_new);

// /**
//  * @brief computes output based on input and set of outputs - effectively runs the PID control algorithm
//  * 
//  * @param controller --> Pointer to PID struct whose controller is to be run
// */
// void compute_controller(pid_t* controller);

// /**
//  * @brief updates PID values for future expansion of OTA tuning
//  * 
//  * @param controller --> Pointer to PID struct whose controller is to be updated
//  * @param kp_new --> New P value
//  * @param ki_new --> New I value
//  * @param kd_new --> New D value
// */
// void update_pid_controller(pid_t* controller, float kp_new, float ki_new, float kd_new);
