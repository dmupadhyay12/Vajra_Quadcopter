### "Vajra" - Flight Software for a Custom Quadcopter

## Premise

This repo holds all the flight software for "Vajra", a custom quadcopter running (currently) on an STM32F7 Microcontroller

It entails the following subsystems:
 - SBUS RX Receiver, interfaced with MCU over UART
 - MPU6050 6-axis IMU, interfaced with IMU over I2C
 - ESC control, used to provide motor speed control via PWM and timers
 - PID controller - uses state estimator and feedback from IMU to develop a rate controlled system (still in progress)

 ## Inspiration

 Inspiration came from my passion for aerospace and avionics since I was young. My time at the Waterloo Aerial Robotics Group (WARG) gave me experience working on flight software, and leading a large multidiscplinary team. WARG inspired me to work on my own quadcopter and firmware therein.

 ## Next Steps

 This project is still a WIP. The PID controller still needs to be tuned, and the state estimation improved further.

 Long term, I would like to add realtime telemetry for things like attitude, voltage/current, and develop a custom flight controller PCB.

 Additionally, would like to implement some better control algorithms such as LQR or MPC, and possibly implement some CV or autonomy via a connected R-Pi as well.