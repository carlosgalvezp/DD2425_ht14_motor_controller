#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <iostream>

#include "ros/ros.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include <geometry_msgs/Twist.h>

#include <ras_utils/controller.h>
#include <ras_utils/kalman_filter.h>

#define PUBLISH_RATE 10 // Hz
#define CONTROL_RATE 10 // Hz
#define QUEUE_SIZE 1000

// ** Robot params
#define TICKS_PER_REV 360     // Ticks per revolution for encoders
#define WHEEL_RADIUS  0.0352  // Wheel radius [m]
#define WHEEL_BASE    0.23    // Distance between wheels [m]

// ** PID Controllers
#define KP1 10.0 // 5.0
#define KD1 7.5 // 5.0
#define KI1 3.5 // 1.0

#define KP2 9.0 // 5.0
#define KD2 7.5 // 5.0
#define KI2 3.5 // 1.0

// ** Kalman Filter params
#define Q1 3.0 // 0.46     // Sensor noise for wheel 1
#define Q2 3.0 // 0.55     // Sensor noise for wheel 2
#define R0  0.2      // Process noise
#define SIGMA_0 100.0 // Initial uncertainty

double v_ref_, w_ref_;   // Command v and w from twist
double w1_measured_, w2_measured_; // Read w1 and w2 from encoders

Controller controller1_, controller2_;
KalmanFilter kf1_, kf2_;

/**
 * @brief callback function when new msg from encoders is received
 * @param msg
 */
void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);

/**
 * @brief callback function when new msg from twist is received
 * @param msg
 */
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);


/**
 * @brief PID controllers for each wheel
 * @param PWM1 angular velocity for left wheel
 * @param PWM2 angular velocity for right wheel
 */
void control(int& PWM1, int& PWM2);

/**
 * @brief Saturates control signal to be in the range [-255, 255]
 * @param x
 */
int saturate(const double& x);

void initialize_kf();


#endif // MOTOR_CONTROLLER_H
