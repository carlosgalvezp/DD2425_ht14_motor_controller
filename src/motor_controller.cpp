#include <iostream>
#include <fstream>

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
#define WHEEL_RADIUS  0.05  // Wheel radius [m]
#define WHEEL_BASE    0.205    // Distance between wheels [m]

// ** Kalman Filter params
#define Q1 3.0 // 0.46     // Sensor noise for wheel 1
#define Q2 3.0 // 0.55     // Sensor noise for wheel 2
#define R0  0.2      // Process noise
#define SIGMA_0 100.0 // Initial uncertainty

class Motor_Controller
{
public:

    Motor_Controller(const ros::NodeHandle &n);
    void run();

private:
    ros::NodeHandle n_;

    double v_ref_, w_ref_;   // Command v and w from twist
    double w_l_measured_, w_r_measured_; // Read w1 and w2 from encoders

    Controller controller_l_, controller_r_;
    KalmanFilter kf1_, kf2_;

    ros::Publisher pwm_pub_;
    ros::Subscriber encoder_sub_, twist_sub_;

    std::ofstream file;

    double kp_l_, kd_l_, ki_l_, kp_r_, kd_r_, ki_r_;
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
     * @brief Saturates control signal to be in the range [-255, 255]
     * @param x
     */
    int saturate(const double& x);

    /**
     * @brief PID controllers for each wheel
     * @param PWM1 angular velocity for left wheel
     * @param PWM2 angular velocity for right wheel
     */
    void control(int& PWM_L, int& PWM_R);

    void initialize_kf();
};

int main (int argc, char* argv[])
{
    // ** Create and initialize ROS node
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;
    Motor_Controller mc(n);

    // ** Run node
    mc.run();    
}

Motor_Controller::Motor_Controller(const ros::NodeHandle& n)
    : n_(n)
{
    file.open("/home/ras/wheel.txt");
    // ** Publisher
    pwm_pub_ = n_.advertise<ras_arduino_msgs::PWM>
                              ("/arduino/pwm",QUEUE_SIZE);
    // ** Subscribers
    encoder_sub_ = n_.subscribe("/arduino/encoders", 1000,
                                &Motor_Controller::encodersCallback, this);
    twist_sub_   = n_.subscribe("/motor_controller/twist", 1000,
                                &Motor_Controller::twistCallback, this);
    // ** Get parameters
    n_.getParam("Motor_Controller/KP_L", kp_l_);
    n_.getParam("Motor_Controller/KD_L", kd_l_);
    n_.getParam("Motor_Controller/KI_L", ki_l_);

    n_.getParam("Motor_Controller/KP_R", kp_r_);
    n_.getParam("Motor_Controller/KD_R", kd_r_);
    n_.getParam("Motor_Controller/KI_R", ki_r_);

    // ** Initialize controllers
    controller_l_ = Controller(kp_l_, kd_l_, ki_l_);
    controller_r_ = Controller(kp_r_, kd_r_, ki_r_);

    // ** Initialize KalmanFilter
    //initialize_kf();
}

void Motor_Controller::run()
{
    // ** Publish data
    ros::Rate rate(PUBLISH_RATE); // 10 Hz

    while(ros::ok())
    {
        // ** Create msg
        ras_arduino_msgs::PWM msg;

        // ** Compute control commands
        control(msg.PWM2, msg.PWM1);

        // ** Publish
        pwm_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Exiting...\n";
}

void Motor_Controller::encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    // ** Get the measurement (the - sign is because we get negative data from encoders)
    double z_l = -(msg->delta_encoder1*2*M_PI)/(TICKS_PER_REV*msg->timestamp/1000.0);
    double z_r = -(msg->delta_encoder2*2*M_PI)/(TICKS_PER_REV*msg->timestamp/1000.0);

    // ** Filter with Kalman Filter
    //Eigen::VectorXd dummy_v(1), z1_v(1), z2_v(1), w1_filtered(1), w2_filtered(1);
    //Eigen::MatrixXd dummy_m(1,1);
    //dummy_v << 0;
    //dummy_m << 0;
    //z1_v << z1;
    //z2_v << z2;

    //kf1_.filter(dummy_v, z1_v, w1_filtered, dummy_m);
    //kf2_.filter(dummy_v, z2_v, w2_filtered, dummy_m);

    // ** Store the result
    //w1_measured_ = w1_filtered(0);
    //w2_measured_ = w2_filtered(0);
    w_l_measured_ = z_l;
    w_r_measured_ = z_r;
}

void Motor_Controller::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // ** Get linear and angular velocity
    v_ref_ = msg->linear.x;
    w_ref_ = msg->angular.z;
}

void Motor_Controller::control(int& PWM_L, int& PWM_R)
{
    // ** Desired angular velocities from kinematic equations
    double w_l_ref = (v_ref_ - (WHEEL_BASE/2.0) * w_ref_) / WHEEL_RADIUS;
    double w_r_ref = (v_ref_ + (WHEEL_BASE/2.0) * w_ref_) / WHEEL_RADIUS;

    ROS_INFO("Desired w: %.3f, %.3f ; Current: %.3f, %.3f\n",
             w_l_ref, w_r_ref, w_l_measured_, w_r_measured_);

    file <<w_r_measured_<< " "<< w_l_measured_<<std::endl;
    // ** Call PID controller
    controller_l_.setData(w_l_ref, w_l_measured_);
    controller_r_.setData(w_r_ref, w_r_measured_);
    PWM_L = saturate(controller_l_.computeControl());
    PWM_R = -saturate(controller_r_.computeControl()); // The motor is reversed
    ROS_INFO("Control signals (L,R): %i, %i\n", PWM_L, PWM_R);
}

int Motor_Controller::saturate(const double& x)
{
    if(x > 255)
        return 255;
    else if (x < -255)
        return -255;
    else
        return (int)x;
}
/*
void initialize_kf()
{
    Eigen::MatrixXd A(1,1);
    Eigen::MatrixXd B(1,1);
    Eigen::MatrixXd C(1,1);
    Eigen::MatrixXd R(1,1);
    Eigen::MatrixXd Q_1(1,1);
    Eigen::MatrixXd Q_2(1,1);
    Eigen::VectorXd mu0(1);
    Eigen::MatrixXd sigma0(1,1);

    A << 1;
    B << 0;
    C << 1;
    R << R0*R0;
    Q_1 << Q1*Q1;
    Q_2 << Q2*Q2;

    mu0 << 0;
    sigma0 << SIGMA_0*SIGMA_0;

    kf1_ = KalmanFilter(mu0, sigma0, A, B, C, R, Q_1);
    kf2_ = KalmanFilter(mu0, sigma0, A, B, C, R, Q_2);
}
*/
