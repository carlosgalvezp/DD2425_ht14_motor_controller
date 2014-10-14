#include "motor_controller.h"
#include <fstream>
std::ofstream file;

int main (int argc, char* argv[])
{
    file.open("/home/carlos/encoder_data2.txt");
    // ** Create and initialize ROS node
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    // ** Publisher
    ros::Publisher pwm_pub = n.advertise<ras_arduino_msgs::PWM>
                              ("/kobuki/pwm",QUEUE_SIZE);
    // ** Subscribers
    ros::Subscriber encoder_sub = n.subscribe("/kobuki/encoders", 1000, encodersCallback);
    ros::Subscriber twist_sub   = n.subscribe("/motor_controller/twist", 1000, twistCallback);

    // ** Initialize controllers
    controller1_ = Controller(KP1, KD1, KI1);
    controller2_ = Controller(KP2, KD2, KI2);

    // ** Initialize KalmanFilter
    initialize_kf();

    // ** Publish data
    ros::Rate rate(PUBLISH_RATE); // 10 Hz

    while(ros::ok())
    {
        // ** Create msg
        ras_arduino_msgs::PWM msg;

        // ** Compute control commands
        control(msg.PWM1, msg.PWM2);

        // ** Publish
        pwm_pub.publish(msg);

        // ** Sleep
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Exiting...\n";
    file.close();
}

void encodersCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    // ** Get the measurement
    double z1 = (msg->delta_encoder1*2*M_PI*CONTROL_RATE)/(TICKS_PER_REV);
    double z2 = (msg->delta_encoder2*2*M_PI*CONTROL_RATE)/(TICKS_PER_REV);

    // ** Filter with Kalman Filter
    Eigen::VectorXd dummy_v(1), z1_v(1), z2_v(1), w1_filtered(1), w2_filtered(1);
    Eigen::MatrixXd dummy_m(1,1);
    dummy_v << 0;
    dummy_m << 0;
    z1_v << z1;
    z2_v << z2;

    kf1_.filter(dummy_v, z1_v, w1_filtered, dummy_m);
    kf2_.filter(dummy_v, z2_v, w2_filtered, dummy_m);

    // ** Store the result
    w1_measured_ = w1_filtered(0);
    w2_measured_ = w2_filtered(0);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // ** Get linear and angular velocity
    v_ref_ = msg->linear.x;
    w_ref_ = msg->angular.z;
}

void control(int& PWM1, int& PWM2)
{
    // ** Desired angular velocities from kinematic equations
    double w1_ref = (v_ref_ - (WHEEL_BASE/2.0) * w_ref_) / WHEEL_RADIUS;
    double w2_ref = (v_ref_ + (WHEEL_BASE/2.0) * w_ref_) / WHEEL_RADIUS;

    ROS_INFO("Desired w: %.3f, %.3f ; Current: %.3f, %.3f\n",
             w1_ref, w2_ref, w1_measured_, w2_measured_);

//    std::cout << w1_ref << " " << w1_measured_ << " " << w2_ref << " " << w2_measured_ << std::endl;

//    file << w1_ref << " " << w1_measured_ << " " << w2_ref << " " << w2_measured_ << std::endl;

    // ** Call PID controller
    controller1_.setData(w1_ref, w1_measured_);
    controller2_.setData(w2_ref, w2_measured_);
    PWM1 = saturate(controller1_.computeControl());
    PWM2 = saturate(controller2_.computeControl());

    ROS_INFO("Control signals: %i, %i\n", PWM1, PWM2);
}

int saturate(const double& x)
{
    if(x > 255)
        return 255;
    else if (x < -255)
        return 255;
    else
        return (int)x;
}

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
