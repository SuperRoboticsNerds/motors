#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "motors/pid.h"
#include "motors/odometry.h"
#include <sstream>
#include <math.h>
#include <cmath>

#define PI 3.14159265359

double wheel_radius = 0.05;
double wheel_base = 0.21;

//PID Values
double Kp_left = 9.0;
double Ki_left = 2.0;
double Kd_left = 0.05;

double Kp_right = 10.0;
double Ki_right = 3.0;
double Kd_right = 0.05;

double errorOld_right=0.0;
double errorOld_left=0.0;

double estimated_w_right;
double estimated_w_left;
double integral_right = 0.0;
double integral_left = 0.0;

double desired_w_right = 0.0;
double desired_w_left = 0.0;

double lastTime;
double lastTime_encoder;

ros::Publisher pwm_pub_;
ros::Publisher pid_pub_;
ros::Publisher odom_pub;
ros::Subscriber encoders_sub_;
ros::Subscriber twist_sub_;

//For the odometry
double v_tot = 0.0;
double w_tot = 0.0;


void encoder_function(const ras_arduino_msgs::Encoders encoders_msg)
    {

    double timenow = ros::Time::now().toSec();
    double dt = timenow - lastTime_encoder;
    lastTime_encoder = timenow;


    estimated_w_right = (encoders_msg.delta_encoder1*PI)/(180.0*dt);
    estimated_w_left = (encoders_msg.delta_encoder2*PI)/(180.0*dt);

    motors::odometry odom_msg;
    odom_msg.v = (estimated_w_right + estimated_w_left)*wheel_radius/2.0;
    odom_msg.w = (estimated_w_right - estimated_w_left)*wheel_radius/wheel_base;
    v_tot += odom_msg.v*dt;
    w_tot += odom_msg.w*dt;
    odom_msg.v_tot = v_tot;
    odom_msg.w_tot = w_tot;
    odom_msg.dt = dt;

    odom_pub.publish(odom_msg);
    }

void PWM_function()
    {
        ras_arduino_msgs::PWM pwm_msg;
        motors::pid pid_msg;

        double timenow = ros::Time::now().toSec();
        double dt = timenow - lastTime;
        lastTime = timenow;
        //error
        double error_right = desired_w_right - estimated_w_right;
        double error_left = desired_w_left - estimated_w_left;

        // integral = integral + Dt*error
        integral_right += Ki_right * (error_right)*dt;
        integral_left += Ki_left * (error_left)*dt;

        // derivate = (error - prerror)/Dt
        double derivative_right = Kd_right * (error_right - errorOld_right)/dt;
        double derivative_left = Kd_left * (error_left - errorOld_left)/dt;

        //Output = Kp*error +Ki*integral +kd*derivate
        pwm_msg.PWM1 = (-1.0)*(Kp_right * (error_right) + integral_right + derivative_right);
        pwm_msg.PWM2 = (Kp_left * (error_left) + integral_left + derivative_left);

        errorOld_right = error_right;
        errorOld_left = error_left;

        pwm_pub_.publish(pwm_msg);

        pid_msg.p_right = Kp_right * (error_right);
        pid_msg.i_right = integral_right;
        pid_msg.d_right = derivative_right;

        pid_msg.p_left = Kp_left * (error_left);
        pid_msg.i_left = integral_left;
        pid_msg.d_left = derivative_left;

        pid_pub_.publish(pid_msg);
    }

void twist_function(const geometry_msgs::Twist twist_msg)
    {
        desired_w_right = (twist_msg.linear.x+(wheel_base/2.0)*twist_msg.angular.z)/wheel_radius;
        desired_w_left = (twist_msg.linear.x-(wheel_base/2.0)*twist_msg.angular.z)/wheel_radius;
        if (std::fabs(desired_w_right) < 1e-5){integral_right = 0.0;}
        if (std::fabs(desired_w_left) < 1e-5){integral_left = 0.0;}
    }


bool setup(ros::NodeHandle nh){
    bool paramtest;
    nh.getParam("/has_parameters", paramtest);
    if(!paramtest){
        return false;
    }

    nh.getParam("/Kp_left", Kp_left);
    nh.getParam("/Ki_left", Ki_left);
    nh.getParam("/Kd_left", Kd_left);

    nh.getParam("/Kp_right", Kp_right);
    nh.getParam("/id_right", Ki_right);
    nh.getParam("/Kd_right", Kd_right);

    nh.getParam("/wheel_radius", wheel_radius);
    nh.getParam("/wheel_base", wheel_base);

    return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "good_motor_controller");

    ros::NodeHandle n;

    if(!setup(n)){
        return 1;
    }

    encoders_sub_ = n.subscribe("/arduino/encoders",100,encoder_function);
    twist_sub_ = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",100,twist_function);
    pwm_pub_ = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    pid_pub_ = n.advertise<motors::pid>("/pid", 100);
    odom_pub = n.advertise<motors::odometry>("/odometry",100);

    // Control @ 30 Hz
    double control_frequency = 30.0;

    ros::Rate loop_rate(control_frequency);
    lastTime =ros::Time::now().toSec();
    while(ros::ok())
    {
        PWM_function();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
