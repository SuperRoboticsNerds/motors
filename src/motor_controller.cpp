#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <math.h>





class MotorcontrollerNode
{
public:

    ros::NodeHandle n;
    ros::Publisher pwm_pub_;
    ros::Subscriber encoders_sub_;
    ros::Subscriber twist_sub_;

    MotorcontrollerNode()
    {
        n = ros::NodeHandle("~");
        //motor_controller_ = new KobukiMotors();

        wheel_radius_ = 0.05;
        base_ = 0.208;
	
	alpha1=4.0;
	beta1 = 0.4;

	alpha2=6.0;
	beta2 = 0.6;



    Kp_left=7.65;
    Ki_left=3.8;
    Kd_left=0.15;
        
    Kp_right=8.2;
    Ki_right=4.0;
    Kd_right=0.1;

/*
	Kp_right=2.5;
    Ki_right=0.1;
    Kd_right=0.1;
    	
	Kp_left=3.1;
    Ki_left=0.15;
    Kd_left=0.15;
*/
	errorOld_right=0.0;
	errorOld_left=0.0;
	integral_right=0.0;
	integral_left=0.0;
    	
        //estimated_w_right;
        //estimated_w_left;
        encoders_sub_ = n.subscribe("/arduino/encoders",1,&MotorcontrollerNode::encoder_function,this);
        twist_sub_ = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&MotorcontrollerNode::twist_function,this);
        pwm_pub_ = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    }

    ~MotorcontrollerNode()
    {
        //delete motor_controller_;
    }

void encoder_function(const ras_arduino_msgs::Encoders encoders_msg)
    {// degree/millisecond
	//_right
	  estimated_w_right = (encoders_msg.delta_encoder1*2*M_PI*10)/(360);

	//_left
	  estimated_w_left = (encoders_msg.delta_encoder2*2*M_PI*10)/(360);

	//estimated_w= ((estimated_w_left-estimated_w_right)/base_)*wheel_radius_;
    }

void PWM_function()
    {
    //wheel one i gthe _left one. Wheel two is the _right
	//desired_w
	//integral_right += beta1*(desired_w_right - estimated_w_right);
	//integral_left += beta2*(desired_w_left - estimated_w_left);
	//pwm_msg.PWM1 =-1*( alpha1*(desired_w_right - estimated_w_right) + integral_right );
	//pwm_msg.PWM2 = alpha2*(desired_w_left - estimated_w_left) + integral_left;
	//pwm_msg.PWM1 = estimated_w_left;
    	//pwm_msg.PWM2 = desired_w_left;

	//error
	error_right= desired_w_right - estimated_w_right;
	error_left= desired_w_left - estimated_w_left;

	// integral = integral + Dt*error
	integral_right += Ki_right * (error_right)*0.10;//0.1;
	integral_left += Ki_left * (error_left)*0.10;//0.1;

	// derivate = (error - prerror)/Dt
	derivative_right = Kd_right * (error_right - errorOld_right)/0.10;//0.1;//Dt
	derivative_left = Kd_left * (error_left - errorOld_left)/0.10;//0.1;//Dt
	
	//Output = Kp*error +Ki*integral +kd*derivate
	pwm_msg.PWM1 = (-1)*(Kp_right * (error_right) + integral_right+derivative_right) ;
	pwm_msg.PWM2 = (Kp_left * (error_left) + integral_left+derivative_left);
	//pwm_msg.PWM1 = estimated_w_left;
    	//pwm_msg.PWM2 = desired_w_left;

	errorOld_right=error_right;
	errorOld_left=error_left;





	ROS_INFO("%i,%i",pwm_msg.PWM1, pwm_msg.PWM2);
	pwm_pub_.publish(pwm_msg);
    }
void twist_function(const geometry_msgs::Twist twist_msg)
    {
	desired_w_right = ((twist_msg.linear.x)+(base_/2)*twist_msg.angular.z)/wheel_radius_;

	desired_w_left = ((twist_msg.linear.x)-(base_/2)*twist_msg.angular.z)/wheel_radius_;
     }

private:
  //  KobukiMotors *motor_controller_;

    // [0] corresponds to _right wheel, [1] corresponds to _left wheel

    double wheel_radius_;
    double base_;

    double alpha1;
    double alpha2;

    double beta1;
    double beta2;
    ras_arduino_msgs::PWM pwm_msg;
   

   //variable declaration for motor one
    double estimated_w_right;
    double desired_w_right;
    double integral_right;
    double derivative_right;
    double error_right;
    double errorOld_right;
    double Ki_right;
    double Kd_right;
    double Kp_right;


    //variable declaration for motor two
    double estimated_w_left;
    double desired_w_left;
    double integral_left;
    double derivative_left;
    double error_left;
    double errorOld_left;
    double Ki_left;
    double Kd_left;
    double Kp_left;

};


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "closed_loop_controller");
    MotorcontrollerNode motor_controller_node;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
	// while (ros::ok())
    while(motor_controller_node.n.ok())
    {
    motor_controller_node.PWM_function();

		
 

    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}
