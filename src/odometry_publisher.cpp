
#include "ros/ros.h"
#include <math.h>
#include <ras_arduino_msgs/Encoders.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

#define VEL_MULTIPLIER 65462
#define ROT_MULTIPLIER 0.001

ros::Publisher odom_pub;

//ros::Time current_time_encoder, last_time_encoder;

double B = 0.26, r_r = 0.05, r_l = 0.05;
double xpos,ypos,rot;
double v_lin,v_rot;
double aspeed,bspeed;
double dt;
double last_secs;
bool started = false;



//double deltaLeft = 0.0;
//double deltaRight = 0.0;

void WheelCallback(const ras_arduino_msgs::Encoders::ConstPtr& ticks)
{

  //first time, do nothing except initialize the time.
  if (!started){
    last_secs = ros::Time::now().toSec();
    started = true;
    return;
  }

  

  last_secs = ros::Time::now().toSec();

  aspeed = (double)(ticks->delta_encoder1);
  bspeed = -(double)(ticks->delta_encoder2);
  //dt = (double)(ticks->timestamp);
  dt =  ros::Time::now().toSec() - last_secs ;

  v_lin = dt*VEL_MULTIPLIER*(r_r*aspeed - r_l*bspeed)/2.0;
  v_rot = dt*ROT_MULTIPLIER*(r_r*aspeed + r_l*bspeed)/B;

  rot += v_rot;
  xpos += std::cos(rot)*v_lin;
  ypos += std::sin(rot)*v_lin;


  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(rot);

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = xpos;
  odom.pose.pose.position.y = ypos;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = xpos;
  odom.twist.twist.linear.y = ypos;
  odom.twist.twist.angular.z = rot;

  //publish the message
  odom_pub.publish(odom);

  //last_time_encoder = current_time_encoder;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/arduino/encoders", 100, WheelCallback);
   odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  std::cout << std::fixed << std::setprecision(9) << std::showpoint;
  
  //std::cout << secs;
    while(n.ok()){
        ros::spinOnce();
    }
}

/*

  ros::Subscriber sub = n.subscribe("wheel_encoder", 100, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate r(1.0);


    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
*/
