#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include <math.h>

int main(int argc, char **argv)
{
  /* code */
  ros::init(argc, argv, "simulate_imu_robot_moving");

  ros::NodeHandle n;

  /*Advertise master and get publisher*/
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_input", 1000);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

    /* Declare our msg*/
    sensor_msgs::Imu new_imu_msg;
    float x = 0, y = 0, theta = 0;
    int count = 0;

    tf::Quaternion q;
    double yaw;
    q = tf::createQuaternionFromRPY(0, 0, theta);

    /* Craft our new message
    according to august*/
    x = x + (0.1 * cos(theta));
    y = y + (0.1 * sin(theta));
    theta = 0.05 * count;

    /* Transform to quaternions of orientation of our message*/
    new_imu_msg.orientation.w = q.w;
    new_imu_msg.orientation.x = q.x;
    new_imu_msg.orientation.y = q.y;
    new_imu_msg.orientation.z = q.z;

    ROS_INFO("New value are x=%f y=%f theta=%f at count=%d", x, y, theta, count);

    /*publish our msg*/
    pub.publish(new_imu_msg);

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
