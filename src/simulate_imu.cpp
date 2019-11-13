#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_datatypes.h"

#include <math.h>

#define REFRESH_FREQ 0.1

main(int argc, char **argv)
{
  int count = 0;

  /* Use this type of message to offer 
  possibility to get other source of position
  for testing (i.e. turtlesim) */
  geometry_msgs::Pose2D simuPosition;

  ros::init(argc, argv, "simulate_imu_moving_robot");
  ros::NodeHandle n;

  /*Advertise master and get publisher*/
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_input", 1000);
  ros::Rate loop_rate(REFRESH_FREQ);
  
  /*Init fake position*/
  simuPosition.x = 0;
  simuPosition.y = 0;
  simuPosition.theta = 0;

  while (ros::ok())
  {
    /* Declare our msg*/
    sensor_msgs::Imu new_imu_msg;

    /*Assume that the deltax and deltay are
    constant during deltat which is 1/REFRESH_FREQ*/
    /* First of all... we need to recalculate new theta*/
    simuPosition.theta = REFRESH_FREQ / 2 * count;
    simuPosition.x = simuPosition.x + (REFRESH_FREQ * cos(simuPosition.theta));
    simuPosition.y = simuPosition.y + (REFRESH_FREQ * sin(simuPosition.theta));

    /* Convert position to new quaternion
    Assume that only yaw is significant */
    tf::Quaternion q = tf::createQuaternionFromYaw(simuPosition.theta);

    /* Transform to quaternions of orientation of our message*/
    new_imu_msg.orientation.w = q.getW();
    new_imu_msg.orientation.x = q.getX();
    new_imu_msg.orientation.y = q.getY();
    new_imu_msg.orientation.z = q.getZ();

    ROS_INFO_STREAM(new_imu_msg);

    /*publish our msg*/
    pub.publish(new_imu_msg);

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
