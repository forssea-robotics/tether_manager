#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/LinearMath/Quaternion.h"

#include <math.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turn_counter_tester");
    ros::NodeHandle n;
    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu_input", 1000);

    ros::Rate loop_rate(1); // one Hz update for IMU simulation?

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    unsigned k = 0;

    while (ros::ok())
    {
        sensor_msgs::Imu msg;

        x = x + 0.1 * cos(theta);
        y = y + 0.1 * sin(theta);
        theta = 0.05 * k;
        ++k;

        tf::Quaternion q;
        q.setEuler(theta, 0.0, 0.0);

        msg.orientation.w = q.getW();
        msg.orientation.x = q.getX();
        msg.orientation.y = q.getY();
        msg.orientation.z = q.getZ();

        // publish
        imu_publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
