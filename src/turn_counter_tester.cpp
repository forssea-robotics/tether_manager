#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

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

        // TODO update msg.orientation passing from (x, y, theta) to a quaternion
        msg.orientation.w = 0.0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;

        // publish
        imu_publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
