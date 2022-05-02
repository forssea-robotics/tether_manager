#!/usr/bin/env python
# coding=utf-8

from math import pi, degrees
from threading import Lock

import rclpy
from numpy import round
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from forssea_utilities.helpers import to_angle, get_param
from forssea_utilities.Rate import Rate


class TurnCounter(object):
    def __init__(self):
        self.node = rclpy.Node.__init__('turn_counter')
        self.total_turn_count = 0.0
        self.current_heading, self.heading_zero = None, None
        self.lock = Lock()
        self.rate = Rate()
        self.pub = self.node.create_publisher(Float32, 'count_output', queue_size=1, latch=True)
        self.pub.publish(self.total_turn_count)

        self.node.create_subscription(Odometry, 'odometry/filtered', self.on_odom_received)

        self.reset_server = rclpy.Service("reset_counter", Trigger, self.reset)

    def on_odom_received(self, msg):
        """
        Compute imu from sbg
        :param  sensors_msgs.msg msg: incoming msg message
        """
        self.lock.acquire()
        q = msg.pose.pose.orientation
        _, _, heading = quat2euler([q.w, q.x, q.y, q.z])
        self.node.get_logger().debug("------received odom with heading: {:.4f}°".format(degrees(heading)))
        if self.heading_zero is not None:
            delta = to_angle(heading - self.current_heading)
            turn_inc = delta / (2 * pi)
            self.total_turn_count += turn_inc
            self.node.get_logger().debug("New increment is: {:.4f}° ({:.4f} turns)".format(degrees(delta), turn_inc))
            self.node.get_logger().debug("Total turn count: {:.4f} ({:.4f})".format(self.total_turn_count, degrees(self.total_turn_count * 2 * pi)))
        else:
            self.node.get_logger().debug("initializing Init_heading to: {:.4f}°".format(degrees(heading)))
            self.heading_zero = heading
        self.current_heading = heading
        self.pub.publish(Float32(self.total_turn_count))
        self.lock.release()

    def reset(self, req):
        self.lock.acquire()
        self.node.get_logger().debug("reset called")
        self.heading_zero, self.current_heading = None, None
        self.total_turn_count = 0.0
        self.pub.publish(Float32(self.total_turn_count))
        self.lock.release()
        return True, ''

    def work(self):
        while not rclpy.is_shutdown():
            self.pub.publish(Float32(-self.total_turn_count))
            self.rate.sleep()


if __name__ == "__main__":
    tc = TurnCounter()
    tc.work()