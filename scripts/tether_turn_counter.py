#!/usr/bin/env python
# coding=utf-8

from math import pi, degrees
from threading import Lock

import rclpy
from rclpy.node import Node
from numpy import round
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

# from forssea_utilities.helpers import to_angle, get_param
# from forssea_utilities.Rate import Rate

def sgn(val):
    if val < 0:
        return -1
    else:
        return 1

def to_angle(value):
    return (value + pi * sgn(value)) % (2 * pi) - pi * sgn(value)

class TurnCounter(Node):
    def __init__(self):
        super().__init__('turn_counter')
        self.total_turn_count = 0.0
        self.current_heading, self.heading_zero = None, None
        self.lock = Lock()
        # self.rate = Rate()
        
        self.pub = self.create_publisher(Float32, 'count_output', 1)
        self.create_subscription(Odometry, 'odometry/filtered', self.on_odom_received, 1)
        self.reset_server = self.create_service(Trigger, "reset_counter", self.reset)
        
        turn_count_msg = Float32()
        turn_count_msg.data = self.total_turn_count 
        self.pub.publish(turn_count_msg)
        

    def on_odom_received(self, msg):
        """
        Compute imu from sbg
        :param  sensors_msgs.msg msg: incoming msg message
        """
        self.lock.acquire()
        q = msg.pose.pose.orientation
        _, _, heading = euler_from_quaternion([q.w, q.x, q.y, q.z])
        self.get_logger().debug("------received odom with heading: {:.4f}°".format(degrees(heading)))
        if self.heading_zero is not None:
            delta = to_angle(heading - self.current_heading)
            turn_inc = delta / (2 * pi)
            self.total_turn_count += turn_inc
            self.get_logger().debug("New increment is: {:.4f}° ({:.4f} turns)".format(degrees(delta), turn_inc))
            self.get_logger().debug("Total turn count: {:.4f} ({:.4f})".format(self.total_turn_count, degrees(self.total_turn_count * 2 * pi)))
        else:
            self.get_logger().debug("initializing Init_heading to: {:.4f}°".format(degrees(heading)))
            self.heading_zero = heading
        self.current_heading = heading
        turn_count_msg = Float32()
        turn_count_msg.data = self.total_turn_count 
        self.pub.publish(turn_count_msg)
        self.lock.release()

    def reset(self, req):
        self.lock.acquire()
        self.get_logger().debug("reset called")
        self.heading_zero, self.current_heading = None, None
        self.total_turn_count = 0.0
        turn_count_msg = Float32()
        turn_count_msg.data = self.total_turn_count 
        self.pub.publish(turn_count_msg)
        self.lock.release()
        return True, ''

    # def work(self):
    #     while not rclpy.is_shutdown():
    #         self.pub.publish(Float32(-self.total_turn_count))
    #         self.rate.sleep()

def main():
    rclpy.init()
    tc = TurnCounter()
    rclpy.spin(tc)
    # tc.work()
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()