#!/usr/bin/env python
# coding=utf-8

from math import pi, degrees
from threading import Lock

import rospy
from numpy import round
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from forssea_utilities.helpers import to_angle, get_param
from forssea_utilities.Rate import Rate


class TurnCounter(object):
    def __init__(self):
        rospy.init_node('turn_counter')
        self.total_turn_count = 0.0
        self.current_heading, self.heading_zero = None, None
        self.lock = Lock()
        self.rate = Rate()
        self.pub = rospy.Publisher('count_output', Float32, queue_size=1, latch=True)
        self.pub.publish(self.total_turn_count)

        rospy.Subscriber('odometry/filtered', Odometry, self.on_odom_received)

        self.reset_server = rospy.Service("reset_counter", Trigger, self.reset)

    def on_odom_received(self, msg):
        """
        Compute imu from sbg
        :param  sensors_msgs.msg msg: incoming msg message
        """
        self.lock.acquire()
        q = msg.pose.pose.orientation
        _, _, heading = quat2euler([q.w, q.x, q.y, q.z])
        rospy.logdebug("------received odom with heading: {:.4f}°".format(degrees(heading)))
        if self.heading_zero is not None:
            delta = to_angle(heading - self.current_heading)
            turn_inc = delta / (2 * pi)
            self.total_turn_count += turn_inc
            rospy.logdebug("New increment is: {:.4f}° ({:.4f} turns)".format(degrees(delta), turn_inc))
            rospy.logdebug("Total turn count: {:.4f} ({:.4f})".format(self.total_turn_count, degrees(self.total_turn_count * 2 * pi)))
        else:
            rospy.logdebug("initializing Init_heading to: {:.4f}°".format(degrees(heading)))
            self.heading_zero = heading
        self.current_heading = heading
        self.pub.publish(Float32(self.total_turn_count))
        self.lock.release()

    def reset(self, req):
        self.lock.acquire()
        rospy.logdebug("reset called")
        self.heading_zero, self.current_heading = None, None
        self.total_turn_count = 0.0
        self.pub.publish(Float32(self.total_turn_count))
        self.lock.release()
        return True, ''

    def work(self):
        while not rospy.is_shutdown():
            self.pub.publish(Float32(-self.total_turn_count))
            self.rate.sleep()


if __name__ == "__main__":
    tc = TurnCounter()
    tc.work()