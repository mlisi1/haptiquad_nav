#!/usr/bin/env python3
"""
Thin Twist → TwistStamped adapter for twist_mux.

twist_mux expects TwistStamped; move_base publishes plain Twist.
This node bridges the two without any modification to the velocity values.
"""
import rospy
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper:
    def __init__(self):
        self._frame = rospy.get_param('~frame_id', 'base')
        self._pub   = rospy.Publisher('cmd_vel_out', TwistStamped, queue_size=1)
        rospy.Subscriber('cmd_vel_in', Twist, self._cb, queue_size=1)

    def _cb(self, msg):
        out = TwistStamped()
        out.header.stamp    = rospy.Time.now()
        out.header.frame_id = self._frame
        out.twist           = msg
        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('twist_stamper')
    TwistStamper()
    rospy.spin()
