#!/usr/bin/env python3
"""
Combines /state_estimator/pose_in_odom (PoseWithCovarianceStamped)
and /state_estimator/twist (TwistWithCovarianceStamped) into a
nav_msgs/Odometry message on /odom for move_base.
"""
import rospy
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


class OdomBridge:

    def __init__(self):
        self._pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        pose_sub  = message_filters.Subscriber(
            '/state_estimator/pose_in_odom', PoseWithCovarianceStamped)
        twist_sub = message_filters.Subscriber(
            '/state_estimator/twist', TwistWithCovarianceStamped)

        # Allow up to 0.05s slop between pose and twist stamps
        sync = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, twist_sub], queue_size=5, slop=0.05)
        sync.registerCallback(self._cb)

    def _cb(self, pose_msg, twist_msg):
        odom = Odometry()
        odom.header.stamp    = pose_msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base'
        odom.pose            = pose_msg.pose
        odom.twist           = twist_msg.twist
        self._pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('odom_bridge')
    OdomBridge()
    rospy.spin()
