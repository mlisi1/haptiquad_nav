#!/usr/bin/env python3
"""
Extracts the base_wrench entry from /estimated_forces (world frame)
and republishes it as geometry_msgs/WrenchStamped for the contact probe.
"""
import rospy
from haptiquad_msgs.msg import EstimatedForces
from geometry_msgs.msg import WrenchStamped


class WrenchBridge:

    def __init__(self):
        self._pub = rospy.Publisher('base_wrench', WrenchStamped, queue_size=1)
        rospy.Subscriber('/estimated_forces', EstimatedForces, self._cb, queue_size=1)

    def _cb(self, msg):
        try:
            idx = msg.names.index('base_wrench')
        except ValueError:
            return

        out = WrenchStamped()
        out.header = msg.header
        out.wrench = msg.forces[idx]
        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('wrench_bridge')
    WrenchBridge()
    rospy.spin()
