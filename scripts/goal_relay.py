#!/usr/bin/env python3
"""
goal_relay.py -- forwards rviz "2D Nav Goal" poses from /goal (where the
ANYmal rviz config publishes them) to /move_base_simple/goal.

Replaces topic_tools/relay, which advertises its output topic only when the
first message arrives and publishes that message straight away, before
move_base and contact_probe_manager have connected: the first goal after
every launch was lost and had to be sent twice (hardware run 6). This node
advertises at startup, so subscribers are connected before any goal comes in.
"""
import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('goal_relay')
    pub = rospy.Publisher('out', PoseStamped, queue_size=5)
    rospy.Subscriber('in', PoseStamped, pub.publish, queue_size=5)
    rospy.spin()
