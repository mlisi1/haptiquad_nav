#!/usr/bin/env python3
"""
send_goal_ahead.py -- send a navigation goal relative to the robot's current
pose, more precisely than clicking in rviz.

  rosrun haptiquad_nav send_goal_ahead.py                 # 1.0 m straight ahead
  rosrun haptiquad_nav send_goal_ahead.py 2               # 2.0 m ahead
  rosrun haptiquad_nav send_goal_ahead.py 2.5 -1          # 2.5 m ahead, 1 m to the right
  rosrun haptiquad_nav send_goal_ahead.py 2 0 --yaw 90    # ...and face 90 deg left at the goal
  rosrun haptiquad_nav send_goal_ahead.py 2 --dry-run     # print the goal, don't send it

Ahead/left are in the robot's base frame at the moment the script runs
(left is +y, so a negative lateral offset is to the right); the goal heading
is the robot's current heading plus --yaw. The goal is published once on
/move_base_simple/goal, like the rviz 2D Nav Goal, so contact_probe_manager
treats it as a user goal (a mission). It is expressed in the stack's
nav_frame (read from /contact_probe_manager/nav_frame; odom on the robot,
map in simulation) unless --frame is given.
"""
import argparse
import math
import sys

import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped


def main():
    ap = argparse.ArgumentParser(description='Send a goal relative to the robot.')
    ap.add_argument('ahead', nargs='?', type=float, default=1.0,
                    help='metres ahead of the robot (default 1.0)')
    ap.add_argument('left', nargs='?', type=float, default=0.0,
                    help='metres to the left (negative = right, default 0)')
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='goal heading relative to the current heading, degrees (default 0)')
    ap.add_argument('--frame', default=None,
                    help='goal frame (default: /contact_probe_manager/nav_frame, else odom)')
    ap.add_argument('--base', default='base', help='robot base frame (default base)')
    ap.add_argument('--dry-run', action='store_true', help='print the goal without sending it')
    args = ap.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node('send_goal_ahead', anonymous=True)
    frame = args.frame or rospy.get_param('/contact_probe_manager/nav_frame', 'odom')

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    try:
        tr = buf.lookup_transform(frame, args.base, rospy.Time(0), rospy.Duration(3.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr('No %s -> %s transform: %s', frame, args.base, e)
        sys.exit(1)
    t, q = tr.transform.translation, tr.transform.rotation
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    c, s = math.cos(yaw), math.sin(yaw)

    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = t.x + c * args.ahead - s * args.left
    goal.pose.position.y = t.y + s * args.ahead + c * args.left
    gq = tft.quaternion_from_euler(0.0, 0.0, yaw + math.radians(args.yaw))
    (goal.pose.orientation.x, goal.pose.orientation.y,
     goal.pose.orientation.z, goal.pose.orientation.w) = gq

    print('robot  (%.3f, %.3f) heading %.1f deg in %s' % (t.x, t.y, math.degrees(yaw), frame))
    print('goal   (%.3f, %.3f) heading %.1f deg: %.2f m ahead, %.2f m left'
          % (goal.pose.position.x, goal.pose.position.y,
             math.degrees(yaw) + args.yaw, args.ahead, args.left))
    if args.dry_run:
        return

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    # A message published before the subscribers connect is lost: wait for
    # move_base and contact_probe_manager (both subscribe to this topic).
    deadline = rospy.Time.now() + rospy.Duration(3.0)
    while pub.get_num_connections() < 2 and rospy.Time.now() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if pub.get_num_connections() == 0:
        rospy.logerr('Nobody subscribes to /move_base_simple/goal -- is the navigation core running?')
        sys.exit(1)
    n = pub.get_num_connections()
    goal.header.stamp = rospy.Time.now()
    pub.publish(goal)
    rospy.sleep(0.3)   # let the message go out before the node exits
    print('sent to /move_base_simple/goal (%d subscribers)' % n)


if __name__ == '__main__':
    main()
