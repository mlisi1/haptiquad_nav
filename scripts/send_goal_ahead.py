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
import os
import sys

import rosgraph
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String


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

    print('connecting to the ROS master at %s ...' % os.environ.get('ROS_MASTER_URI', '(ROS_MASTER_URI unset)'))
    rospy.init_node('send_goal_ahead', anonymous=True)
    frame = args.frame or rospy.get_param('/contact_probe_manager/nav_frame', 'odom')

    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)   # keep a reference: it owns the /tf subscribers
    print('waiting for the %s -> %s transform ...' % (frame, args.base))
    try:
        tr = buf.lookup_transform(frame, args.base, rospy.Time(0), rospy.Duration(5.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        print('ERROR: no %s -> %s transform after 5 s: %s' % (frame, args.base, e))
        print('       is /tf reaching this machine? try: rosrun tf tf_echo %s %s' % (frame, args.base))
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

    # Confirmations: contact_probe_manager logs every goal it accepts on a
    # latched topic (the message already there is from an earlier goal), and
    # move_base turns the goal into an action goal on /move_base/goal.
    want = 'goal (%.2f, %.2f)' % (goal.pose.position.x, goal.pose.position.y)
    cpm_ok, mb_ok = [], []
    rospy.Subscriber('/haptiquad_nav/mission_event', String,
                     lambda m: cpm_ok.append(1) if m.data.endswith(want) else None)

    def _mb_cb(m):
        p = m.goal.target_pose.pose.position
        if math.hypot(p.x - goal.pose.position.x, p.y - goal.pose.position.y) < 0.01:
            mb_ok.append(1)
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, _mb_cb)

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    # A message published before a subscriber has connected is lost for that
    # subscriber. Waiting for "2 connections" was not enough: the bag recorder
    # (and rviz) also subscribe, so the goal went out before move_base had
    # connected and only contact_probe_manager got it (hardware run 7). Wait
    # for every subscriber the master knows, then check that move_base
    # actually received it, and send again if not.
    master = rosgraph.Master(rospy.get_name())

    def subscribers():
        try:
            for topic, nodes in master.getSystemState()[1]:
                if topic == '/move_base_simple/goal':
                    return [n for n in nodes if n != rospy.get_name()]
        except Exception:
            pass
        return []

    print('waiting for the subscribers of /move_base_simple/goal to connect ...')
    deadline = rospy.Time.now() + rospy.Duration(5.0)
    while not rospy.is_shutdown():
        subs = subscribers()
        if subs and pub.get_num_connections() >= len(subs):
            break
        if rospy.Time.now() > deadline:
            print('WARNING: %d of %d subscribers connected after 5 s (%s); sending anyway'
                  % (pub.get_num_connections(), len(subs), ', '.join(subs)))
            break
        rospy.sleep(0.05)
    subs = subscribers()
    if not any(n.endswith('move_base') for n in subs):
        print('ERROR: move_base does not subscribe to /move_base_simple/goal -- is the navigation core running?')
        sys.exit(1)
    rospy.sleep(0.2)   # margin for a connection that is still being set up

    for attempt in range(1, 4):
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        print('sent to /move_base_simple/goal (attempt %d, %d of %d subscribers connected)'
              % (attempt, pub.get_num_connections(), len(subs)))
        deadline = rospy.Time.now() + rospy.Duration(1.5)
        while not (mb_ok and cpm_ok) and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)
        if mb_ok:
            break
        print('move_base did not receive it, sending again')
    print('move_base:             %s' % ('goal received' if mb_ok else 'NOT received'))
    print('contact_probe_manager: %s' % ('mission started' if cpm_ok else
                                         'NOT confirmed (calibrating or HALTED? check its log)'))
    if not (mb_ok and cpm_ok):
        sys.exit(2)
    print('OK: the robot is on its way')

if __name__ == '__main__':
    main()
