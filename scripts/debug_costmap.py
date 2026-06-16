#!/usr/bin/env python3
"""
debug_costmap.py — Rolling perception costmap for visualization only.

Subscribes to /scan_obstacles (LaserScan in base frame) and publishes a
nav_msgs/OccupancyGrid showing detected obstacles with inflation, centered
on the robot.  NOT connected to move_base — purely for RViz debugging.

Cost encoding matches costmap_2d's inflation layer:
  100 = lethal obstacle cell
  1–99 = inflated cost (exponential decay from obstacle edge)
  0   = free
"""
import math
import numpy as np
import rospy
import tf.transformations

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


class DebugCostmap:

    def __init__(self):
        self._res    = 0.10   # m/cell — coarser than nav costmap, fine for debug display
        self._w_m    = 20.0   # map width  [m]
        self._h_m    = 20.0   # map height [m]
        self._infl_r = rospy.get_param('~inflation_radius',    0.8)
        self._cscale = rospy.get_param('~cost_scaling_factor', 5.0)

        self._w = int(round(self._w_m / self._res))
        self._h = int(round(self._h_m / self._res))
        ic = int(math.ceil(self._infl_r / self._res))

        # Precomputed 2D cost kernel: kernel[ky, kx] = cost for that offset
        ks = 2 * ic + 1
        kernel = np.zeros((ks, ks), dtype=np.int32)
        for dy in range(-ic, ic + 1):
            for dx in range(-ic, ic + 1):
                dist = math.hypot(dx * self._res, dy * self._res)
                if dist > self._infl_r:
                    continue
                if dist == 0.0:
                    kernel[dy + ic, dx + ic] = 100
                else:
                    cost = int(math.exp(-self._cscale * (dist - self._res)) * 99)
                    kernel[dy + ic, dx + ic] = max(1, min(99, cost))
        self._kernel = kernel
        self._ic = ic

        self._rx = self._ry = self._ryaw = 0.0
        self._scan = None

        self._pub = rospy.Publisher(
            '/haptiquad_nav/debug_costmap', OccupancyGrid, queue_size=1, latch=False)
        rospy.Subscriber('/scan_obstacles', LaserScan,
                         self._scan_cb, queue_size=1)
        rospy.Subscriber('/slam/pose', PoseWithCovarianceStamped,
                         self._pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._publish)   # 5 Hz

        rospy.loginfo('[DebugCostmap] %dx%d cells @ %.2f m/cell, infl=%.2f m',
                      self._w, self._h, self._res, self._infl_r)

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._rx, self._ry = p.position.x, p.position.y
        q = p.orientation
        _, _, self._ryaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    def _scan_cb(self, msg):
        self._scan = msg

    def _publish(self, _event):
        scan = self._scan
        if scan is None:
            return

        # Grid lower-left corner in map frame
        ox = self._rx - self._w_m / 2.0
        oy = self._ry - self._h_m / 2.0

        cos_y = math.cos(self._ryaw)
        sin_y = math.sin(self._ryaw)
        grid = np.zeros((self._h, self._w), dtype=np.int32)
        ic, w, h = self._ic, self._w, self._h
        kernel = self._kernel

        seen = set()
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            a = scan.angle_min + i * scan.angle_increment
            # Scan point in base frame → map frame
            pbx = r * math.cos(a)
            pby = r * math.sin(a)
            pmx = self._rx + pbx * cos_y - pby * sin_y
            pmy = self._ry + pbx * sin_y + pby * cos_y
            # Grid cell
            gx = int((pmx - ox) / self._res)
            gy = int((pmy - oy) / self._res)
            if not (0 <= gx < w and 0 <= gy < h):
                continue
            if (gx, gy) in seen:
                continue
            seen.add((gx, gy))

            # Apply precomputed kernel via numpy slice (fast path)
            y0 = max(0, gy - ic);  y1 = min(h, gy + ic + 1)
            x0 = max(0, gx - ic);  x1 = min(w, gx + ic + 1)
            ky0 = y0 - (gy - ic);  ky1 = ky0 + (y1 - y0)
            kx0 = x0 - (gx - ic);  kx1 = kx0 + (x1 - x0)
            np.maximum(grid[y0:y1, x0:x1], kernel[ky0:ky1, kx0:kx1],
                       out=grid[y0:y1, x0:x1])

        out = OccupancyGrid()
        out.header.stamp        = rospy.Time.now()
        out.header.frame_id     = 'map'
        out.info.resolution     = self._res
        out.info.width          = self._w
        out.info.height         = self._h
        out.info.origin.position.x  = ox
        out.info.origin.position.y  = oy
        out.info.origin.position.z  = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.flatten().tolist()
        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('debug_costmap')
    DebugCostmap()
    rospy.spin()
