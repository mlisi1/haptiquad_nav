#!/usr/bin/env python3
"""
debug_costmap.py — Rolling 3D perception costmap for visualization only.

Subscribes directly to the raw lidar PointCloud2, transforms to base frame
via tf2, applies height/range filters (360° — not just forward sector), then
projects to map frame and publishes an inflated 2D OccupancyGrid.

Confirmed solid obstacles are excluded: they live permanently in the global
costmap and should not create an inflation zone in this debug layer.

NOT connected to move_base — purely for RViz inspection.
"""
import math
import numpy as np
import rospy
import tf.transformations
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped


class DebugCostmap:

    def __init__(self):
        self._res    = 0.10   # m/cell
        self._w_m    = 20.0
        self._h_m    = 20.0
        self._infl_r      = rospy.get_param('~inflation_radius',    0.80)
        self._cscale      = rospy.get_param('~cost_scaling_factor', 3.0)
        self._robot_radius = rospy.get_param('~robot_radius',        0.55)

        # Cloud filter params — keep in sync with contact_probe.yaml
        self._h_min = rospy.get_param('~cloud_height_min', -0.10)
        self._h_max = rospy.get_param('~cloud_height_max',  1.20)
        self._r_min = rospy.get_param('~cloud_range_min',   0.55)
        self._r_max = rospy.get_param('~cloud_range_max',   8.0)

        solid_excl = rospy.get_param('~solid_excl_radius', 0.7)
        self._solid_excl_r2 = solid_excl ** 2

        self._w = int(round(self._w_m / self._res))
        self._h = int(round(self._h_m / self._res))
        ic = int(math.ceil(self._infl_r / self._res))

        # Precomputed inflation kernel
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
                    # Mirrors costmap_2d inflation: cost=99 inside inscribed radius,
                    # exponential falloff from inscribed to inflation boundary.
                    cost = int(math.exp(-self._cscale * (dist - self._robot_radius)) * 99)
                    kernel[dy + ic, dx + ic] = max(1, min(99, cost))
        self._kernel = kernel
        self._ic = ic

        # TF for sensor → base transform
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)

        self._rx = self._ry = self._ryaw = 0.0
        self._cloud    = None
        self._solid_np = None   # (N, 2) float32, map frame

        self._pub = rospy.Publisher(
            '/haptiquad_nav/debug_costmap', OccupancyGrid, queue_size=1, latch=False)
        rospy.Subscriber('point_cloud_in', PointCloud2,
                         self._cloud_cb, queue_size=1)
        rospy.Subscriber('/slam/pose', PoseWithCovarianceStamped,
                         self._pose_cb, queue_size=1)
        rospy.Subscriber('/haptiquad_nav/solid_obstacles', PointCloud2,
                         self._solids_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self._publish)   # 5 Hz

        rospy.loginfo('[DebugCostmap] %dx%d cells @ %.2f m/cell, infl=%.2f m',
                      self._w, self._h, self._res, self._infl_r)

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._rx, self._ry = p.position.x, p.position.y
        q = p.orientation
        _, _, self._ryaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    def _cloud_cb(self, msg):
        self._cloud = msg

    def _solids_cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            self._solid_np = None
            return
        step = msg.point_step
        fmap = {f.name: f.offset for f in msg.fields}
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        ox, oy = fmap['x'], fmap['y']
        sx = raw[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        sy = raw[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        self._solid_np = np.stack([sx, sy], axis=1)

    def _publish(self, _event):
        cloud = self._cloud
        if cloud is None:
            return

        # Get latest sensor → base transform
        try:
            tf_msg = self._tf_buf.lookup_transform(
                'base', cloud.header.frame_id, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, '[DebugCostmap] TF lookup failed: %s', e)
            return

        # Fast numpy XYZ extraction from PointCloud2
        n = cloud.width * cloud.height
        if n == 0:
            return
        step = cloud.point_step
        fmap = {f.name: f.offset for f in cloud.fields}
        if not all(k in fmap for k in ('x', 'y', 'z')):
            return
        buf = np.frombuffer(cloud.data, dtype=np.uint8)
        if buf.size != n * step:
            return
        raw = buf.reshape(n, step)
        ox, oy, oz = fmap['x'], fmap['y'], fmap['z']
        x = raw[:, ox:ox+4].copy().view(np.float32).reshape(-1)
        y = raw[:, oy:oy+4].copy().view(np.float32).reshape(-1)
        z = raw[:, oz:oz+4].copy().view(np.float32).reshape(-1)

        # Drop NaN/inf (organised clouds have many)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        if x.size == 0:
            return

        # Rigid-body transform to base frame: p_base = R * p_sensor + T
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        T = np.array([t.x, t.y, t.z])
        pts_b = np.stack([x, y, z], axis=1) @ R.T + T
        xb, yb, zb = pts_b[:, 0], pts_b[:, 1], pts_b[:, 2]

        # Height filter (base frame) — 360°, not limited to forward sector
        h_ok = (zb >= self._h_min) & (zb <= self._h_max)
        xb, yb = xb[h_ok], yb[h_ok]
        if xb.size == 0:
            return

        # Horizontal range filter
        r2 = xb**2 + yb**2
        r_ok = (r2 >= self._r_min**2) & (r2 <= self._r_max**2)
        xb, yb = xb[r_ok], yb[r_ok]
        if xb.size == 0:
            return

        # Project base frame → map frame using robot pose
        cos_y = math.cos(self._ryaw)
        sin_y = math.sin(self._ryaw)
        pmx = self._rx + xb * cos_y - yb * sin_y
        pmy = self._ry + xb * sin_y + yb * cos_y

        # Exclude returns near confirmed solid obstacles (vectorised)
        solid_np = self._solid_np
        if solid_np is not None and solid_np.shape[0] > 0:
            dx = pmx[:, None] - solid_np[None, :, 0]   # (N, M)
            dy = pmy[:, None] - solid_np[None, :, 1]
            near = np.any(dx**2 + dy**2 < self._solid_excl_r2, axis=1)
            pmx = pmx[~near]
            pmy = pmy[~near]
        if pmx.size == 0:
            return

        # Grid cell indices
        grid_ox = self._rx - self._w_m / 2.0
        grid_oy = self._ry - self._h_m / 2.0
        w, h = self._w, self._h
        gx = ((pmx - grid_ox) / self._res).astype(np.int32)
        gy = ((pmy - grid_oy) / self._res).astype(np.int32)
        in_bounds = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        gx, gy = gx[in_bounds], gy[in_bounds]

        # Apply inflation kernel for each unique grid cell
        ic = self._ic
        kernel = self._kernel
        grid = np.zeros((h, w), dtype=np.int32)
        seen = set()
        for gxi, gyi in zip(gx.tolist(), gy.tolist()):
            if (gxi, gyi) in seen:
                continue
            seen.add((gxi, gyi))
            y0 = max(0, gyi - ic);  y1 = min(h, gyi + ic + 1)
            x0 = max(0, gxi - ic);  x1 = min(w, gxi + ic + 1)
            ky0 = y0 - (gyi - ic);  ky1 = ky0 + (y1 - y0)
            kx0 = x0 - (gxi - ic);  kx1 = kx0 + (x1 - x0)
            np.maximum(grid[y0:y1, x0:x1], kernel[ky0:ky1, kx0:kx1],
                       out=grid[y0:y1, x0:x1])

        out = OccupancyGrid()
        out.header.stamp             = rospy.Time.now()
        out.header.frame_id          = 'map'
        out.info.resolution          = self._res
        out.info.width               = w
        out.info.height              = h
        out.info.origin.position.x   = grid_ox
        out.info.origin.position.y   = grid_oy
        out.info.origin.position.z   = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.flatten().tolist()
        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('debug_costmap')
    DebugCostmap()
    rospy.spin()
