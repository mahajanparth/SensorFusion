#!/usr/bin/env python3
"""Collects the EKF variants, wheel odometry and GPS into one frame for RViz.

GPS /fix is converted to a local ENU frame anchored at the first fix ("map").
Each odometry source is mapped into that frame with an SE(2) transform from
eval/results/alignment.json (written by eval/evaluate.py), so the overlay matches
the evaluation plots. Without the file every source is drawn in its own frame.

For the EKF variants the x/y pose covariance is rotated into the same frame and
drawn as 3-sigma ellipses on /viz/covariance: the current one, plus a trail of
every `cov_every`-th estimate.
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# topic -> (key in alignment.json, label, RGB); colors match the evaluation plots.
SOURCES = {
    '/wheel_odom': ('wheel_odom', 'wheel odom', (0.54, 0.54, 0.52)),
    '/ekf_A/odom': ('A', 'EKF-A', (0.92, 0.41, 0.20)),
    '/ekf_B/odom': ('B', 'EKF-B', (0.11, 0.69, 0.48)),
    '/ekf_C/odom': ('C', 'EKF-C', (0.93, 0.63, 0.00)),
}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class VariantPathPublisher(Node):

    def __init__(self):
        super().__init__('variant_path_publisher')
        self.frame = self.declare_parameter('frame_id', 'map').value
        path = self.declare_parameter('alignment_file', '').value
        self.cov_every = self.declare_parameter('cov_every', 10).value
        self.align = {}
        if path and os.path.exists(path):
            with open(path) as f:
                self.align = json.load(f)
            self.get_logger().info(f'Using alignment from {path}')
        else:
            self.get_logger().warn('No alignment file; sources are drawn in their own frames')

        self.paths, self.path_pubs = {}, {}
        self.cov_trail = {t: [] for t in SOURCES if t != '/wheel_odom'}
        self.cov_current = {}
        self.cov_count = {t: 0 for t in self.cov_trail}
        self.cov_pub = self.create_publisher(MarkerArray, '/viz/covariance', 10)
        for topic, (key, _, _) in SOURCES.items():
            self.paths[topic] = Path()
            self.paths[topic].header.frame_id = self.frame
            self.path_pubs[topic] = self.create_publisher(Path, f'/viz/path_{key}', 10)
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.on_odom(t, m), 50)

        self.origin = None
        self.gps_marker = Marker()
        self.gps_marker.header.frame_id = self.frame
        self.gps_marker.ns, self.gps_marker.id = 'gps', 0
        self.gps_marker.type, self.gps_marker.action = Marker.SPHERE_LIST, Marker.ADD
        self.gps_marker.scale.x = self.gps_marker.scale.y = self.gps_marker.scale.z = 0.8
        self.gps_marker.pose.orientation.w = 1.0
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', 10)
        self.create_subscription(NavSatFix, '/fix', self.on_fix, 10)
        self.create_timer(0.5, self.publish_markers)

    def on_fix(self, msg):
        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
        lat0, lon0 = self.origin
        # Same local tangent plane as eval/evaluate.py.
        east = (msg.longitude - lon0) * math.cos(math.radians(lat0)) * 111320.0
        north = (msg.latitude - lat0) * 110574.0
        good = msg.position_covariance[0] < 2.0
        self.gps_marker.points.append(Point(x=east, y=north, z=0.0))
        self.gps_marker.colors.append(ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0 if good else 0.35))

    def on_odom(self, topic, msg):
        key = SOURCES[topic][0]
        a = self.align.get(key, dict(yaw=0.0, t=[0.0, 0.0]))
        c, s = math.cos(a['yaw']), math.sin(a['yaw'])
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation) + a['yaw']

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame
        pose.pose.position.x = c * p.x - s * p.y + a['t'][0]
        pose.pose.position.y = s * p.x + c * p.y + a['t'][1]
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        path = self.paths[topic]
        path.header.stamp = msg.header.stamp
        path.poses.append(pose)
        self.path_pubs[topic].publish(path)

        if topic in self.cov_trail:
            cov = msg.pose.covariance
            # Rotate the x/y block into the map frame: R * P * R^T.
            pxx, pxy, pyy = cov[0], cov[1], cov[7]
            rxx = c * c * pxx - 2 * c * s * pxy + s * s * pyy
            rxy = c * s * (pxx - pyy) + (c * c - s * s) * pxy
            ryy = s * s * pxx + 2 * c * s * pxy + c * c * pyy
            ellipse = (pose.pose.position.x, pose.pose.position.y, rxx, rxy, ryy)
            self.cov_current[topic] = ellipse
            if self.cov_count[topic] % self.cov_every == 0:
                self.cov_trail[topic].append(ellipse)
            self.cov_count[topic] += 1

    def publish_markers(self):
        arr = MarkerArray()
        if self.gps_marker.points:
            arr.markers.append(self.gps_marker)
        placed = []
        for i, (topic, (_, label, rgb)) in enumerate(SOURCES.items()):
            poses = self.paths[topic].poses
            if not poses:
                continue
            x, y = poses[-1].pose.position.x, poses[-1].pose.position.y
            # Stack labels whose path heads are close together so they stay readable.
            y += 5.0 * sum(1 for px, py in placed if math.hypot(px - x, py - y) < 10.0)
            placed.append((poses[-1].pose.position.x, poses[-1].pose.position.y))
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id = 'labels', i
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x = x + 4.0
            m.pose.position.y = y
            m.pose.position.z = 4.0
            m.pose.orientation.w = 1.0
            m.scale.z = 4.0
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            m.text = label
            arr.markers.append(m)
        self.marker_pub.publish(arr)
        self.publish_covariance()

    def publish_covariance(self):
        arr = MarkerArray()
        for i, (topic, trail) in enumerate(self.cov_trail.items()):
            rgb = SOURCES[topic][2]
            for ns, ellipses, width, alpha in (('trail', trail, 0.15, 0.45),
                                               ('current', [self.cov_current[topic]] if topic in self.cov_current else [], 0.35, 1.0)):
                for j, e in enumerate(ellipses):
                    m = Marker()
                    m.header.frame_id = self.frame
                    m.ns, m.id = f'{ns}_{SOURCES[topic][0]}', j
                    m.type, m.action = Marker.LINE_STRIP, Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = width
                    m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
                    m.points = ellipse_points(*e)
                    arr.markers.append(m)
        if arr.markers:
            self.cov_pub.publish(arr)


def ellipse_points(x, y, pxx, pxy, pyy, n_sigma=3.0, segments=36):
    """Outline of the n-sigma ellipse of a 2x2 covariance centred at (x, y)."""
    tr, det = pxx + pyy, pxx * pyy - pxy * pxy
    disc = math.sqrt(max(tr * tr / 4 - det, 0.0))
    l1, l2 = tr / 2 + disc, max(tr / 2 - disc, 0.0)
    ang = 0.5 * math.atan2(2 * pxy, pxx - pyy)
    a, b = n_sigma * math.sqrt(max(l1, 0.0)), n_sigma * math.sqrt(l2)
    ca, sa = math.cos(ang), math.sin(ang)
    pts = []
    for k in range(segments + 1):
        t = 2 * math.pi * k / segments
        ex, ey = a * math.cos(t), b * math.sin(t)
        pts.append(Point(x=x + ca * ex - sa * ey, y=y + sa * ex + ca * ey, z=0.1))
    return pts


def main():
    rclpy.init()
    rclpy.spin(VariantPathPublisher())


if __name__ == '__main__':
    main()
