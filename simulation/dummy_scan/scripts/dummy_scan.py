#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

import rclpy
import tf2_ros
import tf_transformations
import cv2

import nav_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg


# 直線のピクセル値を取得
def LineIterator(p1, p2):
    p1x = p1[0]
    p1y = p1[1]
    p2x = p2[0]
    p2y = p2[1]

    dx = p2x - p1x
    dy = p2y - p1y

    xnum = np.abs(dx) + 1
    ynum = np.abs(dy) + 1

    if(xnum < ynum):
        slope = dx.astype(np.float32) / dy.astype(np.float32)
        steep = True
    else:
        slope = dy.astype(np.float32) / dx.astype(np.float32)
        steep = False

    # x,yのうち、成分が多い方を基準にする
    line_pixnum = np.maximum(xnum, ynum)
    line = np.empty(shape=(line_pixnum, 2), dtype=np.int32)

    # ブレゼンハムのアルゴリズム
    if steep:
        # yをベースに、xを求める
        if p1y > p2y:
            line[:, 1] = np.arange(p1y, p2y - 1, -1)
        else:
            line[:, 1] = np.arange(p1y, p2y + 1, 1)
        line[:, 0] = (slope * (line[:, 1] - p1y)).astype(np.int32) + p1x
    else:
        # xをベースに、yを求める
        if p1x > p2x:
            line[:, 0] = np.arange(p1x, p2x - 1, -1)
        else:
            line[:, 0] = np.arange(p1x, p2x + 1, 1)
        line[:, 1] = (slope * (line[:, 0] - p1x)).astype(np.int32) + p1y

    return line.astype(int)


class Pose:

    def __init__(self, x: float=0.0, y: float=0.0, th: float=0.0):
        self.x = x
        self.y = y
        self.th = th
        
    @classmethod
    def from_ros_pose(cls, pose: geometry_msgs.msg.Pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (_, _, th) = tf_transformations.euler_from_quaternion(quaternion)
        return cls(pose.position.x, pose.position.y, th)

    @classmethod
    def from_transform(cls, transform: geometry_msgs.msg.Transform):
        quaternion = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        (_, _, th) = tf_transformations.euler_from_quaternion(quaternion)
        return cls(transform.translation.x, transform.translation.y, th)
    
    def to_ros_pose(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = float(self.x)
        pose.position.y = float(self.y)
        pose.position.z = float(0.0)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def __add__(self, rhs):
        return Pose(self.x + rhs.x, self.y + rhs.y, self.th + rhs.th)
    
    def __mul__(self, rhs):
        c = np.cos(self.th)
        s = np.sin(self.th)
        return Pose(self.x + c * rhs.x - s * rhs.y,
                    self.y + s * rhs.x + c * rhs.y,
                    self.th + rhs.th)
    
    def inv(self):
        return Pose(-self.x, -self.y, -self.th)
    
    def __str__(self):
        return f"[{self.x}, {self.y}, {self.th}]"


class Scan:

    def __init__(self, sensor_frame, ang_min, ang_max, ang_step, range_min, range_max):
        self._scan = sensor_msgs.msg.LaserScan()
        self._scan.header.frame_id = sensor_frame
        self._scan.angle_min = ang_min
        self._scan.angle_max = ang_max
        self._scan.angle_increment = ang_step
        self._scan.time_increment = 0.0
        self._scan.scan_time = 0.0
        self._scan.range_min = range_min
        self._scan.range_max = range_max
        self._scan.ranges = [0.0 for idx in np.arange(self._scan.angle_min, self._scan.angle_max, self._scan.angle_increment)]
        self.angles_template = np.arange(ang_min, ang_max, ang_step)
    
    @property
    def range_max(self):
        return self._scan.range_max
    
    @property
    def range_min(self):
        return self._scan.range_min
    
    @property
    def message(self):
        return self._scan
    
    def set_range(self, idx, value):
        self._scan.ranges[idx] = value + np.random.multivariate_normal([0], np.diag([0.005 ** 2]), 1)
        
    def set_range_invalid(self, idx):
        self._scan.ranges[idx] = (self._scan.range_max + 1)


class ScanMap():

    def __init__(self):
        self._initialized = False
    
    def set_map(self, map):
        info = map.info
        width = info.width
        height = info.height
        self._map_to_img = Pose.from_ros_pose(info.origin)
        self._resolution = info.resolution
        self._map = np.array(map.data).reshape(height, width).astype(np.int8)
        cv2.imwrite("map.jpeg",self._map)
        
        self._initialized = True
    
    def update_scan(self, scan: Scan, map_to_sensor):
        if not self._initialized:
            return

        img_to_sensor = self._map_to_img.inv() * map_to_sensor
        pix_x = int(img_to_sensor.x / self._resolution)
        pix_y = int(img_to_sensor.y / self._resolution)
        
        angles = scan.angles_template + img_to_sensor.th
        xmaxs = (pix_x + (scan.range_max * np.cos(angles)) / self._resolution).astype(int)
        ymaxs = (pix_y + (scan.range_max * np.sin(angles)) / self._resolution).astype(int)
        xmins = (pix_x + (scan.range_min * np.cos(angles)) / self._resolution).astype(int)
        ymins = (pix_y + (scan.range_min * np.sin(angles)) / self._resolution).astype(int)

        map_rows, map_cols = self._map.shape
        
        for i in range(angles.size):
            points = LineIterator((xmins[i], ymins[i]), (xmaxs[i], ymaxs[i]))
            points[:, 1] = np.clip(points[:, 1], 0, map_rows - 1)
            points[:, 0] = np.clip(points[:, 0], 0, map_cols - 1)
            pixval = self._map[points[:, 1], points[:, 0]]
            idxs = np.where(pixval == 100)[0]
            if(idxs.size == 0):
                scan.set_range_invalid(i)
            else:
                p = points[idxs[0]]
                dist = np.hypot((p[0] - pix_x), (p[1] - pix_y)) * self._resolution
                scan.set_range(i, dist)


class ScanNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("scan_node")
        
        prefix = self.get_namespace().lstrip('/')
        self.declare_parameter('odom_frame', "odom")
        self.declare_parameter('sensor_frame', "wheels_base_laser_link")
        self.declare_parameter('robot_base_frame', "base_link")
        self.declare_parameter('map_frame', "map")
        self.declare_parameter('update_period', 0.05)

        self.declare_parameter('scan/range_min', 0.01)
        self.declare_parameter('scan/range_max', 10.0)
        self.declare_parameter('scan/angle_min', -math.pi / 2)
        self.declare_parameter('scan/angle_max', math.pi / 2)
        self.declare_parameter('scan/angle_step', 0.05)

        init_pose_topic = "initialpose"
        map_topic = "/map"
        scan_topic = "/scan"
        
        self._robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._sensor_frame = self.get_parameter('sensor_frame').get_parameter_value().string_value
        period = self.get_parameter('update_period').value

        range_min = self.get_parameter('scan/range_min').value
        range_max = self.get_parameter('scan/range_max').value
        ang_min = self.get_parameter('scan/angle_min').value
        ang_max = self.get_parameter('scan/angle_max').value
        ang_step = self.get_parameter('scan/angle_step').value
        self._scan_pub = self.create_publisher(sensor_msgs.msg.LaserScan, scan_topic, rclpy.qos.qos_profile_sensor_data)
        
        self._scan = Scan(self._sensor_frame, ang_min, ang_max, ang_step, range_min, range_max)
        self._tf_buffer = tf2_ros.buffer.Buffer()
        self._tf_listener = tf2_ros.transform_listener.TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(period, self.timer_callback)
        self._map_to_odom = Pose(0.0, 0.0, 0.0)
        
        latching_qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(nav_msgs.msg.OccupancyGrid, map_topic, self.map_callback, latching_qos)
        self.init_pose_sub = self.create_subscription(geometry_msgs.msg.PoseWithCovarianceStamped, init_pose_topic, self.init_pose_callback, rclpy.qos.qos_profile_system_default)

        self._scan_map = ScanMap()
        
    def map_callback(self, msg):
        self._scan_map.set_map(msg)
    
    def init_pose_callback(self, msg : geometry_msgs.msg.PoseWithCovarianceStamped):
        try:
            trans_base_to_odom = self._tf_buffer.lookup_transform(self._robot_base_frame, self._odom_frame, rclpy.time.Time())
        except(Exception):
            return
        
        map_to_base = Pose.from_ros_pose(msg.pose.pose)
        base_to_odom = Pose.from_transform(trans_base_to_odom.transform)
        self._map_to_odom = map_to_base * base_to_odom

    def timer_callback(self):
        try:
            trans_odom_to_sensor = self._tf_buffer.lookup_transform(self._odom_frame, self._sensor_frame, rclpy.time.Time())
        except(Exception):
            return
        
        odom_to_sensor = Pose.from_transform(trans_odom_to_sensor.transform)
        map_to_sensor = self._map_to_odom * odom_to_sensor
        self._scan_map.update_scan(self._scan, map_to_sensor)

        scan_msg = self._scan.message        
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        self._scan_pub.publish(scan_msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ScanNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
