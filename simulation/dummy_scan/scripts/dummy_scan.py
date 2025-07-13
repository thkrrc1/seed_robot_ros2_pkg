#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

import numpy as np
import math
import cv2
import threading
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, TransformStamped
import time

def getYaw(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)
    return yaw


def pose_to_mat(pose):
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.matrix(tf_transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat


def transform_to_mat(transform):
    quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    pos = np.matrix([transform.translation.x, transform.translation.y, transform.translation.z]).T
    mat = np.matrix(tf_transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat


def mat_to_pose(mat):
    pose = Pose()
    pose.position.x = mat[0, 3]
    pose.position.y = mat[1, 3]
    pose.position.z = mat[2, 3]
    quat = tf_transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


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


class DummyScan(Node):

    def __init__(self, map_topic, scan_topic, sensor_frame, scan_base_frame, scan_range_min, scan_range_max, angle_min, angle_max, resolution):
        super().__init__("dummy_scan")
        period = 0.05
        
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 1)
        
        self.timer = self.create_timer(period, self.timer_callback)
        self.map_base_frame = ""
        self.resolution = 0
        self.sensor_frame = sensor_frame
        self.scan_base_frame = scan_base_frame
        self.origin_x_pix = 0
        self.origin_y_pix = 0

        self.map_topic = map_topic
        self.scan = LaserScan()
        self.scan.header.frame_id = sensor_frame
        self.scan.angle_min = angle_min
        self.scan.angle_max = angle_max
        self.scan.angle_increment = resolution
        self.scan.time_increment = 0.0
        self.scan.scan_time = 0.0
        self.scan.range_min = float(scan_range_min)
        self.scan.range_max = float(scan_range_max)
        self.scan.ranges = [0.0 for idx in np.arange(angle_min, angle_max, resolution)]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.origin_ofst = np.eye(4)
        self.org_map = OccupancyGrid()
        
        self.count = 0
    
    def init_pose_callback(self, msg):
        self.timer.cancel()
        self.get_logger().info('init_pose received : %d' % threading.get_ident())
        # odomからセンサまでの変換
        try:
            trans_odom_to_sensor = self.tf_buffer.lookup_transform(self.scan_base_frame, self.sensor_frame, rclpy.time.Time())
        except(Exception):
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % (self.scan_base_frame, self.sensor_frame))
            self.timer.reset()
            return
        try:
            trans_baselink_to_sensor = self.tf_buffer.lookup_transform("base_link", self.sensor_frame, rclpy.time.Time())
        except(Exception):
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % (self.scan_base_frame, self.sensor_frame))
            self.timer.reset()
            return
        
        mat_odom_to_sensor = transform_to_mat(trans_odom_to_sensor.transform)
        mat_baselink_to_sensor = transform_to_mat(trans_baselink_to_sensor.transform)
        mat_map_to_baselink = pose_to_mat(msg.pose.pose)
        self.origin_ofst = (mat_map_to_baselink * mat_baselink_to_sensor) * np.linalg.inv(mat_odom_to_sensor)
        
        self.timer.reset()
        
    def map_callback(self, msg):
        self.get_logger().info('map received : %d' % threading.get_ident())
        self.org_map = msg
    
        self.timer.cancel()
        self.map_base_frame = self.org_map.header.frame_id
        info = self.org_map.info
        self.resolution = info.resolution

        width = info.width  # cells
        height = info.height  # cells
        
        if(width == 0 or height == 0):
            self.map = np.array([])
            self.timer.reset()
            return
        
        origin_x = info.origin.position.x  # ロボット原点から見たマップ原点位置[m](反転画像座標的には、画像左上位置)
        origin_y = info.origin.position.y
        origin_z = info.origin.position.z
        origin_yaw = getYaw(info.origin.orientation)
        map_tmp = np.array(self.org_map.data).reshape(height, width).astype(np.uint8)
        map_tmp = np.where(map_tmp == 255, 0, map_tmp)
        
        p1 = np.array([-width/2,-height/2])
        p2 = np.array([-width/2, height/2])
        
        mat = np.array([[np.cos(origin_yaw), - np.sin(origin_yaw)],[np.sin(origin_yaw), np.cos(origin_yaw)]])
        new_p1 = np.fabs(mat.dot(p1))
        new_p2 = np.fabs(mat.dot(p2))
        
        max_x = max(new_p1[0],new_p2[0])
        max_y = max(new_p1[1],new_p2[1])
        new_width = int(max_x*2)
        new_height = int(max_y*2)        

        # odomから見た、画像原点位置
        self.origin_x_pix = int(origin_x / self.resolution)
        self.origin_y_pix = int(origin_y / self.resolution)
        self.origin_yaw_rad = origin_yaw
        
        self.origin_x_pix +=  int((-new_width/2) - ((-width/2) * np.cos(origin_yaw) - (-height/2) * np.sin(origin_yaw)))
        self.origin_y_pix +=  int((-new_height/2) - ((-width/2) * np.sin(origin_yaw) + (-height/2) * np.cos(origin_yaw)))

        # マップの回転
        angle = math.degrees(origin_yaw)
        scale = 1.0
        center = (int(width / 2), int(height / 2))
        trans = cv2.getRotationMatrix2D(center, -angle, scale) #反時計周りが正なので、正負逆転
        trans[0][2] += new_width/2 - width / 2
        trans[1][2] += new_height/2 - height / 2
        self.map = cv2.warpAffine(map_tmp, trans, (new_width, new_height))

        now = rclpy.time.Time();
        try:
            trans_odom_to_sensor = self.tf_buffer.lookup_transform(self.scan_base_frame, self.sensor_frame, now, timeout=rclpy.duration.Duration(seconds=1))
        except(Exception):
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % (self.scan_base_frame, self.sensor_frame))
            return
        
        try:
            trans_baselink_to_sensor = self.tf_buffer.lookup_transform("base_link", self.sensor_frame, now, timeout=rclpy.duration.Duration(seconds=10))
        except(Exception):
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % ("base_link", self.sensor_frame))
            self.timer.reset()
            return
                
        try:
            trans_map_to_baselink = self.tf_buffer.lookup_transform("map", "base_link", now, timeout=rclpy.duration.Duration(seconds=10))
        except(Exception):
            trans_map_to_baselink = TransformStamped()
            trans_map_to_baselink.transform.rotation.w = 1.0
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % ("map", "base_link"))
            self.timer.reset()
        
        mat_odom_to_sensor = transform_to_mat(trans_odom_to_sensor.transform)
        mat_baselink_to_sensor = transform_to_mat(trans_baselink_to_sensor.transform)
        mat_map_to_baselink = transform_to_mat(trans_map_to_baselink.transform)
        self.origin_ofst = (mat_map_to_baselink * mat_baselink_to_sensor) * np.linalg.inv(mat_odom_to_sensor)

        self.timer.reset()
        
        
    def timer_callback(self):
        if(self.count < 50):
            self.scan.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(self.scan)
            self.count = self.count+1
            return
        elif self.count == 50 or self.resolution == 0 or self.map.size == 0:
            latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, latching_qos)
            self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.init_pose_callback, rclpy.qos.qos_profile_system_default)
            self.count = self.count+1
            return
        
        try:
            trans_odom_to_sensor = self.tf_buffer.lookup_transform(self.scan_base_frame, self.sensor_frame, rclpy.time.Time())
        except(Exception):
            self.get_logger().error("tf transform could not be solved. from: %s to: %s" % (self.scan_base_frame, self.sensor_frame))
            self.scan.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(self.scan)
            return
        
        mat_odom_to_sensor = transform_to_mat(trans_odom_to_sensor.transform)
        pose_map_to_sensor = mat_to_pose(self.origin_ofst * mat_odom_to_sensor)
        
        sensor_yaw = getYaw(pose_map_to_sensor.orientation)
        sensor_pos = pose_map_to_sensor.position
        sensor_range = self.scan.range_max  # [m]
        ang_min = self.scan.angle_min + sensor_yaw
        ang_max = self.scan.angle_max + sensor_yaw
        ang_step = self.scan.angle_increment
        
        cur_x_pix = int(sensor_pos.x / self.resolution)
        cur_y_pix = int(sensor_pos.y / self.resolution)
        
        # odomから見た、センサ最大、最小距離のピクセル値
        angles = np.arange(ang_min, ang_max, ang_step)
        xmaxs = (cur_x_pix + (self.scan.range_max * np.cos(angles)) / self.resolution).astype(int)
        ymaxs = (cur_y_pix + (self.scan.range_max * np.sin(angles)) / self.resolution).astype(int)
        xmins = (cur_x_pix + (self.scan.range_min * np.cos(angles)) / self.resolution).astype(int)
        ymins = (cur_y_pix + (self.scan.range_min * np.sin(angles)) / self.resolution).astype(int)

        for i in range(angles.size):
            # 画像原点から見たlidarの走査直線上のピクセル座標を取得
            points = LineIterator((xmins[i], ymins[i]), (xmaxs[i], ymaxs[i]))
            points = points - (self.origin_x_pix, self.origin_y_pix)

            # 座標が画像の範囲を超えないようにする
            points = np.where(points < 0, 0, points)
            points[:, 0] = np.where(points[:, 0] >= self.map.shape[1], self.map.shape[1] - 1, points[:, 0])
            points[:, 1] = np.where(points[:, 1] >= self.map.shape[0], self.map.shape[0] - 1, points[:, 1])

            # lidarの値を入れる
            pixval = self.map[points[:, 1], points[:, 0]]  # 直線上のピクセル値
            idxs = np.where(pixval == 100)[0]  # whereは、タプルで返ってくるので、最初の要素を取り出す
            if(idxs.size == 0):
                self.scan.ranges[i] = (sensor_range + 1)
            else:
                p = points[idxs[0]]
                dist = math.hypot((p[0] + self.origin_x_pix) - cur_x_pix, (p[1] + self.origin_y_pix) - cur_y_pix) * self.resolution
                self.scan.ranges[i] = dist + np.random.multivariate_normal([0], np.diag([0.005**2]), 1)

        self.scan.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(self.scan)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DummyScan("/scan_map", "/scan", "wheels_base_laser_link", "odom", 0.01, 10, -2., 2., 0.05)
        # executor = MultiThreadedExecutor()
        # rclpy.spin(node,executor)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
