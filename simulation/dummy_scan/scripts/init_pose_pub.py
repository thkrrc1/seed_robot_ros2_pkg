import rclpy
from rclpy.node import Node

import tf2_ros
import tf_transformations
import rcl_interfaces.msg
import geometry_msgs.msg


class PublisherNode(Node):

    def __init__(self):
        super().__init__("init_pose_pub")

        self.declare_parameter('pose', [0.0, 0.0, 0.0])
        self.declare_parameter('map_frame', "map")
        
        self.publisher = self.create_publisher(geometry_msgs.msg.PoseWithCovarianceStamped, self.get_namespace()+'/initialpose', rclpy.qos.qos_profile_system_default)
        
        self._timer = self.create_timer(5.0, self.timer_callback)
        
    def timer_callback(self):
        pose = self.get_parameter('pose').get_parameter_value().double_array_value
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        msg = geometry_msgs.msg.PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_frame

        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, pose[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.pose.covariance = [0.0] * 36 
        msg.pose.covariance[0] = 0.0001  
        msg.pose.covariance[7] = 0.0001  
        msg.pose.covariance[35] = 0.0001 

        self.publisher.publish(msg)
        self._timer.cancel()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
