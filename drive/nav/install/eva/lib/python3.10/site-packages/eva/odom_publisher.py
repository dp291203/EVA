# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import math
# import tf_transformations
# from rclpy.qos import qos_profile_sensor_data
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# class OdomPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_publisher')
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.publisher_ = self.create_publisher(Odometry, 'odom', qos_profile)
#         self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#         self.get_logger().info("Serial port opened: /dev/ttyACM0")

#         self.timer = self.create_timer(0.05, self.timer_callback)  

#     def timer_callback(self):
#         try:
#             line = self.serial_port.readline().decode().strip()
#             if not line:
#                 return

#             x_str, y_str, heading_str = line.split(',')
#             x = float(x_str)
#             y = float(y_str)
#             heading = float(heading_str)  # in radians

#             odom = Odometry()
#             odom.header.stamp = self.get_clock().now().to_msg()
#             odom.header.frame_id = "odom"
#             odom.child_frame_id = "base_link"
#             odom.pose.pose.position.x = x
#             odom.pose.pose.position.y = y
#             odom.pose.pose.position.z = 0.0

#             # Use tf_transformations to convert Euler angles to quaternion
#             quat = tf_transformations.quaternion_from_euler(0, 0, heading)
#             odom.pose.pose.orientation = Quaternion(
#                 x=quat[0],
#                 y=quat[1],
#                 z=quat[2],
#                 w=quat[3]
#             )

#             self.publisher_.publish(odom)

#         except Exception as e:
#             self.get_logger().warn(f"Error parsing serial line: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(Odometry, 'odom', qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial port opened: /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode().strip()
            if not line:
                return

            x_str, y_str, heading_str = line.split(',')
            x = float(x_str)
            y = float(y_str)
            heading = float(heading_str)  # in radians

            current_time = self.get_clock().now().to_msg()

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            # Euler to Quaternion conversion
            quat = tf_transformations.quaternion_from_euler(0, 0, heading)
            odom.pose.pose.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            )

            self.publisher_.publish(odom)

            # Also publish the transform
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"Error parsing serial line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

