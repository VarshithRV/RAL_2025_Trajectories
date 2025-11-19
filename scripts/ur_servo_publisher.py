#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

class URServoPublisher(Node):
    def __init__(self):
        super().__init__("ur_servo_publisher")
        self.ur_servo_publisher = self.create_publisher(TwistStamped,"/servo_node/delta_twist_cmds",qos_profile=10)
        timer_period = 1/50
        self.timer = self.create_timer(timer_period,self.timer_callback)
    
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = -0.1
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.ur_servo_publisher.publish(msg)

def main(args=None):
    rclpy.init()
    ur_servo_publisher_node = URServoPublisher()
    rclpy.spin(ur_servo_publisher_node)
    ur_servo_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()