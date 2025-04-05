#!/usr/bin/env python3.12
import rclpy
from rclpy.node import Node
from bluedot import BlueDot
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class BlueDotControl(Node):
    def __init__(self):
        super().__init__('blue_dot_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bd = BlueDot()
        self.bd.when_pressed = self.on_press
        self.bd.when_moved = self.on_move
        self.bd.when_released = self.on_release

    def on_press(self):
        self.get_logger().info("Blue Dot pressed!")

    def on_move(self, pos):
        twist = Twist()
        if pos.top:
            twist.linear.x = 1.0  # Forward (W)
            twist.angular.z = 0.0
            self.get_logger().info("W")
        elif pos.bottom:
            twist.linear.x = -1.0  # Backward (S)
            twist.angular.z = 0.0
            self.get_logger().info("S")
        elif pos.left:
            twist.linear.x = 0.0  # Left (A)
            twist.angular.z = 1.0
            self.get_logger().info("A")
        elif pos.right:
            twist.linear.x = 0.0  # Right (D)
            twist.angular.z = -1.0
            self.get_logger().info("D")
        self.publisher.publish(twist)

    def on_release(self):
        twist = Twist()  # Stop (F)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Blue Dot released!")

def main(args=None):
    rclpy.init(args=args)
    blue_dot_control = BlueDotControl()
    rclpy.spin(blue_dot_control)
    blue_dot_control.destroy_node()
    rclpy.shutdown()

def timer_callback(self):
    msg = String()
    msg.data = 'Blue dot control is running!'
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)

if __name__ == '__main__':
    main()