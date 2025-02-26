import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class PWMController(Node):
    def __init__(self):
        super().__init__('pwm_controller')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        self.max_pwm = 1000
        self.wheelbase = 0.3  # Distance between left and right wheels
        self.track_width = 0.3  # Distance between front and rear wheels
        self.max_speed = 1.0

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Skid-steer kinematics (simplified: front/rear same side move together)
        fl_speed = linear_x - angular_z * (self.wheelbase / 2)
        fr_speed = linear_x + angular_z * (self.wheelbase / 2)
        rl_speed = fl_speed
        rr_speed = fr_speed

        # Map to PWM
        pwm = [
            int(fl_speed / self.max_speed * self.max_pwm),
            int(fr_speed / self.max_speed * self.max_pwm),
            int(rl_speed / self.max_speed * self.max_pwm),
            int(rr_speed / self.max_speed * self.max_pwm)
        ]
        for i in range(4):
            pwm[i] = max(min(pwm[i], self.max_pwm), -self.max_pwm)

        # Send to STM32
        command = f"{pwm[0]},{pwm[1]},{pwm[2]},{pwm[3]}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent PWM: {command.strip()}")

    def __del__(self):
        self.serial_port.close()

def main():
    rclpy.init()
    node = PWMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
