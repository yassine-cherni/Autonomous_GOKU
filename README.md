Below is a customized README tailored to your specific robot setup, which I’ll call **"Skippy"** (a playful nod to its mobility and your preference to "skip" Micro-ROS). This README reflects your hardware (Raspberry Pi 5 with ROS2, STM32F4 for motor control, BLDC motors with encoders, and sensors on the Pi), incorporates `ros2_control`, and aligns with the differential drive configuration we’ve discussed. I’ve streamlined it for clarity and relevance to your project.

---

```markdown
# Skippy: Autonomous Mobile Robot with ROS2

**Skippy** is a differential-drive autonomous robot built for indoor navigation, powered by ROS2 Humble on a Raspberry Pi 5 and real-time motor control on an STM32F4. It features BLDC motors with encoders for precise odometry, alongside LiDAR, IMU, and camera sensors for mapping and perception. The robot uses the `ros2_control` framework for seamless integration with ROS2 navigation stacks.

---

## Features
- **Hardware**:
  - Raspberry Pi 5: ROS2 host, sensor processing (LiDAR, IMU, camera).
  - STM32F4: BLDC motor control and encoder feedback via SPI.
  - 2 BLDC motors with built-in encoders (differential drive).
- **Control**: Velocity-based control using `diff_drive_controller` in `ros2_control`.
- **Sensors**: LiDAR (e.g., RPLIDAR), IMU (e.g., MPU6050), and camera (e.g., USB or CSI) on the Pi.
- **ROS2 Distro**: Humble Hawksbill (2022).
- **Enhancements**:
  - Odometry fusion (encoders + IMU).
  - Teleoperation script for manual control.
  - Diagnostics for motor status monitoring.

---

## Prerequisites
### Hardware
- Raspberry Pi 5 (Ubuntu 22.04 LTS).
- STM32F4 (e.g., STM32F407VG Discovery).
- 2 BLDC motors with encoders and compatible driver (e.g., ESC or custom H-bridge).
- SPI connection (Pi GPIO 10/11/9 to STM32 PA5/PA7/PA4).
- Sensors: LiDAR (USB/UART), IMU (I2C/SPI), camera (USB/CSI).

### Software
- ROS2 Humble: `sudo apt install ros-humble-desktop`.
- `ros2_control` and controllers: `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers`.
- STM32CubeIDE for firmware development.
- Python dependencies: `pip install spidev`.

### Tools
- Git, CMake, `colcon` for building ROS2 packages.

---

## Project Structure
```
skippy_robot/
├── config/                   # Configuration files (e.g., robot_control.yaml)
├── launch/                   # Launch files (e.g., robot_control.launch.py)
├── src/                      # Source code
│   ├── stm32f4_hardware_interface.cpp  # Custom ros2_control hardware interface
│   ├── teleop.py             # Teleoperation script
│   └── diagnostics.py        # Diagnostics node
├── README.md                 # This file
└── package.xml               # ROS2 package manifest
```

---

## Setup Instructions
### 1. Raspberry Pi 5 Setup
1. Install Ubuntu 22.04 and ROS2 Humble (see [ROS2 docs](https://docs.ros.org/en/humble/Installation.html)).
2. Enable SPI:
   - Edit `/boot/firmware/config.txt`: Add `dtparam=spi=on`.
   - Reboot: `sudo reboot`.
3. Clone and build:
   ```bash
   git clone <repo-url> ~/ros2_ws/src/skippy_robot
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### 2. STM32F4 Firmware
1. Open STM32CubeIDE and create a project for your STM32F4.
2. Configure:
   - SPI1: PA5 (SCK), PA7 (MOSI), PA6 (MISO), PA4 (CS), slave mode.
   - TIM2/TIM3: Encoder mode for left/right motor encoders.
   - PWM: TIM1 CH1/CH2 for BLDC motor control (e.g., PA8/PA9).
3. Implement SPI communication (see example below).
4. Build and flash the firmware.

### 3. Hardware Connections
- **SPI**: Pi GPIO 10 (MOSI) → STM32 PA7, GPIO 11 (SCK) → PA5, GPIO 9 (MISO) → PA6, GPIO 8 (CS) → PA4, common ground.
- **Motors**:
  - Left: TIM1 CH1 (PA8) to motor driver, encoder to TIM2.
  - Right: TIM1 CH2 (PA9) to motor driver, encoder to TIM3.
- **Power**: 5V/3.3V for Pi/STM32, separate supply (e.g., 12V) for motors.

### 4. ROS2 Control Setup
- Configuration: See `config/robot_control.yaml`.
- Launch: `ros2 launch skippy_robot robot_control.launch.py`.

---

## STM32F4 Firmware Example
```c
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1, htim2, htim3;

void spi_handler() {
  uint8_t rx_buffer[8], tx_buffer[16];
  if (HAL_SPI_Receive(&hspi1, rx_buffer, 8, 100) == HAL_OK && rx_buffer[0] == 0xAA) {
    uint8_t mode = rx_buffer[1];
    int16_t left_vel = (rx_buffer[2] << 8) | rx_buffer[3];
    int16_t right_vel = (rx_buffer[4] << 8) | rx_buffer[5];
    if (mode == 1) {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Left motor
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_vel);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Right motor
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_vel);
    }
  }
  // Send encoder data
  int32_t left_enc = __HAL_TIM_GET_COUNTER(&htim2);
  int32_t right_enc = __HAL_TIM_GET_COUNTER(&htim3);
  int16_t left_vel = compute_velocity(left_enc); // Custom function
  int16_t right_vel = compute_velocity(right_enc);
  tx_buffer[0] = 0xBB;
  memcpy(tx_buffer + 1, &left_enc, 4);
  memcpy(tx_buffer + 5, &right_enc, 4);
  memcpy(tx_buffer + 9, &left_vel, 2);
  memcpy(tx_buffer + 11, &right_vel, 2);
  tx_buffer[15] = 0xXX; // Checksum (simplified)
  HAL_SPI_Transmit(&hspi1, tx_buffer, 16, 100);
}

int main() {
  // HAL init, SPI/TIM setup (generated by STM32CubeIDE)
  while (1) {
    spi_handler();
  }
}
```

---

## ROS2 Control Configuration
### `config/robot_control.yaml`
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

stm32f4_hardware:
  ros__parameters:
    joints:
      - left_wheel
      - right_wheel
    wheel_separation: 0.4  # Adjust to your robot
    wheel_radius: 0.08    # Adjust to your robot
    encoder_resolution: 4000  # Adjust to your encoders

diff_drive_controller:
  ros__parameters:
    left_wheel: "left_wheel"
    right_wheel: "right_wheel"
    wheel_separation: 0.4
    wheel_radius: 0.08
    publish_rate: 50.0
    base_frame_id: "base_link"
    odom_frame_id: "odom"
```

### `launch/robot_control.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=["/path/to/skippy_robot/config/robot_control.yaml"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen",
        ),
    ])
```

---

## Usage
1. **Launch Control**:
   ```bash
   ros2 launch skippy_robot robot_control.launch.py
   ```
2. **Teleoperation**:
   ```bash
   ros2 run skippy_robot teleop.py
   ```
3. **Monitor Diagnostics**:
   ```bash
   ros2 topic echo /diagnostics
   ```

---

## Teleoperation Example
```python
# src/teleop.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5  # m/s
        self.turn = 1.0   # rad/s

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.1)[0] else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("WASD to move, Q to quit")
        while True:
            key = self.get_key()
            twist = Twist()
            if key == 'w': twist.linear.x = self.speed
            elif key == 's': twist.linear.x = -self.speed
            elif key == 'a': twist.angular.z = self.turn
            elif key == 'd': twist.angular.z = -self.turn
            elif key == 'q': break
            self.pub.publish(twist)

def main():
    rclpy.init()
    Teleop().run()
    rclpy.shutdown()
```

---

## Tuning
- **Wheel Parameters**: Adjust `wheel_separation` and `wheel_radius` in `robot_control.yaml`.
- **Encoder Resolution**: Match `encoder_resolution` to your BLDC motor encoders.
- **PWM Scaling**: Map velocity commands (e.g., 0–1000) to your motor driver’s range in STM32 firmware.

---

## Future Enhancements
- Add PID control on STM32F4 for velocity stabilization.
- Integrate IMU data into `diff_drive_controller` for better odometry.
- Support for SLAM (e.g., Cartographer) with LiDAR.

---

## License
MIT License - free to use and modify!

---

## About
**Skippy** is a lean, efficient robot designed for learning ROS2 and autonomous navigation. Built by [Your Name] with love for robotics!
```

---

### Key Customizations
1. **Robot Name**: Changed to "Skippy" for personality and relevance.
2. **Hardware**: Reflects your setup (Pi 5 with sensors, STM32F4 with BLDC motors + encoders, SPI comms).
3. **ROS2 Distro**: Updated to Humble (stable for Pi 5, unlike Jazzy which is newer).
4. **Controller**: Focuses on `ros2_control` with `diff_drive_controller`, matching your differential drive design.
5. **Structure**: Simplified to your files (no Gazebo simulation yet, as you didn’t mention it).
6. **Examples**: Includes tailored STM32 firmware, ROS2 config, and teleop script.

Let me know if you’d like to tweak the name, add simulation details, or adjust anything else! This README should guide you or anyone else through setting up and running Skippy.
