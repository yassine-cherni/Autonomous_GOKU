# Goku Autonomous Robot with ROS 2 Jazzy

This project implements an all-terrain robot with smooth PWM control for four motors, leveraging ROS 2 Jazzy Jalisco, a Raspberry Pi 5, an STM32F4 microcontroller, and Gazebo Harmonic for simulation. The robot uses precise PWM ramping to ensure smooth acceleration and deceleration, suitable for indoor navigation over varied surfaces.

## Features

- **Hardware**: Raspberry Pi 5 (ROS 2 host) + STM32F4 (PWM control)
- **Control**: 4 PWM channels for a four-wheeled robot (e.g., skid-steer or mecanum)
- **Simulation**: Gazebo Harmonic with an indoor environment
- **Enhancements**:
  - PID control for velocity stabilization
  - Diagnostics node for real-time motor and system monitoring
  - Teleop script for manual control
- **ROS 2 Distro**: Jazzy Jalisco (May 2024)

## Prerequisites

- **Hardware**:
  - Raspberry Pi 5 (Ubuntu 24.04 Noble)
  - STM32F4 (e.g., STM32F407 Discovery)
  - Motor driver (e.g., L298N) and 4 DC motors
  - UART connection (Pi GPIO 14/15 to STM32 PA2/PA3)
- **Software**:
  - ROS 2 Jazzy (`sudo apt install ros-jazzy-desktop`)
  - Gazebo Harmonic (`sudo apt install ros-jazzy-ros-gz`)
  - STM32CubeIDE
  - Python dependencies: `pip install pyserial`
- **Tools**: Git, CMake, colcon

## Project Structure

```
all_terrain_robot/
├── launch/               # ROS 2 launch files
├── src/                  # Source code
│   ├── pwm_controller.py # ROS 2 node for PWM commands
│   ├── teleop.py         # Teleop script
│   ├── diagnostics.py    # Diagnostics node
│   └── stm32/            # STM32 firmware
├── urdf/                 # Robot URDF
├── worlds/               # Gazebo world files
├── README.md
└── package.xml           # ROS 2 package manifest
```

## Setup Instructions

### 1. Raspberry Pi 5 Setup

1. Install Ubuntu 24.04 and ROS 2 Jazzy (see [ROS 2 docs](https://docs.ros.org/en/jazzy/)).
2. Enable UART:
   - Edit `/boot/firmware/config.txt`:
     ```
     enable_uart=1
     dtoverlay=disable-bt
     ```
   - Reboot.
3. Clone this repo:
   ```
   git clone <repo-url> ~/ros2_ws/src/all_terrain_robot
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### 2. STM32F4 Firmware

1. Open STM32CubeIDE and create a new project for your STM32F4 board.
2. Configure:
   - **TIM1**: PWM on CH1 (PA8), CH2 (PA9), 1 kHz, ARR = 1000
   - **TIM2**: PWM on CH1 (PB3), CH2 (PB10), 1 kHz, ARR = 1000
   - **USART2**: 115200 baud, PA2-TX, PA3-RX
   - **GPIO**: PA0–PA3 for motor direction
3. Copy `stm32/main.c` from this repo into your project.
4. Build and flash the firmware.

### 3. Hardware Connections

- **UART**: Pi GPIO 14 (TX) → STM32 PA3 (RX), Pi GPIO 15 (RX) → STM32 PA2 (TX), common ground
- **Motors**:
  - Front Left: TIM1 CH1 (PA8), direction PA0
  - Front Right: TIM1 CH2 (PA9), direction PA1
  - Rear Left: TIM2 CH1 (PB3), direction PA2
  - Rear Right: TIM2 CH2 (PB10), direction PA3
- **Power**: Separate supply for motors (e.g., 12V); 3.3V for STM32 if needed

### 4. Simulation Setup

1. Ensure Gazebo Harmonic is installed.
2. Launch the simulation:
   ```
   ros2 launch all_terrain_robot launch_sim.py
   ```

## Enhancements

### PID Control

Added to the STM32 firmware for velocity stabilization:

```c
// In stm32/main.c
float kp = 0.5, ki = 0.1, kd = 0.05; // PID gains
int error[4] = {0}, integral[4] = {0}, last_error[4] = {0};

void update_pwm(void) {
    for (int i = 0; i < 4; i++) {
        error[i] = pwm_target[i] - pwm_current[i];
        integral[i] += error[i];
        int derivative = error[i] - last_error[i];
        int pid_output = kp * error[i] + ki * integral[i] + kd * derivative;

        pwm_current[i] += (pid_output > RAMP_STEP) ? RAMP_STEP : (pid_output < -RAMP_STEP) ? -RAMP_STEP : pid_output;
        pwm_current[i] = (pwm_current[i] > PWM_MAX) ? PWM_MAX : (pwm_current[i] < -PWM_MAX) ? -PWM_MAX : pwm_current[i];
        last_error[i] = error[i];
    }
    // Set PWM and direction as before
}
```

- **Tuning**: Adjust `kp`, `ki`, `kd` based on motor response (start with small values).

### Diagnostics Node

Monitors motor PWM and system status:

```python
# src/diagnostics.py
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class Diagnostics(Node):
    def __init__(self):
        super().__init__('diagnostics')
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        self.pwm_values = [0] * 4

    def update_pwm(self, pwm):
        self.pwm_values = pwm

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        status = DiagnosticStatus(name='Motor Control', level=DiagnosticStatus.OK, message='Running')
        for i, pwm in enumerate(self.pwm_values):
            status.values.append(KeyValue(key=f'Motor {i}', value=str(pwm)))
        msg.status.append(status)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Diagnostics()
    rclpy.spin(node)
```

- Add to launch file:
  ```python
  Node(package='all_terrain_robot', executable='diagnostics.py', output='screen')
  ```

### Teleop Script

For manual control:

```python
# src/teleop.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.5
        self.turn = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
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
    node = Teleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
```

- Run: `ros2 run all_terrain_robot teleop.py`

## Usage

- **Simulation**: 
  ```
  ros2 launch all_terrain_robot launch_sim.py
  ```
- **Hardware**:
  1. Start PWM controller:
     ```
     ros2 run all_terrain_robot pwm_controller.py
     ```
  2. Test with teleop:
     ```
     ros2 run all_terrain_robot teleop.py
     ```
  3. Monitor:
     ```
     ros2 topic echo /diagnostics
     ```

## Tuning

- **PWM Ramping**: Adjust `RAMP_STEP` (5–20) and `RAMP_DELAY_MS` (10–50) in `stm32/main.c`
- **PID**: Tune `kp`, `ki`, `kd` for stable velocity
- **Kinematics**: Modify `wheelbase` and `track_width` in `pwm_controller.py`

## Future Enhancements

- Add IMU feedback for tilt compensation
- Implement mecanum wheel kinematics
- Port to a custom `ros2_control` hardware interface for STM32

## License

MIT License - feel free to use and modify!
