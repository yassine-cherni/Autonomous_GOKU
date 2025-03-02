#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class DiffDriveHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Load parameters from URDF
    wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
    left_motor_port_ = info_.hardware_parameters["left_motor_port"];
    right_motor_port_ = info_.hardware_parameters["right_motor_port"];

    // Initialize hardware (e.g., serial connection)
    // Replace with your motor controller initialization
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Initializing hardware...");
    // Example: Open serial ports, initialize I2C, etc.

    hw_states_.resize(4, 0.0);  // [left_pos, right_pos, left_vel, right_vel]
    hw_commands_.resize(2, 0.0);  // [left_vel_cmd, right_vel_cmd]

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("left_wheel_joint", "position", &hw_states_[0]);
    state_interfaces.emplace_back("right_wheel_joint", "position", &hw_states_[1]);
    state_interfaces.emplace_back("left_wheel_joint", "velocity", &hw_states_[2]);
    state_interfaces.emplace_back("right_wheel_joint", "velocity", &hw_states_[3]);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("left_wheel_joint", "velocity", &hw_commands_[0]);
    command_interfaces.emplace_back("right_wheel_joint", "velocity", &hw_commands_[1]);
    return command_interfaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Activating hardware...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Deactivating hardware...");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // Read encoder data from your hardware
    // Example: Replace with actual encoder reading logic
    double left_pos = read_encoder_left();  // Your function
    double right_pos = read_encoder_right();
    double dt = 0.02;  // Assume 50 Hz update rate

    hw_states_[0] = left_pos;
    hw_states_[1] = right_pos;
    hw_states_[2] = (left_pos - last_left_pos_) / dt;  // Velocity estimation
    hw_states_[3] = (right_pos - last_right_pos_) / dt;

    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    // Send velocity commands to your motors
    double left_vel_cmd = hw_commands_[0];
    double right_vel_cmd = hw_commands_[1];

    // Example: Replace with your motor control logic
    send_velocity_to_motors(left_vel_cmd, right_vel_cmd);

    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_states_;    // [left_pos, right_pos, left_vel, right_vel]
  std::vector<double> hw_commands_;  // [left_vel_cmd, right_vel_cmd]
  double wheel_separation_, wheel_radius_;
  std::string left_motor_port_, right_motor_port_;
  double last_left_pos_ = 0.0, last_right_pos_ = 0.0;

  // Replace these with your hardware-specific functions
  double read_encoder_left() { return 0.0; }  // Implement encoder reading
  double read_encoder_right() { return 0.0; }
  void send_velocity_to_motors(double left_vel, double right_vel) {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Left: %f, Right: %f", left_vel, right_vel);
    // Implement motor control (e.g., PWM, serial commands)
  }
};

#include <hardware_interface/resource_manager.hpp>
REGISTER_SYSTEM(DiffDriveHardware, "your_hardware_interface/DiffDriveHardware")
