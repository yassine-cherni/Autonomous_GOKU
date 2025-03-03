#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
/* Draft for using ros2_control
It Writes motor velocity commands to the STM32F4 and Reads encoder positions/velocities from the STM32F4.
*/

class STM32F4HardwareInterface : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    // Initialize SPI (e.g., using spidev)
    // Define 2 joints (left_wheel, right_wheel)
    joint_names_ = {"left_wheel", "right_wheel"};
    velocities_.resize(2, 0.0);
    positions_.resize(2, 0.0);
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < 2; ++i) {
      state_interfaces.emplace_back(joint_names_[i], "position", &positions_[i]);
      state_interfaces.emplace_back(joint_names_[i], "velocity", &velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < 2; ++i) {
      command_interfaces.emplace_back(joint_names_[i], "velocity", &velocities_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Read encoder data from STM32F4 via SPI
    // Example SPI packet: [Left_Enc: 123456, Right_Enc: 123450, Left_Vel: 500, Right_Vel: 498]
    uint8_t rx_buffer[16];
    spi_read(rx_buffer, 16); // Your SPI read function
    positions_[0] = decode_int32(rx_buffer + 1);  // Left encoder counts
    positions_[1] = decode_int32(rx_buffer + 5);  // Right encoder counts
    velocities_[0] = decode_int16(rx_buffer + 9); // Left velocity (ticks/s)
    velocities_[1] = decode_int16(rx_buffer + 11); // Right velocity (ticks/s)
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Send velocity commands to STM32F4 via SPI
    // Example packet: [Mode: 1, Left_Vel: 1050, Right_Vel: 1050, E-Stop: 0]
    uint8_t tx_buffer[8] = {0xAA, 1, (uint8_t)(velocities_[0] >> 8), (uint8_t)velocities_[0],
                            (uint8_t)(velocities_[1] >> 8), (uint8_t)velocities_[1], 0, 0xXX};
    spi_write(tx_buffer, 8); // Your SPI write function
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> velocities_, positions_;
};
