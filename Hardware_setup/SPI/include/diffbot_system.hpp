#ifndef DIFFDRIVE_STM32F4__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_STM32F4__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_stm32f4/visibility_control.h" // Updated

#include "diffdrive_stm32f4/stm32_spi_comms.hpp" // Updated
#include "diffdrive_stm32f4/wheel.hpp" // Updated

namespace diffdrive_stm32f4 // Updated
{
class DiffDriveSTM32F4Hardware : public hardware_interface::SystemInterface // Updated
{
    struct Config
    {
        std::string left_wheel_name = "";
        std::string right_wheel_name = "";
        float loop_rate = 0.0;
        std::string spi_device = "";
        int speed_hz = 0;
        int mode = 0;
        int enc_counts_per_rev = 0;
        int pid_p = 0;
        int pid_d = 0;
        int pid_i = 0;
        int pid_o = 0;
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveSTM32F4Hardware); // Updated

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    DIFFDRIVE_STM32F4_PUBLIC // Updated
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    STM32SPIComms comms_;
    Config cfg_;
    Wheel wheel_l_;
    Wheel wheel_r_;
};

}  // namespace diffdrive_stm32f4 // Updated

#endif  // DIFFDRIVE_STM32F4__DIFFBOT_SYSTEM_HPP_
