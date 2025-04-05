#ifndef CAN_HARDWARE_INTERFACE_HPP
#define CAN_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class CanHardwareInterface : public hardware_interface::SystemInterface {
public:
    CanHardwareInterface();
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void statusCallback(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
    int diagnostic_status_;
};

#endif // CAN_HARDWARE_INTERFACE_HPP
