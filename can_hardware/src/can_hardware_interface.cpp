#include "my_can_hardware/can_hardware_interface.hpp"

CanHardwareInterface::CanHardwareInterface() : diagnostic_status_(0) {}

hardware_interface::CallbackReturn CanHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = std::make_shared<rclcpp::Node>("can_hardware_node");
    status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "diagnostic_status", 10, std::bind(&CanHardwareInterface::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "CAN Hardware Interface initialized");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "can_bus", "diagnostic_status", &diagnostic_status_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanHardwareInterface::export_command_interfaces() {
    return std::vector<hardware_interface::CommandInterface>{};
}

hardware_interface::CallbackReturn CanHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(node_->get_logger(), "CAN Hardware Interface activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(node_->get_logger(), "CAN Hardware Interface deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CanHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    rclcpp::spin_some(node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    return hardware_interface::return_type::OK;
}

void CanHardwareInterface::statusCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    diagnostic_status_ = msg->data;
    RCLCPP_INFO(node_->get_logger(), "Updated diagnostic status: %d", diagnostic_status_);
}
