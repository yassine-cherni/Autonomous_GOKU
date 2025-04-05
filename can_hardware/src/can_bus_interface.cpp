#include "my_can_hardware/can_bus_interface.hpp"

CanBusInterface::CanBusInterface() : Node("can_bus_interface") {
    can_tx_pub_ = create_publisher<can_msgs::msg::Frame>("can/tx", 10);
    can_rx_sub_ = create_subscription<can_msgs::msg::Frame>(
        "can/rx", 10, std::bind(&CanBusInterface::canCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::Int32>("diagnostic_status", 10);

    RCLCPP_INFO(this->get_logger(), "CAN Bus Interface initialized");
}

void CanBusInterface::canCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    if (msg->id == 0x101) { // Example CAN ID for status
        int status = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Received CAN status: %d", status);

        auto status_msg = std_msgs::msg::Int32();
        status_msg.data = status;
        status_pub_->publish(status_msg);
    }
}

void CanBusInterface::sendCommand(uint32_t id, uint8_t data) {
    auto frame = can_msgs::msg::Frame();
    frame.id = id;
    frame.data = {data};
    frame.is_extended = false;
    frame.is_rtr = false;
    can_tx_pub_->publish(frame);
}
