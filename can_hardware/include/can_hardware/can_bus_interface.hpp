#ifndef CAN_BUS_INTERFACE_HPP
#define CAN_BUS_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/int32.hpp>

class CanBusInterface : public rclcpp::Node {
public:
    CanBusInterface();
    void sendCommand(uint32_t id, uint8_t data);

private:
    void canCallback(const can_msgs::msg::Frame::SharedPtr msg);

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
};

#endif // CAN_BUS_INTERFACE_HPP
