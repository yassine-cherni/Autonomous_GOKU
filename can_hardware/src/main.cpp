#include "my_can_hardware/can_bus_interface.hpp"
#include "my_can_hardware/can_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create and spin the CAN bus interface node
    auto can_node = std::make_shared<CanBusInterface>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(can_node);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
