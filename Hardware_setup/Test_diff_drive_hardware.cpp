#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "stm32f4xx_hal.h"  // STM32 HAL library
#include <cmath>

class DiffDriveHardwareSTM32 : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Load parameters from URDF
    wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
    ticks_per_rev_ = std::stoi(info_.hardware_parameters["ticks_per_rev"]);

    hw_states_.resize(4, 0.0);  // [left_pos, right_pos, left_vel, right_vel]
    hw_commands_.resize(2, 0.0);  // [left_vel_cmd, right_vel_cmd]

    // Initialize STM32 hardware
    if (!init_hardware()) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardwareSTM32"), "Failed to initialize hardware");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardwareSTM32"), "Hardware initialized");
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
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardwareSTM32"), "Activating hardware...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardwareSTM32"), "Deactivating hardware...");
    set_pwm_duty(htim1, TIM_CHANNEL_1, 0);  // Stop left motor
    set_pwm_duty(htim1, TIM_CHANNEL_2, 0);  // Stop right motor
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) override {
    double dt = period.seconds();
    if (dt <= 0) dt = 0.02;

    // Read encoder counts (using HAL timer input capture)
    int32_t left_count = __HAL_TIM_GET_COUNTER(&htim3);
    int32_t right_count = __HAL_TIM_GET_COUNTER(&htim4);

    hw_states_[0] = (left_count * 2 * M_PI) / ticks_per_rev_;  // Position in radians
    hw_states_[1] = (right_count * 2 * M_PI) / ticks_per_rev_;
    hw_states_[2] = (hw_states_[0] - last_left_pos_) / dt;     // Velocity
    hw_states_[3] = (hw_states_[1] - last_right_pos_) / dt;

    last_left_pos_ = hw_states_[0];
    last_right_pos_ = hw_states_[1];

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    double left_vel = hw_commands_[0];
    double right_vel = hw_commands_[1];

    // Convert velocity to PWM duty cycle (0-1000, assuming 10-bit PWM)
    int left_duty = velocity_to_pwm(left_vel);
    int right_duty = velocity_to_pwm(right_vel);

    // Set direction
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, left_vel >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, right_vel >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Set PWM
    set_pwm_duty(htim1, TIM_CHANNEL_1, std::abs(left_duty));
    set_pwm_duty(htim1, TIM_CHANNEL_2, std::abs(right_duty));

    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_states_;
  std::vector<double> hw_commands_;
  double wheel_separation_, wheel_radius_;
  int ticks_per_rev_;
  double last_left_pos_ = 0.0, last_right_pos_ = 0.0;

  TIM_HandleTypeDef htim1;  // PWM timer
  TIM_HandleTypeDef htim3;  // Left encoder
  TIM_HandleTypeDef htim4;  // Right encoder

  bool init_hardware() {
    // GPIO and timer initialization (simplified, use STM32CubeMX for full config)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    // PWM setup (TIM1, PA8 and PA9, 50 Hz, 10-bit resolution)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 167;  // 168 MHz / (167+1) = 1 MHz
    htim1.Init.Period = 999;     // 1 MHz / (999+1) = 1 kHz PWM
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // Direction pins (PB0, PB1)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Encoder setup (TIM3: PB4, PB5; TIM4: PB6, PB7)
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.Period = 65535;
    HAL_TIM_Encoder_Init(&htim3, TIM_ENCODERMODE_TI12);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.Period = 65535;
    HAL_TIM_Encoder_Init(&htim4, TIM_ENCODERMODE_TI12);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    return true;
  }

  int velocity_to_pwm(double vel) {
    const double max_vel = 2.0;  // Max velocity in rad/s (tune this)
    double duty = std::abs(vel) / max_vel;
    return static_cast<int>(duty * 1000);  // 0-1000 for 10-bit PWM
  }

  void set_pwm_duty(TIM_HandleTypeDef &htim, uint32_t channel, int duty) {
    __HAL_TIM_SET_COMPARE(&htim, channel, std::min(std::max(duty, 0), 1000));
  }
};

#include <hardware_interface/resource_manager.hpp>
REGISTER_SYSTEM(DiffDriveHardwareSTM32, "your_hardware_interface/DiffDriveHardwareSTM32")
