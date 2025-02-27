#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#define PWM_MAX 1000
#define RAMP_STEP 10
#define RAMP_DELAY_MS 20
#define WHEEL_RADIUS 0.05
#define WHEEL_BASE 0.3
#define TICKS_PER_REV 360

TIM_HandleTypeDef htim1, htim2, htim3;
I2C_HandleTypeDef hi2c1; // For MPU-6050
volatile int32_t encoder_counts[4] = {0};
int pwm_current[4] = {0}, pwm_target[4] = {0};
float pos_x = 0, pos_y = 0, theta = 0;
float accel[3] = {0}, gyro[3] = {0};

rcl_subscription_t cmd_sub;
rcl_publisher_t odom_pub, imu_pub;
geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) encoder_counts[0] = __HAL_TIM_GET_COUNTER(htim); // FL
    // Add for other wheels
}

void read_imu() {
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100);
    accel[0] = (int16_t)(buffer[0] << 8 | buffer[1]) / 16384.0; // Accel X (g)
    accel[1] = (int16_t)(buffer[2] << 8 | buffer[3]) / 16384.0; // Accel Y
    accel[2] = (int16_t)(buffer[4] << 8 | buffer[5]) / 16384.0; // Accel Z
    gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]) / 131.0;  // Gyro Z (deg/s)
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    pwm_target[0] = pwm_target[2] = (msg->linear.x - msg->angular.z * WHEEL_BASE / 2) * PWM_MAX;
    pwm_target[1] = pwm_target[3] = (msg->linear.x + msg->angular.z * WHEEL_BASE / 2) * PWM_MAX;
}

void update_pwm_and_odom() {
    for (int i = 0; i < 4; i++) {
        pwm_current[i] += (pwm_target[i] > pwm_current[i]) ? RAMP_STEP : -RAMP_STEP;
        pwm_current[i] = (pwm_current[i] > PWM_MAX) ? PWM_MAX : (pwm_current[i] < -PWM_MAX) ? -PWM_MAX : pwm_current[i];
    }
    // PWM output as before

    float dt = RAMP_DELAY_MS / 1000.0;
    float left_dist = encoder_counts[0] * WHEEL_RADIUS / TICKS_PER_REV;
    float right_dist = encoder_counts[1] * WHEEL_RADIUS / TICKS_PER_REV;
    float vel_linear = (left_dist + right_dist) / (2.0 * dt);
    float vel_angular = (right_dist - left_dist) / (WHEEL_BASE * dt);

    pos_x += vel_linear * cos(theta) * dt;
    pos_y += vel_linear * sin(theta) * dt;
    theta += vel_angular * dt;

    odom_msg.header.stamp = rclc_now();
    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data = "base_link";
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2);
    odom_msg.pose.pose.orientation.w = cos(theta / 2);
    odom_msg.twist.twist.linear.x = vel_linear;
    odom_msg.twist.twist.angular.z = vel_angular;
    rcl_publish(&odom_pub, &odom_msg, NULL);

    read_imu();
    imu_msg.header.stamp = odom_msg.header.stamp;
    imu_msg.linear_acceleration.x = accel[0] * 9.81; // m/s^2
    imu_msg.linear_acceleration.y = accel[1] * 9.81;
    imu_msg.linear_acceleration.z = accel[2] * 9.81;
    imu_msg.angular_velocity.z = gyro[2] * M_PI / 180.0; // rad/s
    rcl_publish(&imu_pub, &imu_msg, NULL);
}

void setup() {
    set_microros_serial_transports(USART1);
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &rcl_get_default_allocator());
    rcl_node_t node;
    rclc_node_init_default(&node, "stm32_node", "", &support);

    rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
    rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &rcl_get_default_allocator());
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, cmd_vel_callback, ON_NEW_DATA);

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init(); // For MPU-6050
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void loop() {
    update_pwm_and_odom();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    HAL_Delay(RAMP_DELAY_MS);
}
