#include "main.h"
#include "usart.h"
#include "tim.h"

#define PWM_MAX 1000       // Timer ARR value
#define RAMP_STEP 10       // PWM increment per step
#define RAMP_DELAY_MS 20   // Delay between steps

UART_HandleTypeDef huart2; // UART2 for Pi communication
TIM_HandleTypeDef htim1;   // TIM1 for front motors
TIM_HandleTypeDef htim2;   // TIM2 for rear motors

int pwm_current[4] = {0, 0, 0, 0}; // FL, FR, RL, RR
int pwm_target[4] = {0, 0, 0, 0};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

void update_pwm(void) {
    for (int i = 0; i < 4; i++) {
        if (pwm_current[i] < pwm_target[i]) {
            pwm_current[i] = (pwm_current[i] + RAMP_STEP > pwm_target[i]) ? pwm_target[i] : pwm_current[i] + RAMP_STEP;
        } else if (pwm_current[i] > pwm_target[i]) {
            pwm_current[i] = (pwm_current[i] - RAMP_STEP < pwm_target[i]) ? pwm_target[i] : pwm_current[i] - RAMP_STEP;
        }
    }

    // Set PWM (TIM1 CH1-2 for front, TIM2 CH1-2 for rear)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(pwm_current[0])); // FL
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs(pwm_current[1])); // FR
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(pwm_current[2])); // RL
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(pwm_current[3])); // RR

    // Direction control
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, pwm_current[0] >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // FL dir
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, pwm_current[1] >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // FR dir
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, pwm_current[2] >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // RL dir
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, pwm_current[3] >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET); // RR dir
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t rx_buffer[40];
    if (huart->Instance == USART2) {
        if (sscanf((char*)rx_buffer, "%d,%d,%d,%d", &pwm_target[0], &pwm_target[1], &pwm_target[2], &pwm_target[3]) == 4) {
            for (int i = 0; i < 4; i++) {
                pwm_target[i] = (pwm_target[i] > PWM_MAX) ? PWM_MAX : (pwm_target[i] < -PWM_MAX) ? -PWM_MAX : pwm_target[i];
            }
        }
        HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    uint8_t rx_buffer[40];
    HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));

    while (1) {
        update_pwm();
        HAL_Delay(RAMP_DELAY_MS);
    }
}
