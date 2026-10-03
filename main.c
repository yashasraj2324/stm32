/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F446RE Motor + Dual Ultrasonic Controller
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ============================================================
 * UART
 * ============================================================ */

#define UART_RX_BUFFER_SIZE       64U
#define UART_TX_QUEUE_DEPTH       8U
#define UART_TX_MESSAGE_SIZE      96U

uint8_t uart_rx_byte;
char uart_rx_buffer[UART_RX_BUFFER_SIZE];
char uart_command_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_rx_discard = 0;
volatile uint8_t uart_command_ready = 0;

typedef struct
{
    uint8_t data[UART_TX_MESSAGE_SIZE];
    uint16_t length;
} UART_TxMessage;

static UART_TxMessage uart_tx_queue[UART_TX_QUEUE_DEPTH];
static volatile uint8_t uart_tx_head = 0;
static volatile uint8_t uart_tx_tail = 0;
static volatile uint8_t uart_tx_busy = 0;

/* ============================================================
 * Motor configuration
 * ============================================================ */

#define PWM_PERIOD                4199U
#define MAX_WHEEL_SPEED_MPS       1.0f
#define MOTOR_COMMAND_TIMEOUT_MS  500U
#define MOTOR_RAMP_STEP_MPS       0.05f
#define MOTOR_UPDATE_PERIOD_MS    10U
#define MOTOR_DIRECTION_DEADTIME_US 50U
#define MOTOR_REVERSAL_HOLD_MS    150U

volatile float target_left_velocity = 0.0f;
volatile float target_right_velocity = 0.0f;
static float applied_left_velocity = 0.0f;
static float applied_right_velocity = 0.0f;
static int8_t left_motor_direction = 0;
static int8_t right_motor_direction = 0;
static uint32_t left_reversal_hold_until = 0U;
static uint32_t right_reversal_hold_until = 0U;
static uint8_t left_reversal_pending = 0U;
static uint8_t right_reversal_pending = 0U;

volatile uint32_t last_command_tick = 0;
static uint32_t last_motor_update_tick = 0;

/* ============================================================
 * Ultrasonic configuration
 * ============================================================ */

#define ULTRASONIC_TIMEOUT_US     30000U
#define ULTRASONIC_UPDATE_MS      100U

/*
 * LEFT HC-SR04
 *
 * TRIG = PB0
 * ECHO = PB1
 *
 * RIGHT HC-SR04
 *
 * TRIG = PB2
 * ECHO = PC0
 */

volatile uint32_t left_distance_mm = 0;
volatile uint32_t right_distance_mm = 0;

uint32_t last_ultrasonic_tick = 0;

char distance_msg[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);

/* USER CODE BEGIN PFP */

/* UART */

static void UART_Send(const char *text);
static void UART_Tx_Service(void);
static void Process_Command(void);

/* Motors */

static void Motor_Set_Left(float velocity);
static void Motor_Set_Right(float velocity);
static void Motor_Stop(void);
static void Motor_Update(void);
static uint32_t Velocity_To_PWM(float velocity);

/* Ultrasonic */

static void DWT_Init(void);
static void Delay_us(uint32_t us);
static uint32_t DWT_Cycles_Per_Us(void);

static uint32_t Ultrasonic_Read(
    GPIO_TypeDef *TRIG_PORT,
    uint16_t TRIG_PIN,
    GPIO_TypeDef *ECHO_PORT,
    uint16_t ECHO_PIN
);

static void Motor_DeadTime(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 * UART SEND
 * ============================================================ */

static void UART_Send(const char *text)
{
    size_t text_length = strlen(text);
    uint8_t next_head;

    if ((text_length == 0U) ||
        (text_length >= UART_TX_MESSAGE_SIZE))
    {
        return;
    }

    next_head = (uint8_t)((uart_tx_head + 1U) %
                          UART_TX_QUEUE_DEPTH);

    if (next_head == uart_tx_tail)
    {
        return;
    }

    memcpy(uart_tx_queue[uart_tx_head].data,
           text,
           text_length);
    uart_tx_queue[uart_tx_head].length = (uint16_t)text_length;
    uart_tx_head = next_head;
}

static void UART_Tx_Service(void)
{
    if ((uart_tx_busy != 0U) ||
        (uart_tx_tail == uart_tx_head))
    {
        return;
    }

    uart_tx_busy = 1U;

    if (HAL_UART_Transmit_IT(
            &huart2,
            uart_tx_queue[uart_tx_tail].data,
            uart_tx_queue[uart_tx_tail].length) != HAL_OK)
    {
        uart_tx_busy = 0U;
        uart_tx_tail = (uint8_t)((uart_tx_tail + 1U) %
                                 UART_TX_QUEUE_DEPTH);
    }
}

/* ============================================================
 * VELOCITY → PWM
 *
 * 0.0 m/s = 0 PWM
 * 1.0 m/s = 4199 PWM
 * ============================================================ */

static uint32_t Velocity_To_PWM(float velocity)
{
    float abs_velocity;
    float pwm;

    if (velocity < 0.0f)
    {
        abs_velocity = -velocity;
    }
    else
    {
        abs_velocity = velocity;
    }

    if (abs_velocity >= MAX_WHEEL_SPEED_MPS)
    {
        return PWM_PERIOD;
    }

    pwm =
        (abs_velocity / MAX_WHEEL_SPEED_MPS)
        * (float)PWM_PERIOD;

    if (pwm < 0.0f)
    {
        pwm = 0.0f;
    }

    if (pwm > (float)PWM_PERIOD)
    {
        pwm = (float)PWM_PERIOD;
    }

    return (uint32_t)pwm;
}

/* ============================================================
 * LEFT MOTOR
 *
 * PA6 = TIM3_CH1 = RPWM
 * PA7 = TIM14_CH1 = LPWM
 * ============================================================ */

static void Motor_Set_Left(float velocity)
{
    uint32_t pwm = Velocity_To_PWM(velocity);
    int8_t direction = (velocity > 0.0f) ? 1 : ((velocity < 0.0f) ? -1 : 0);

    if (direction != left_motor_direction)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
        Motor_DeadTime();
        left_motor_direction = direction;
    }

    if (direction > 0)
    {
        /* Forward */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_1,
            pwm
        );

        __HAL_TIM_SET_COMPARE(
            &htim14,
            TIM_CHANNEL_1,
            0
        );
    }
    else if (direction < 0)
    {
        /* Reverse */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_1,
            0
        );

        __HAL_TIM_SET_COMPARE(
            &htim14,
            TIM_CHANNEL_1,
            pwm
        );
    }
    else
    {
        /* Stop */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_1,
            0
        );

        __HAL_TIM_SET_COMPARE(
            &htim14,
            TIM_CHANNEL_1,
            0
        );
    }
}

/* ============================================================
 * RIGHT MOTOR
 *
 * PC7  = TIM3_CH2 = RPWM
 * PB10 = TIM2_CH3 = LPWM
 * ============================================================ */

static void Motor_Set_Right(float velocity)
{
    uint32_t pwm = Velocity_To_PWM(velocity);
    int8_t direction = (velocity > 0.0f) ? 1 : ((velocity < 0.0f) ? -1 : 0);

    if (direction != right_motor_direction)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
        Motor_DeadTime();
        right_motor_direction = direction;
    }

    if (direction > 0)
    {
        /* Forward */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_2,
            pwm
        );

        __HAL_TIM_SET_COMPARE(
            &htim2,
            TIM_CHANNEL_3,
            0
        );
    }
    else if (direction < 0)
    {
        /* Reverse */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_2,
            0
        );

        __HAL_TIM_SET_COMPARE(
            &htim2,
            TIM_CHANNEL_3,
            pwm
        );
    }
    else
    {
        /* Stop */

        __HAL_TIM_SET_COMPARE(
            &htim3,
            TIM_CHANNEL_2,
            0
        );

        __HAL_TIM_SET_COMPARE(
            &htim2,
            TIM_CHANNEL_3,
            0
        );
    }
}

/* ============================================================
 * STOP BOTH MOTORS
 * ============================================================ */

static void Motor_Stop(void)
{
    target_left_velocity = 0.0f;
    target_right_velocity = 0.0f;
    applied_left_velocity = 0.0f;
    applied_right_velocity = 0.0f;
    left_motor_direction = 0;
    right_motor_direction = 0;
    left_reversal_hold_until = 0U;
    right_reversal_hold_until = 0U;
    left_reversal_pending = 0U;
    right_reversal_pending = 0U;

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim14,
        TIM_CHANNEL_1,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_2,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_3,
        0
    );
}

static float Motor_Ramp_Value(float current, float target)
{
    if (current < target)
    {
        current += MOTOR_RAMP_STEP_MPS;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= MOTOR_RAMP_STEP_MPS;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

static float Motor_Update_Wheel(float current,
                                float target,
                                uint32_t now,
                                uint32_t *hold_until,
                                uint8_t *reversal_pending)
{
    if ((target == 0.0f) &&
        (current == 0.0f))
    {
        *reversal_pending = 0U;
        *hold_until = 0U;
    }

    if ((current != 0.0f) &&
        (target != 0.0f) &&
        (((current > 0.0f) && (target > 0.0f)) ||
         ((current < 0.0f) && (target < 0.0f))))
    {
        *reversal_pending = 0U;
        *hold_until = 0U;
    }

    if ((current != 0.0f) &&
        (target != 0.0f) &&
        (((current > 0.0f) && (target < 0.0f)) ||
         ((current < 0.0f) && (target > 0.0f))))
    {
        target = 0.0f;
        *reversal_pending = 1U;
    }

    if ((current == 0.0f) &&
        (*reversal_pending != 0U) &&
        (*hold_until == 0U))
    {
        *hold_until = now + MOTOR_REVERSAL_HOLD_MS;
    }

    if ((current == 0.0f) &&
        (*hold_until != 0U) &&
        ((int32_t)(now - *hold_until) < 0))
    {
        return 0.0f;
    }

    if ((current == 0.0f) &&
        (*hold_until != 0U) &&
        ((int32_t)(now - *hold_until) >= 0))
    {
        *hold_until = 0U;
        *reversal_pending = 0U;
    }

    return Motor_Ramp_Value(current, target);
}

static void Motor_Update(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_motor_update_tick) <
        MOTOR_UPDATE_PERIOD_MS)
    {
        return;
    }

    last_motor_update_tick = now;
    applied_left_velocity =
        Motor_Update_Wheel(applied_left_velocity,
                           target_left_velocity,
                           now,
                           &left_reversal_hold_until,
                           &left_reversal_pending);
    applied_right_velocity =
        Motor_Update_Wheel(applied_right_velocity,
                           target_right_velocity,
                           now,
                           &right_reversal_hold_until,
                           &right_reversal_pending);

    Motor_Set_Left(applied_left_velocity);
    Motor_Set_Right(applied_right_velocity);
}

static void Motor_DeadTime(void)
{
    Delay_us(MOTOR_DIRECTION_DEADTIME_US);
}

/*
 * Fault-context safe state. This intentionally uses only direct peripheral
 * register writes and does not depend on HAL locks, SysTick, or UART.
 */
void Motor_SafeState_FaultContext(void)
{
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM14->CCR1 = 0U;
    TIM2->CCR3 = 0U;

    TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM14->CCER &= ~TIM_CCER_CC1E;
    TIM2->CCER &= ~TIM_CCER_CC3E;

    GPIOA->BSRR = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7) << 16U;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16U;
    GPIOC->BSRR = (uint32_t)GPIO_PIN_7 << 16U;
}

/* ============================================================
 * COMMAND PROCESSOR
 *
 * PING
 * STOP
 * VEL 500 500
 * VEL -500 -500
 * VEL 500 -500
 * ============================================================ */

static void Process_Command(void)
{
    int left_milli;
    int right_milli;
    char extra;

    char response[64];

    /* --------------------------------------------------------
     * PING
     * -------------------------------------------------------- */

    if (strcmp(uart_command_buffer, "PING") == 0)
    {
        UART_Send("PONG\r\n");

        return;
    }

    /* --------------------------------------------------------
     * STOP
     * -------------------------------------------------------- */

    if (strcmp(uart_command_buffer, "STOP") == 0)
    {
        Motor_Stop();

        last_command_tick = HAL_GetTick();

        UART_Send("STOP_OK\r\n");

        return;
    }

    /* --------------------------------------------------------
     * VEL
     * -------------------------------------------------------- */

    if (strncmp(uart_command_buffer, "VEL ", 4) == 0)
    {
        if (sscanf(
                uart_command_buffer + 4,
                "%d %d %c",
                &left_milli,
                &right_milli,
                &extra
            ) == 2)
        {
            /* Convert milli-m/s → m/s */

            target_left_velocity =
                (float)left_milli / 1000.0f;

            target_right_velocity =
                (float)right_milli / 1000.0f;

            /* ------------------------------------------------
             * Clamp left
             * ------------------------------------------------ */

            if (target_left_velocity >
                MAX_WHEEL_SPEED_MPS)
            {
                target_left_velocity =
                    MAX_WHEEL_SPEED_MPS;
            }

            if (target_left_velocity <
                -MAX_WHEEL_SPEED_MPS)
            {
                target_left_velocity =
                    -MAX_WHEEL_SPEED_MPS;
            }

            /* ------------------------------------------------
             * Clamp right
             * ------------------------------------------------ */

            if (target_right_velocity >
                MAX_WHEEL_SPEED_MPS)
            {
                target_right_velocity =
                    MAX_WHEEL_SPEED_MPS;
            }

            if (target_right_velocity <
                -MAX_WHEEL_SPEED_MPS)
            {
                target_right_velocity =
                    -MAX_WHEEL_SPEED_MPS;
            }

            /* Refresh watchdog */

            last_command_tick =
                HAL_GetTick();

            snprintf(
                response,
                sizeof(response),
                "VEL_OK %.3f %.3f\r\n",
                target_left_velocity,
                target_right_velocity
            );

            UART_Send(response);

            return;
        }

        UART_Send("ERR\r\n");

        return;
    }

    UART_Send("ERR\r\n");
}

/* ============================================================
 * DWT INITIALIZATION
 *
 * Used for accurate microsecond timing.
 * ============================================================ */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |=
        CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |=
        DWT_CTRL_CYCCNTENA_Msk;
}

/* ============================================================
 * MICROSECOND DELAY
 * ============================================================ */

static void Delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    start = DWT->CYCCNT;

    ticks = us * DWT_Cycles_Per_Us();

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

static uint32_t DWT_Cycles_Per_Us(void)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();

    return (hclk >= 1000000U) ? (hclk / 1000000U) : 1U;
}

/* ============================================================
 * HC-SR04 READ
 *
 * Returns distance in millimeters.
 *
 * Returns 0 if no echo is received.
 * ============================================================ */

static uint32_t Ultrasonic_Read(
    GPIO_TypeDef *TRIG_PORT,
    uint16_t TRIG_PIN,
    GPIO_TypeDef *ECHO_PORT,
    uint16_t ECHO_PIN)
{
    uint32_t start;
    uint32_t timeout_ticks;
    uint32_t pulse_ticks;
    uint32_t pulse_us;
    uint32_t distance_mm;

    /* --------------------------------------------------------
     * Trigger LOW
     * -------------------------------------------------------- */

    HAL_GPIO_WritePin(
        TRIG_PORT,
        TRIG_PIN,
        GPIO_PIN_RESET
    );

    Delay_us(2);

    /* --------------------------------------------------------
     * Trigger HIGH for 10 us
     * -------------------------------------------------------- */

    HAL_GPIO_WritePin(
        TRIG_PORT,
        TRIG_PIN,
        GPIO_PIN_SET
    );

    Delay_us(10);

    HAL_GPIO_WritePin(
        TRIG_PORT,
        TRIG_PIN,
        GPIO_PIN_RESET
    );

    /* --------------------------------------------------------
     * Wait for ECHO HIGH
     * -------------------------------------------------------- */

    timeout_ticks = ULTRASONIC_TIMEOUT_US * DWT_Cycles_Per_Us();
    start = DWT->CYCCNT;

    while (
        HAL_GPIO_ReadPin(
            ECHO_PORT,
            ECHO_PIN
        ) == GPIO_PIN_RESET
    )
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_ticks)
        {
            return 0;
        }
    }

    /* --------------------------------------------------------
     * Start pulse measurement
     * -------------------------------------------------------- */

    start = DWT->CYCCNT;

    while (
        HAL_GPIO_ReadPin(
            ECHO_PORT,
            ECHO_PIN
        ) == GPIO_PIN_SET
    )
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_ticks)
        {
            return 0;
        }
    }

    pulse_ticks =
        DWT->CYCCNT - start;

    pulse_us =
        pulse_ticks /
        (HAL_RCC_GetHCLKFreq() / 1000000U);

    /* --------------------------------------------------------
     * Distance calculation
     *
     * Speed of sound ≈ 343 m/s
     *
     * distance(mm) =
     *     time(us) × 343 / 2000
     * -------------------------------------------------------- */

    distance_mm =
        (pulse_us * 343U) / 2000U;

    return distance_mm;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
    /* MCU Configuration */

    HAL_Init();

    /* Microsecond timer */

    DWT_Init();

    /* Configure system clock */

    SystemClock_Config();

    /* Initialize peripherals */

    MX_GPIO_Init();

    MX_USART2_UART_Init();

    MX_TIM2_Init();

    MX_TIM3_Init();

    MX_TIM14_Init();

    /* ========================================================
     * Start PWM
     * ======================================================== */

    if (HAL_TIM_PWM_Start(
        &htim3,
        TIM_CHANNEL_1
    ) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim3,
        TIM_CHANNEL_2
    ) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim14,
        TIM_CHANNEL_1
    ) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim2,
        TIM_CHANNEL_3
    ) != HAL_OK) Error_Handler();

    /* Start motors stopped */

    Motor_Stop();

    /* ========================================================
     * Start UART interrupt reception
     * ======================================================== */

    if (HAL_UART_Receive_IT(
        &huart2,
        &uart_rx_byte,
        1
    ) != HAL_OK) Error_Handler();

    /* ========================================================
     * Board peripherals
     * ======================================================== */

    BSP_LED_Init(LED2);

    BSP_PB_Init(
        BUTTON_USER,
        BUTTON_MODE_EXTI
    );

    last_command_tick =
        HAL_GetTick();

    last_ultrasonic_tick =
        HAL_GetTick();

    /* ========================================================
     * Main loop
     * ======================================================== */

    while (1)
    {
        if (uart_command_ready != 0U)
        {
            Process_Command();
            uart_command_ready = 0U;
        }

        UART_Tx_Service();
        Motor_Update();

        /* ----------------------------------------------------
         * MOTOR SAFETY WATCHDOG
         * ---------------------------------------------------- */

        if (
            (HAL_GetTick() - last_command_tick) >
            MOTOR_COMMAND_TIMEOUT_MS
        )
        {
            Motor_Stop();
        }

        /* ----------------------------------------------------
         * ULTRASONIC UPDATE
         *
         * Left sensor first.
         * Small delay prevents acoustic interference.
         * ---------------------------------------------------- */

        if (
            (HAL_GetTick() - last_ultrasonic_tick) >=
            ULTRASONIC_UPDATE_MS
        )
        {
            last_ultrasonic_tick =
                HAL_GetTick();

    /* LEFT */

            left_distance_mm =
                Ultrasonic_Read(
                    GPIOB,
                    GPIO_PIN_0,
                    GPIOB,
                    GPIO_PIN_1
                );

            /* Small separation between sensors */

            HAL_Delay(2);

            /* RIGHT */

            right_distance_mm =
                Ultrasonic_Read(
                    GPIOB,
                    GPIO_PIN_2,
                    GPIOC,
                    GPIO_PIN_0
                );

            /* ------------------------------------------------
             * Send:
             *
             * DIST left_mm right_mm
             * ------------------------------------------------ */

            snprintf(
                distance_msg,
                sizeof(distance_msg),
                "DIST %lu %lu\r\n",
                (unsigned long)left_distance_mm,
                (unsigned long)right_distance_mm
            );

            UART_Send(distance_msg);
        }

        /* LED heartbeat */

        BSP_LED_Toggle(LED2);

        HAL_Delay(10);
    }
}

/* ============================================================
 * SYSTEM CLOCK
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE3
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = 16;

    RCC_OscInitStruct.PLL.PLLN = 336;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV4;

    RCC_OscInitStruct.PLL.PLLQ = 2;

    RCC_OscInitStruct.PLL.PLLR = 2;

    if (
        HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (
        HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}

/* ============================================================
 * TIM2
 *
 * PB10 → TIM2_CH3 → RIGHT LPWM
 *
 * IMPORTANT:
 * Prescaler 0 and period 4199 provide a 20 kHz carrier at the
 * configured 84 MHz timer clock.
 * ============================================================ */

static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;

    htim2.Init.Prescaler = 0;

    htim2.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim2.Init.Period = PWM_PERIOD;

    htim2.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim2.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (
        HAL_TIM_PWM_Init(&htim2) != HAL_OK
    )
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim2,
            &sMasterConfig
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;

    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim2,
            &sConfigOC,
            TIM_CHANNEL_3
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/* ============================================================
 * TIM3
 *
 * PA6 → CH1 → LEFT RPWM
 * PC7 → CH2 → RIGHT RPWM
 * ============================================================ */

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;

    htim3.Init.Prescaler = 0;

    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim3.Init.Period = PWM_PERIOD;

    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (
        HAL_TIM_PWM_Init(&htim3) != HAL_OK
    )
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;

    /* PA6 */

    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            TIM_CHANNEL_1
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    /* PC7 */

    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            TIM_CHANNEL_2
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}

/* ============================================================
 * TIM14
 *
 * PA7 → TIM14_CH1 → LEFT LPWM
 * ============================================================ */

static void MX_TIM14_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim14.Instance = TIM14;

    htim14.Init.Prescaler = 0;

    htim14.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim14.Init.Period = PWM_PERIOD;

    htim14.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim14.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (
        HAL_TIM_Base_Init(&htim14) != HAL_OK
    )
    {
        Error_Handler();
    }

    if (
        HAL_TIM_PWM_Init(&htim14) != HAL_OK
    )
    {
        Error_Handler();
    }

    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;

    sConfigOC.Pulse = 0;

    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;

    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;

    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim14,
            &sConfigOC,
            TIM_CHANNEL_1
        ) != HAL_OK
    )
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim14);
}

/* ============================================================
 * USART2
 * ============================================================ */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (
        HAL_UART_Init(&huart2) != HAL_OK
    )
    {
        Error_Handler();
    }
}

/* ============================================================
 * GPIO
 *
 * PB0 = LEFT TRIG
 * PB1 = LEFT ECHO
 * PB2 = RIGHT TRIG
 * PC0 = RIGHT ECHO
 * ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO clocks */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Initial trigger state LOW */

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0 | GPIO_PIN_2,
        GPIO_PIN_RESET
    );

    /* --------------------------------------------------------
     * PC0
     *
     * RIGHT ECHO
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );

    /* --------------------------------------------------------
     * PB0 + PB2
     *
     * TRIG outputs
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 | GPIO_PIN_2;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );

    /* --------------------------------------------------------
     * PB1
     *
     * LEFT ECHO
     * -------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );
}

/* ============================================================
 * UART RX CALLBACK
 * ============================================================ */

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (
            (uart_rx_byte == '\r') ||
            (uart_rx_byte == '\n')
        )
        {
            if ((uart_rx_index > 0U) &&
                (uart_rx_discard == 0U) &&
                (uart_command_ready == 0U))
            {
                uart_rx_buffer[uart_rx_index] =
                    '\0';

                memcpy(uart_command_buffer,
                       uart_rx_buffer,
                       uart_rx_index + 1U);

                uart_command_ready = 1U;
            }

            if ((uart_rx_index > 0U) ||
                (uart_rx_discard != 0U))
            {
                uart_rx_index = 0U;
                uart_rx_discard = 0U;
            }
        }
        else
        {
            if ((uart_rx_discard == 0U) &&
                uart_rx_index <
                UART_RX_BUFFER_SIZE - 1U
            )
            {
                uart_rx_buffer[uart_rx_index++] =
                    (char)uart_rx_byte;
            }
            else
            {
                uart_rx_discard = 1U;
            }
        }

        if (HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1
        ) != HAL_OK)
        {
            Motor_SafeState_FaultContext();
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_tail = (uint8_t)((uart_tx_tail + 1U) %
                                 UART_TX_QUEUE_DEPTH);
        uart_tx_busy = 0U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart_tx_busy = 0U;
    }
}

/* ============================================================
 * ERROR HANDLER
 * ============================================================ */

void Error_Handler(void)
{
    __disable_irq();
    Motor_SafeState_FaultContext();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
}

#endif
