/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 5ch PID line follower: 90-degree turns, junctions, dashed line
  *                   STM32F103 (Blue Pill) + TB6612FNG, HSI/2*16 = 64 MHz
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include <stdlib.h>

/* USER CODE BEGIN PD */
/* ---------- Sensors ---------- */
#define BLACK_LINE          GPIO_PIN_SET    /* GPIO_PIN_RESET if sensor outputs 0 on black */
#define REVERSE_SENSORS     0               /* 1 if PA1 is physically the RIGHT-most sensor */

/* ---------- Motors ---------- */
#define MOTOR_A_INVERT      0               /* A = left  */
#define MOTOR_B_INVERT      0               /* B = right */
#define PWM_ARR             3199            /* 64MHz / 3200 = 20 kHz */

/* ---------- Speeds (0..1000) ---------- */
#define BASE_SPEED          400
#define MAX_SPEED           800
#define MIN_SPEED          -250
#define TURN_SPEED          380
#define SEARCH_SPEED        350

/* ---------- PID ---------- */
#define KP                  130.0f
#define KI                  0.0f
#define KD                  90.0f
#define I_LIMIT             300.0f
#define LOOP_MS             5

/* ---------- Timing / behaviour ---------- */
#define START_DELAY_MS      3000
#define ADVANCE_MS          150     /* forward move so wheel axle reaches the corner (tune!) */
#define COOLDOWN_MS         250     /* ignore corners/junctions right after one */
#define LOST_STRAIGHT_MS    250     /* dashed gap: keep straight this long */
#define STOP_LOST_MS        1500    /* line lost this long -> stop (end of track) */
#define TURN_TIMEOUT_MS     2000
#define CORNER_CONFIRM      2       /* consecutive loops to confirm a pattern */
/* USER CODE END PD */

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static float    last_error = 0.0f;
static float    prev_error = 0.0f;
static float    integral   = 0.0f;
static uint32_t lost_since = 0;
static uint8_t  is_lost    = 0;
static uint32_t cooldown_until = 0;
volatile uint8_t dbg_mask = 0;      /* watch in Live Expressions: bit0=PA1 ... bit4=PA5 */
volatile float   dbg_err  = 0.0f;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN 0 */
static const float weights[5] = { -2.0f, -1.0f, 0.0f, 1.0f, 2.0f };

/* returns bit i = 1 when sensor i (PA1..PA5, left->right) sees black */
static uint8_t Read_Sensors(void)
{
    uint32_t idr = GPIOA->IDR;
    uint8_t m = 0;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t level = (idr >> (i + 1)) & 1u;
        uint8_t on = (BLACK_LINE == GPIO_PIN_SET) ? level : (uint8_t)!level;
#if REVERSE_SENSORS
        m |= (uint8_t)(on << (4 - i));
#else
        m |= (uint8_t)(on << i);
#endif
    }
    return m;
}

static uint8_t Count_Bits(uint8_t m)
{
    uint8_t c = 0;
    while (m) { c += (m & 1u); m >>= 1; }
    return c;
}

static void Motor_Write(int16_t speed, GPIO_TypeDef *p1, uint16_t pin1,
                        GPIO_TypeDef *p2, uint16_t pin2, uint32_t channel, uint8_t invert)
{
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    uint8_t fwd = (speed >= 0);
    uint32_t mag = (uint32_t)(fwd ? speed : -speed);
    uint32_t duty = mag * (PWM_ARR + 1u) / 1000u;
    if (duty > PWM_ARR) duty = PWM_ARR;

    uint8_t d = fwd ^ invert;
    if (mag == 0) {
        HAL_GPIO_WritePin(p1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p2, pin2, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(p1, pin1, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p2, pin2, d ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim2, channel, duty);
}

/* A = left (PA6/PA7, CH1), B = right (PB0/PB1, CH2) */
void Set_Motors(int16_t left, int16_t right)
{
    Motor_Write(left,  GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7, TIM_CHANNEL_1, MOTOR_A_INVERT);
    Motor_Write(right, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, TIM_CHANNEL_2, MOTOR_B_INVERT);
}

void Stop_Motors(void)
{
    Set_Motors(0, 0);
}

static void Reset_PID(void)
{
    last_error = 0.0f;
    prev_error = 0.0f;
    integral   = 0.0f;
    is_lost    = 0;
}

/* dir: +1 = right, -1 = left. Spin in place until center sensor re-finds the line */
static void Turn_90(int8_t dir)
{
    int16_t l = (dir > 0) ?  TURN_SPEED : -TURN_SPEED;
    int16_t r = (dir > 0) ? -TURN_SPEED :  TURN_SPEED;
    uint32_t t;

    /* phase 1: make sure center sensor left the old line */
    t = HAL_GetTick();
    while ((Read_Sensors() & 0x04) && (HAL_GetTick() - t) < 600) {
        Set_Motors(l, r);
    }
    /* phase 2: spin until center sensor sees the new line */
    t = HAL_GetTick();
    while (!(Read_Sensors() & 0x04) && (HAL_GetTick() - t) < TURN_TIMEOUT_MS) {
        Set_Motors(l, r);
    }
    /* short opposite pulse to kill inertia */
    Set_Motors(-l / 2, -r / 2);
    HAL_Delay(15);
    Stop_Motors();
    HAL_Delay(40);

    Reset_PID();
    cooldown_until = HAL_GetTick() + COOLDOWN_MS;
}

/*
 * Called when a corner/junction pattern was confirmed.
 * kind: +1 right pattern, -1 left pattern, 0 = both sides (T / cross)
 * Drive forward, then check if the line continues straight.
 */
static void Handle_Corner(int8_t kind)
{
    Set_Motors(BASE_SPEED, BASE_SPEED);
    HAL_Delay(ADVANCE_MS);
    Stop_Motors();
    HAL_Delay(30);

    uint8_t m = Read_Sensors();
    uint8_t straight = (m & 0x0E) != 0;   /* s2,s3,s4 see line ahead */

    if (straight) {
        /* junction with continuation: just keep going */
        Reset_PID();
        cooldown_until = HAL_GetTick() + COOLDOWN_MS;
        return;
    }

    if (kind == 0) {
        /* T dead-end (e.g. unload box wall): destination reached -> stop */
        Stop_Motors();
        while (1) { }
    }
    Turn_90(kind);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* PWMA -> PA15 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* PWMB -> PB3  */
  Stop_Motors();

  HAL_Delay(START_DELAY_MS);

  uint8_t  confirm_cnt = 0;
  int8_t   confirm_kind = 99;
  uint32_t next_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if ((int32_t)(HAL_GetTick() - next_tick) < 0) continue;
      next_tick += LOOP_MS;

      uint8_t m = Read_Sensors();
      dbg_mask = m;
      uint8_t s1 = (m >> 0) & 1, s2 = (m >> 1) & 1, s3 = (m >> 2) & 1;
      uint8_t s4 = (m >> 3) & 1, s5 = (m >> 4) & 1;
      uint8_t count = Count_Bits(m);
      uint32_t now = HAL_GetTick();

      /* ---------- 1) Corner / junction detection ---------- */
      int8_t kind = 99;
      if (now >= cooldown_until && count >= 3) {
          if (s1 && s5)                     kind = 0;    /* T / cross */
          else if (s3 && s4 && s5 && !s1)   kind = 1;    /* right 90  */
          else if (s1 && s2 && s3 && !s5)   kind = -1;   /* left 90   */
      }
      if (kind != 99) {
          if (kind == confirm_kind) confirm_cnt++; else { confirm_kind = kind; confirm_cnt = 1; }
          if (confirm_cnt >= CORNER_CONFIRM) {
              confirm_cnt = 0; confirm_kind = 99;
              Handle_Corner(kind);
              next_tick = HAL_GetTick();
          }
          continue;
      } else {
          confirm_cnt = 0; confirm_kind = 99;
      }

      /* ---------- 2) Normal PID line following ---------- */
      if (count > 0)
      {
          is_lost = 0;

          float sum = 0.0f;
          for (uint8_t i = 0; i < 5; i++) if (m & (1u << i)) sum += weights[i];
          float error = sum / (float)count;
          dbg_err = error;

          integral += error;
          if (integral >  I_LIMIT) integral =  I_LIMIT;
          if (integral < -I_LIMIT) integral = -I_LIMIT;

          float derivative = error - prev_error;
          prev_error = error;
          last_error = error;

          float corr = KP * error + KI * integral + KD * derivative;

          float left  = (float)BASE_SPEED + corr;
          float right = (float)BASE_SPEED - corr;
          if (left  > MAX_SPEED) left  = MAX_SPEED;
          if (left  < MIN_SPEED) left  = MIN_SPEED;
          if (right > MAX_SPEED) right = MAX_SPEED;
          if (right < MIN_SPEED) right = MIN_SPEED;

          Set_Motors((int16_t)left, (int16_t)right);
      }
      /* ---------- 3) Line lost: dashed gap / search / end ---------- */
      else
      {
          if (!is_lost) { is_lost = 1; lost_since = now; }
          uint32_t lost_ms = now - lost_since;

          if (lost_ms < LOST_STRAIGHT_MS) {
              /* dashed-line gap: go straight, no correction */
              Set_Motors(BASE_SPEED, BASE_SPEED);
          }
          else if (lost_ms < STOP_LOST_MS) {
              /* really lost: search toward the last seen side */
              if (last_error > 0.3f)       Set_Motors(SEARCH_SPEED, -SEARCH_SPEED);
              else if (last_error < -0.3f) Set_Motors(-SEARCH_SPEED, SEARCH_SPEED);
              else                         Set_Motors(BASE_SPEED, BASE_SPEED);
          }
          else {
              /* end of track */
              Stop_Motors();
              while (1) { }
          }
      }
  /* USER CODE END WHILE */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_ARR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* Free PA15 / PB3 from JTAG (keep SWD) and fully remap TIM2:
     CH1 -> PA15, CH2 -> PB3 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_AFIO_REMAP_TIM2_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /* IR array PA1..PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* AIN1/AIN2 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* BIN1/BIN2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PWM pins: PA15 (TIM2_CH1), PB3 (TIM2_CH2) */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
