/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  I2C_HandleTypeDef *handle;
  const char *name;
  uint32_t consecutive_failures;
  uint32_t digital_filter;
} I2C_FaultContext;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_I2C_ADDR (0x36u << 1) /* HAL expects 8-bit address */
#define AS5600_ANGLE_START 0x0Eu
#define AS5600_STEPS 4096.0f
#define CAN_ID_ENCODER1_STATUS 0x180U
#define CAN_ID_ENCODER2_STATUS 0x182U
#define CAN_ID_LIMIT_SWITCH 0x181U
#define ENC_FLAG_INDEX_SEEN 0x01u
#define ENC_FLAG_ERROR_LATCHED 0x02u
#define ENC_FLAG_SENSOR_FAULT 0x04u
#define ENC_FLAG_OVER_SPEED 0x08u
#define CAN_ERROR_FLAG_BUS_OFF (1UL << 9)
#define I2C_FAULT_RECOVERY_THRESHOLD 2u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint32_t angle_check_pending = 0UL;
static FDCAN_TxHeaderTypeDef encoder_status_headers[2];
static FDCAN_TxHeaderTypeDef limit_switch_header;
static uint8_t encoder_status_flags[2] = {0u, 0u};
static volatile uint8_t limit_switch_pending = 0u;
static uint8_t limit_switch_frame[8] = {0};
static uint8_t limit_switch_last_state = 0u;
static uint8_t limit_switch_initialized = 0u;
static uint32_t limit_switch_next_periodic_tick = 0u;
static I2C_FaultContext i2c_fault_contexts[] = {
    {&hi2c1, "I2C1", 0u, 0u},
    {&hi2c2, "I2C2", 0u, 0u},
};
static int32_t encoder_position_q24[2] = {0};
static volatile uint8_t fdcan_bus_off_pending = 0u;
static volatile uint32_t fdcan_bus_off_request_tick = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ICACHE_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void AngleCheck_Run(void);
static void CAN_InitTxHeaders(void);
static HAL_StatusTypeDef CAN_SendFrame(FDCAN_TxHeaderTypeDef *header,
                                       uint8_t data[8]);
static void CAN_SendEncoderStatus(uint8_t index);
static int32_t ClampToS24(int32_t value);
static void PackS24BE(uint8_t *dest, int32_t value);
static I2C_FaultContext *I2C_GetContext(I2C_HandleTypeDef *handle);
static HAL_StatusTypeDef I2C_RecoverBus(I2C_FaultContext *ctx);
static HAL_StatusTypeDef I2C_HandleFault(I2C_FaultContext *ctx);
static HAL_StatusTypeDef AS5600_ReadAngleWithFallback(I2C_HandleTypeDef *hi2c,
                                                      I2C_FaultContext *ctx,
                                                      uint16_t *angle12);
static void LimitSwitch_Poll(void);
static void LimitSwitch_RecordEvent(uint8_t state, uint8_t is_change);
static void LimitSwitch_Service(void);
static void CAN_ScheduleBusOffRecovery(void);
static void CAN_ServiceBusOff(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);

static HAL_StatusTypeDef AS5600_ReadAngle12(I2C_HandleTypeDef *hi2c,
                                            uint16_t *angle12) {
  uint8_t buf[2] = {0};
  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_ANGLE_START,
                       I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 10);
  if (st != HAL_OK) {
    return st;
  }

  *angle12 = (uint16_t)(((buf[0] & 0x0Fu) << 8) | buf[1]);
  return HAL_OK;
}

static void UART_SendLine(const char *s) {
  size_t len = strlen(s);
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)len, 50);
}

static void print_i2c_error(const char *tag, I2C_HandleTypeDef *hi2c) {
  uint32_t err = HAL_I2C_GetError(hi2c);
  char msg[96];
  snprintf(msg, sizeof(msg), "%s ERR=0x%08lX%s%s%s%s%s%s\r\n", tag,
           (unsigned long)err, (err & HAL_I2C_ERROR_BERR) ? " BERR" : "",
           (err & HAL_I2C_ERROR_ARLO) ? " ARLO" : "",
           (err & HAL_I2C_ERROR_AF) ? " AF(NACK)" : "",
           (err & HAL_I2C_ERROR_OVR) ? " OVR" : "",
           (err & HAL_I2C_ERROR_DMA) ? " DMA" : "",
           (err & HAL_I2C_ERROR_TIMEOUT) ? " TIMEOUT" : "");
  UART_SendLine(msg);
}

static HAL_StatusTypeDef probe_as5600(I2C_HandleTypeDef *hi2c,
                                      const char *tag) {
  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, 10);
  char line[64];
  if (st != HAL_OK) {
    snprintf(line, sizeof(line), "%s: NACK @0x36\r\n", tag);
    UART_SendLine(line);
    print_i2c_error(tag, hi2c);
  } else {
    snprintf(line, sizeof(line), "%s: OK\r\n", tag);
    UART_SendLine(line);
  }
  return st;
}

static int32_t ClampToS24(int32_t value) {
  if (value > 0x7FFFFF) {
    return 0x7FFFFF;
  }
  if (value < -0x800000) {
    return -0x800000;
  }
  return value;
}

static void PackS24BE(uint8_t *dest, int32_t value) {
  const int32_t clamped = ClampToS24(value);
  dest[0] = (uint8_t)((uint32_t)clamped >> 16);
  dest[1] = (uint8_t)((uint32_t)clamped >> 8);
  dest[2] = (uint8_t)((uint32_t)clamped);
}

static I2C_FaultContext *I2C_GetContext(I2C_HandleTypeDef *handle) {
  if (handle == NULL) {
    return NULL;
  }
  for (size_t i = 0;
       i < (sizeof(i2c_fault_contexts) / sizeof(i2c_fault_contexts[0])); ++i) {
    if (i2c_fault_contexts[i].handle == handle) {
      return &i2c_fault_contexts[i];
    }
  }
  return NULL;
}

static HAL_StatusTypeDef I2C_RecoverBus(I2C_FaultContext *ctx) {
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  if (HAL_I2C_DeInit(ctx->handle) != HAL_OK) {
    char line[96];
    snprintf(line, sizeof(line), "%s: I2C deinit failed\r\n", ctx->name);
    UART_SendLine(line);
    return HAL_ERROR;
  }

  if (HAL_I2C_Init(ctx->handle) != HAL_OK) {
    char line[96];
    snprintf(line, sizeof(line), "%s: I2C init failed\r\n", ctx->name);
    UART_SendLine(line);
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(ctx->handle, I2C_ANALOGFILTER_ENABLE) !=
      HAL_OK) {
    char line[96];
    snprintf(line, sizeof(line), "%s: analog filter cfg failed\r\n", ctx->name);
    UART_SendLine(line);
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(ctx->handle, ctx->digital_filter) !=
      HAL_OK) {
    char line[96];
    snprintf(line, sizeof(line), "%s: digital filter cfg failed\r\n",
             ctx->name);
    UART_SendLine(line);
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef I2C_HandleFault(I2C_FaultContext *ctx) {
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  if (ctx->consecutive_failures < UINT32_MAX) {
    ctx->consecutive_failures++;
  }

  char line[96];
  snprintf(line, sizeof(line), "%s fault count=%lu\r\n", ctx->name,
           (unsigned long)ctx->consecutive_failures);
  UART_SendLine(line);

  if (ctx->consecutive_failures < I2C_FAULT_RECOVERY_THRESHOLD) {
    return HAL_ERROR;
  }

  snprintf(line, sizeof(line), "%s attempting bus recovery\r\n", ctx->name);
  UART_SendLine(line);

  HAL_StatusTypeDef status = I2C_RecoverBus(ctx);
  if (status == HAL_OK) {
    snprintf(line, sizeof(line), "%s recovery succeeded\r\n", ctx->name);
    UART_SendLine(line);
    ctx->consecutive_failures = 0u;
  } else {
    snprintf(line, sizeof(line), "%s recovery failed\r\n", ctx->name);
    UART_SendLine(line);
    if (ctx->consecutive_failures > I2C_FAULT_RECOVERY_THRESHOLD) {
      ctx->consecutive_failures = I2C_FAULT_RECOVERY_THRESHOLD;
    }
  }
  return status;
}

static HAL_StatusTypeDef AS5600_ReadAngleWithFallback(I2C_HandleTypeDef *hi2c,
                                                      I2C_FaultContext *ctx,
                                                      uint16_t *angle12) {
  HAL_StatusTypeDef status = AS5600_ReadAngle12(hi2c, angle12);
  if (ctx == NULL) {
    return status;
  }

  if (status == HAL_OK) {
    ctx->consecutive_failures = 0u;
    return HAL_OK;
  }

  print_i2c_error(ctx->name, hi2c);
  (void)I2C_HandleFault(ctx);

  status = AS5600_ReadAngle12(hi2c, angle12);
  if (status == HAL_OK) {
    ctx->consecutive_failures = 0u;
    return HAL_OK;
  }

  print_i2c_error(ctx->name, hi2c);
  if (ctx->consecutive_failures == 0u) {
    ctx->consecutive_failures = 1u;
  }
  return status;
}

static void LimitSwitch_RecordEvent(uint8_t state, uint8_t is_change) {
  limit_switch_frame[0] = state;
  limit_switch_frame[1] = 0u;
  limit_switch_frame[2] = 0u;
  limit_switch_frame[3] = 0u;
  limit_switch_frame[4] = 0u;
  limit_switch_frame[5] = 0u;
  limit_switch_frame[6] = 0u;
  limit_switch_frame[7] = 0u;
  limit_switch_pending = 1u;

  if (is_change != 0u) {
    const char *edge_str = (state != 0u) ? "rising" : "falling";
    char line[64];
    snprintf(line, sizeof(line), "Limit switch: state=%u edge=%s\r\n",
             (unsigned int)state, edge_str);
    UART_SendLine(line);
  }
}

static void LimitSwitch_Poll(void) {
  uint32_t now = HAL_GetTick();
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
  uint8_t state = (pin_state == GPIO_PIN_SET) ? 1u : 0u;

  if (limit_switch_initialized == 0u) {
    limit_switch_last_state = state;
    limit_switch_initialized = 1u;
    limit_switch_next_periodic_tick = now;
    LimitSwitch_RecordEvent(state, 1u);
    return;
  }

  if (state != limit_switch_last_state) {
    limit_switch_last_state = state;
    limit_switch_next_periodic_tick = now + 200u;
    LimitSwitch_RecordEvent(state, 1u);
    return;
  }

  if ((int32_t)(now - limit_switch_next_periodic_tick) >= 0) {
    limit_switch_next_periodic_tick = now + 200u;
    if (limit_switch_pending == 0u) {
      LimitSwitch_RecordEvent(state, 0u);
    }
  }
}

static void CAN_InitTxHeaders(void) {
  FDCAN_TxHeaderTypeDef base = {0};
  base.IdType = FDCAN_STANDARD_ID;
  base.TxFrameType = FDCAN_DATA_FRAME;
  base.DataLength = FDCAN_DLC_BYTES_8;
  base.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  base.BitRateSwitch = FDCAN_BRS_OFF;
  base.FDFormat = FDCAN_CLASSIC_CAN;
  base.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  base.MessageMarker = 0;

  encoder_status_headers[0] = base;
  encoder_status_headers[1] = base;
  encoder_status_headers[0].Identifier = CAN_ID_ENCODER1_STATUS;
  encoder_status_headers[1].Identifier = CAN_ID_ENCODER2_STATUS;

  limit_switch_header = base;
  limit_switch_header.Identifier = CAN_ID_LIMIT_SWITCH;
}

static HAL_StatusTypeDef CAN_SendFrame(FDCAN_TxHeaderTypeDef *header,
                                       uint8_t data[8]) {
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
    char line[64];
    snprintf(line, sizeof(line), "CAN TX busy (fifo full), id=0x%03lX\r\n",
             (unsigned long)header->Identifier);
    UART_SendLine(line);
    FDCAN_ProtocolStatusTypeDef proto = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto);
    uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
    snprintf(line, sizeof(line),
             "CAN diag: act=%u lec=%u bo=%u err=0x%08lX\r\n",
             (unsigned int)proto.Activity, (unsigned int)proto.LastErrorCode,
             (unsigned int)proto.BusOff, (unsigned long)err);
    UART_SendLine(line);
    if ((proto.BusOff != 0U) || ((err & CAN_ERROR_FLAG_BUS_OFF) != 0U)) {
      CAN_ScheduleBusOffRecovery();
    }
    return HAL_BUSY;
  }

  HAL_StatusTypeDef status =
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, data);
  if (status != HAL_OK) {
    char line[96];
    snprintf(line, sizeof(line), "CAN TX failed status=%ld id=0x%03lX\r\n",
             (long)status, (unsigned long)header->Identifier);
    UART_SendLine(line);
    FDCAN_ProtocolStatusTypeDef proto = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto);
    uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
    snprintf(line, sizeof(line),
             "CAN diag: act=%u lec=%u bo=%u err=0x%08lX\r\n",
             (unsigned int)proto.Activity, (unsigned int)proto.LastErrorCode,
             (unsigned int)proto.BusOff, (unsigned long)err);
    UART_SendLine(line);
    if ((proto.BusOff != 0U) || ((err & CAN_ERROR_FLAG_BUS_OFF) != 0U)) {
      CAN_ScheduleBusOffRecovery();
    }
  }
  return status;
}

static void CAN_SendEncoderStatus(uint8_t index) {
  if (index >= 2U) {
    return;
  }
  uint8_t payload[8] = {0};
  PackS24BE(&payload[0], encoder_position_q24[index]);
  payload[3] = encoder_status_flags[index];
  (void)CAN_SendFrame(&encoder_status_headers[index], payload);
}

static void LimitSwitch_Service(void) {
  if (limit_switch_pending == 0U) {
    return;
  }

  uint8_t payload[8];
  uint8_t have_frame = 0U;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (limit_switch_pending != 0U) {
    memcpy(payload, limit_switch_frame, sizeof(payload));
    limit_switch_pending = 0U;
    have_frame = 1U;
  }
  if (primask == 0U) {
    __enable_irq();
  }

  if (have_frame == 0U) {
    return;
  }

  HAL_StatusTypeDef tx_status = CAN_SendFrame(&limit_switch_header, payload);
  if (tx_status == HAL_OK) {
    return;
  }

  uint32_t primask2 = __get_PRIMASK();
  __disable_irq();
  memcpy(limit_switch_frame, payload, sizeof(limit_switch_frame));
  limit_switch_pending = 1U;
  if (primask2 == 0U) {
    __enable_irq();
  }
}

static void CAN_ScheduleBusOffRecovery(void) {
  if (fdcan_bus_off_pending == 0u) {
    fdcan_bus_off_pending = 1u;
    fdcan_bus_off_request_tick = HAL_GetTick();
  }
}

static void CAN_ServiceBusOff(void) {
  if (fdcan_bus_off_pending == 0u) {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - fdcan_bus_off_request_tick) < 200u) {
    return;
  }

  UART_SendLine("CAN bus-off recovery start\r\n");

  if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
    UART_SendLine("CAN stop failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK) {
    UART_SendLine("CAN deinit failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  MX_FDCAN1_Init();
  CAN_InitTxHeaders();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    UART_SendLine("CAN restart failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0U) !=
      HAL_OK) {
    UART_SendLine("CAN notification re-arm failed\r\n");
  }

  UART_SendLine("CAN bus-off recovery complete\r\n");
  fdcan_bus_off_pending = 0u;
  CAN_SendEncoderStatus(0u);
  CAN_SendEncoderStatus(1u);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  initialise_monitor_handles(); // Enable semihosting
  printf("Hello via semihosting!\n");

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_InitTxHeaders();
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0U) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }
  AngleCheck_Run();
  CAN_ServiceBusOff();
  LimitSwitch_Poll();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LimitSwitch_Service();
    CAN_ServiceBusOff();

    if (angle_check_pending > 0UL) {
      angle_check_pending--;
      AngleCheck_Run();
      CAN_ServiceBusOff();
      continue;
    }

    if (limit_switch_pending != 0U) {
      continue;
    }

    __WFI();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 43;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60808CD3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60808CD3;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void) {

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
   */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2499;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 SW1_NC_Pin SW1_NO_Pin SW2_NC_Pin
                           SW2_NO_Pin */
  GPIO_InitStruct.Pin =
      GPIO_PIN_1 | SW1_NC_Pin | SW1_NO_Pin | SW2_NC_Pin | SW2_NO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void AngleCheck_Run(void) {
  probe_as5600(&hi2c1, "I2C1");
  probe_as5600(&hi2c2, "I2C2");

  uint16_t angle1 = 0;
  uint16_t angle2 = 0;
  I2C_FaultContext *i2c1_ctx = I2C_GetContext(&hi2c1);
  I2C_FaultContext *i2c2_ctx = I2C_GetContext(&hi2c2);
  HAL_StatusTypeDef s1 =
      AS5600_ReadAngleWithFallback(&hi2c1, i2c1_ctx, &angle1);
  HAL_StatusTypeDef s2 =
      AS5600_ReadAngleWithFallback(&hi2c2, i2c2_ctx, &angle2);

  if (s1 != HAL_OK) {
    encoder_status_flags[0] |= (ENC_FLAG_SENSOR_FAULT | ENC_FLAG_ERROR_LATCHED);
  }
  if (s2 != HAL_OK) {
    encoder_status_flags[1] |= (ENC_FLAG_SENSOR_FAULT | ENC_FLAG_ERROR_LATCHED);
  }

  if (s1 == HAL_OK) {
    encoder_status_flags[0] |= ENC_FLAG_INDEX_SEEN;
    encoder_status_flags[0] &= (uint8_t)~ENC_FLAG_SENSOR_FAULT;
    int32_t centered = ((int32_t)angle1 - 2048) << 12;
    encoder_position_q24[0] = ClampToS24(centered);
  }
  if (s2 == HAL_OK) {
    encoder_status_flags[1] |= ENC_FLAG_INDEX_SEEN;
    encoder_status_flags[1] &= (uint8_t)~ENC_FLAG_SENSOR_FAULT;
    int32_t centered = ((int32_t)angle2 - 2048) << 12;
    encoder_position_q24[1] = ClampToS24(centered);
  }

  const float deg1 = (angle1 * 360.0f) / AS5600_STEPS;
  const float deg2 = (angle2 * 360.0f) / AS5600_STEPS;

  char line[96];
  if ((s1 == HAL_OK) && (s2 == HAL_OK)) {
    snprintf(line, sizeof(line), "A1=%u (%.2fdeg), A2=%u (%.2fdeg)\r\n", angle1,
             deg1, angle2, deg2);
  } else {
    snprintf(line, sizeof(line), "ERR A1=%u (%.2fdeg), A2=%u (%.2fdeg)\r\n",
             angle1, deg1, angle2, deg2);
  }

  UART_SendLine(line);

  CAN_SendEncoderStatus(0U);
  CAN_SendEncoderStatus(1U);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    if (angle_check_pending < UINT32_MAX) {
      angle_check_pending++;
    }
  } else if (htim->Instance == TIM2) {
    LimitSwitch_Poll();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) { (void)GPIO_Pin; }

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if (hfdcan == &hfdcan1) {
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
      UART_SendLine("CAN bus-off interrupt\r\n");
      CAN_ScheduleBusOffRecovery();
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
