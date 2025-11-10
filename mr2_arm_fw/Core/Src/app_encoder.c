#include "app_encoder.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "app_can.h"

#define AS5600_I2C_ADDR (0x36u << 1)
#define AS5600_ANGLE_START 0x0Eu
#define AS5600_STEPS 4096.0f
#define CAN_ID_ENCODER1_STATUS 0x180U
#define CAN_ID_ENCODER2_STATUS 0x182U
#define ENC_FLAG_INDEX_SEEN 0x01u
#define ENC_FLAG_ERROR_LATCHED 0x02u
#define ENC_FLAG_SENSOR_FAULT 0x04u
#define ENC_FLAG_OVER_SPEED 0x08u
#define I2C_FAULT_RECOVERY_THRESHOLD 2u

typedef struct {
  I2C_HandleTypeDef *handle;
  const char *name;
  uint32_t consecutive_failures;
} I2C_FaultContext;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static FDCAN_TxHeaderTypeDef encoder_headers[2];
static uint8_t encoder_status_flags[2] = {0u, 0u};
static int32_t encoder_position_q24[2] = {0};
static volatile uint32_t angle_check_pending = 0UL;
static I2C_FaultContext i2c_fault_contexts[] = {
    {&hi2c1, "I2C1", 0u},
    {&hi2c2, "I2C2", 0u},
};

static int32_t ClampToS24(int32_t value);
static void PackS24BE(uint8_t *dest, int32_t value);
static HAL_StatusTypeDef AS5600_ReadAngle12(I2C_HandleTypeDef *hi2c,
                                            uint16_t *angle12);
static void print_i2c_error(const char *tag, I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef probe_as5600(I2C_HandleTypeDef *hi2c,
                                      const char *tag);
static I2C_FaultContext *I2C_GetContext(I2C_HandleTypeDef *handle);
static HAL_StatusTypeDef I2C_RecoverBus(I2C_FaultContext *ctx);
static HAL_StatusTypeDef I2C_HandleFault(I2C_FaultContext *ctx);
static HAL_StatusTypeDef AS5600_ReadAngleWithFallback(I2C_HandleTypeDef *hi2c,
                                                      I2C_FaultContext *ctx,
                                                      uint16_t *angle12);
static void AppEncoder_RunSample(void);
static void AppEncoder_SendStatus(uint8_t index);
static void AppEncoder_OnBusReset(void);

void AppEncoder_Init(void) {
  angle_check_pending = 0UL;
  for (size_t i = 0;
       i < (sizeof(i2c_fault_contexts) / sizeof(i2c_fault_contexts[0])); ++i) {
    i2c_fault_contexts[i].consecutive_failures = 0u;
  }
  encoder_status_flags[0] = 0u;
  encoder_status_flags[1] = 0u;
  encoder_position_q24[0] = 0;
  encoder_position_q24[1] = 0;

  AppCAN_InitHeader(&encoder_headers[0], CAN_ID_ENCODER1_STATUS,
                    FDCAN_DLC_BYTES_4);
  AppCAN_InitHeader(&encoder_headers[1], CAN_ID_ENCODER2_STATUS,
                    FDCAN_DLC_BYTES_4);

  AppCAN_RegisterBusResetCallback(AppEncoder_OnBusReset);
}

void AppEncoder_RequestMeasurement(void) {
  if (angle_check_pending < UINT32_MAX) {
    angle_check_pending++;
  }
}

void AppEncoder_OnSchedulerTick(void) { AppEncoder_RequestMeasurement(); }

bool AppEncoder_Service(void) {
  if (angle_check_pending == 0UL) {
    return false;
  }
  angle_check_pending--;
  AppEncoder_RunSample();
  return true;
}

static void AppEncoder_OnBusReset(void) {
  AppCAN_InitHeader(&encoder_headers[0], CAN_ID_ENCODER1_STATUS,
                    FDCAN_DLC_BYTES_4);
  AppCAN_InitHeader(&encoder_headers[1], CAN_ID_ENCODER2_STATUS,
                    FDCAN_DLC_BYTES_4);
  AppEncoder_SendStatus(0U);
  AppEncoder_SendStatus(1U);
}

static HAL_StatusTypeDef AS5600_ReadAngle12(I2C_HandleTypeDef *hi2c,
                                            uint16_t *angle12) {
  if ((hi2c == NULL) || (angle12 == NULL)) {
    return HAL_ERROR;
  }

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

static void print_i2c_error(const char *tag, I2C_HandleTypeDef *hi2c) {
  uint32_t err = HAL_I2C_GetError(hi2c);
  printf("%s ERR=0x%08lX%s%s%s%s%s%s\r\n", tag, (unsigned long)err,
         (err & HAL_I2C_ERROR_BERR) ? " BERR" : "",
         (err & HAL_I2C_ERROR_ARLO) ? " ARLO" : "",
         (err & HAL_I2C_ERROR_AF) ? " AF(NACK)" : "",
         (err & HAL_I2C_ERROR_OVR) ? " OVR" : "",
         (err & HAL_I2C_ERROR_DMA) ? " DMA" : "",
         (err & HAL_I2C_ERROR_TIMEOUT) ? " TIMEOUT" : "");
}

static HAL_StatusTypeDef probe_as5600(I2C_HandleTypeDef *hi2c,
                                      const char *tag) {
  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, 10);
  if (st != HAL_OK) {
    printf("%s: NACK @0x36\r\n", tag);
    print_i2c_error(tag, hi2c);
  } else {
    printf("%s: OK\r\n", tag);
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
    printf("%s: I2C deinit failed\r\n", ctx->name);
    return HAL_ERROR;
  }

  if (HAL_I2C_Init(ctx->handle) != HAL_OK) {
    printf("%s: I2C init failed\r\n", ctx->name);
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(ctx->handle, I2C_ANALOGFILTER_ENABLE) !=
      HAL_OK) {
    printf("%s: analog filter cfg failed\r\n", ctx->name);
    return HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(ctx->handle, 0) != HAL_OK) {
    printf("%s: digital filter cfg failed\r\n", ctx->name);
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

  printf("%s fault count=%lu\r\n", ctx->name,
         (unsigned long)ctx->consecutive_failures);

  if (ctx->consecutive_failures < I2C_FAULT_RECOVERY_THRESHOLD) {
    return HAL_ERROR;
  }

  printf("%s attempting bus recovery\r\n", ctx->name);

  HAL_StatusTypeDef status = I2C_RecoverBus(ctx);
  if (status == HAL_OK) {
    printf("%s recovery succeeded\r\n", ctx->name);
    ctx->consecutive_failures = 0u;
  } else {
    printf("%s recovery failed\r\n", ctx->name);
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

static void AppEncoder_RunSample(void) {
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

  if ((s1 == HAL_OK) && (s2 == HAL_OK)) {
    printf("A1=%u (%.2fdeg), A2=%u (%.2fdeg)\r\n", angle1, deg1, angle2, deg2);
  } else {
    printf("ERR A1=%u (%.2fdeg), A2=%u (%.2fdeg)\r\n", angle1, deg1, angle2,
           deg2);
  }

  AppEncoder_SendStatus(0U);
  AppEncoder_SendStatus(1U);
}

static void AppEncoder_SendStatus(uint8_t index) {
  if (index >= 2U) {
    return;
  }
  uint8_t payload[4] = {0};
  PackS24BE(&payload[0], encoder_position_q24[index]);
  payload[3] = encoder_status_flags[index];
  (void)AppCAN_SendFrame(&encoder_headers[index], payload);
}
