#include "app_can.h"

#include <stddef.h>
#include <stdio.h>

#include "stm32h5xx_hal_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE BEGIN FDCAN_REINIT_EXTERN */
extern void App_FDCAN1_Reinit(void);
/* USER CODE END FDCAN_REINIT_EXTERN */

#define CAN_ERROR_FLAG_BUS_OFF (1UL << 9)
#define APP_CAN_MAX_CALLBACKS 4U

static volatile uint8_t fdcan_bus_off_pending = 0U;
static volatile uint32_t fdcan_bus_off_request_tick = 0U;
static AppCAN_BusResetCallback reset_callbacks[APP_CAN_MAX_CALLBACKS] = {0};

void AppCAN_Init(void) {
  fdcan_bus_off_pending = 0U;
  fdcan_bus_off_request_tick = 0U;
  for (size_t i = 0; i < APP_CAN_MAX_CALLBACKS; ++i) {
    reset_callbacks[i] = NULL;
  }
}

void AppCAN_RegisterBusResetCallback(AppCAN_BusResetCallback callback) {
  if (callback == NULL) {
    return;
  }
  for (size_t i = 0; i < APP_CAN_MAX_CALLBACKS; ++i) {
    if (reset_callbacks[i] == callback) {
      return;
    }
    if (reset_callbacks[i] == NULL) {
      reset_callbacks[i] = callback;
      return;
    }
  }
}

void AppCAN_InitHeader(FDCAN_TxHeaderTypeDef *header, uint32_t identifier,
                       uint32_t data_length) {
  if (header == NULL) {
    return;
  }
  header->IdType = FDCAN_STANDARD_ID;
  header->TxFrameType = FDCAN_DATA_FRAME;
  header->DataLength = data_length;
  header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header->BitRateSwitch = FDCAN_BRS_OFF;
  header->FDFormat = FDCAN_CLASSIC_CAN;
  header->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header->MessageMarker = 0U;
  header->Identifier = identifier;
}

HAL_StatusTypeDef AppCAN_SendFrame(FDCAN_TxHeaderTypeDef *header,
                                   const uint8_t *payload) {
  uint8_t scratch[8] = {0};
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
    printf("CAN TX busy (fifo full), id=0x%03lX\r\n",
           (unsigned long)header->Identifier);
    FDCAN_ProtocolStatusTypeDef proto = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto);
    uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
    printf("CAN diag: act=%u lec=%u bo=%u err=0x%08lX\r\n",
           (unsigned int)proto.Activity, (unsigned int)proto.LastErrorCode,
           (unsigned int)proto.BusOff, (unsigned long)err);
    if ((proto.BusOff != 0U) || ((err & CAN_ERROR_FLAG_BUS_OFF) != 0U)) {
      AppCAN_ScheduleBusOffRecovery();
    }
    return HAL_BUSY;
  }

  uint8_t *tx_payload = (payload != NULL) ? (uint8_t *)payload : scratch;
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
      &hfdcan1, header, tx_payload);
  if (status != HAL_OK) {
    printf("CAN TX failed status=%ld id=0x%03lX\r\n", (long)status,
           (unsigned long)header->Identifier);
    FDCAN_ProtocolStatusTypeDef proto = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto);
    uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
    printf("CAN diag: act=%u lec=%u bo=%u err=0x%08lX\r\n",
           (unsigned int)proto.Activity, (unsigned int)proto.LastErrorCode,
           (unsigned int)proto.BusOff, (unsigned long)err);
    if ((proto.BusOff != 0U) || ((err & CAN_ERROR_FLAG_BUS_OFF) != 0U)) {
      AppCAN_ScheduleBusOffRecovery();
    }
  }
  return status;
}

void AppCAN_ScheduleBusOffRecovery(void) {
  if (fdcan_bus_off_pending == 0U) {
    fdcan_bus_off_pending = 1U;
    fdcan_bus_off_request_tick = HAL_GetTick();
  }
}

void AppCAN_ServiceBusOff(void) {
  if (fdcan_bus_off_pending == 0U) {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - fdcan_bus_off_request_tick) < 200U) {
    return;
  }

  printf("CAN bus-off recovery start\r\n");

  if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
    printf("CAN stop failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK) {
    printf("CAN deinit failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  /* USER CODE BEGIN FDCAN_REINIT_CALL */
  App_FDCAN1_Reinit();
  /* USER CODE END FDCAN_REINIT_CALL */

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    printf("CAN restart failed, retry later\r\n");
    fdcan_bus_off_request_tick = now;
    return;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0U) !=
      HAL_OK) {
    printf("CAN notification re-arm failed\r\n");
  }

  printf("CAN bus-off recovery complete\r\n");
  fdcan_bus_off_pending = 0U;

  for (size_t i = 0; i < APP_CAN_MAX_CALLBACKS; ++i) {
    if (reset_callbacks[i] != NULL) {
      reset_callbacks[i]();
    }
  }
}

void AppCAN_HandleErrorStatus(uint32_t error_status) {
  if ((error_status & FDCAN_IT_BUS_OFF) != 0U) {
    printf("CAN bus-off interrupt\r\n");
    AppCAN_ScheduleBusOffRecovery();
  }
}
