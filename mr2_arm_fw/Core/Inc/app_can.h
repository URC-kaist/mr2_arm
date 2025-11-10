#ifndef APP_CAN_H
#define APP_CAN_H

#include "main.h"

typedef void (*AppCAN_BusResetCallback)(void);

void AppCAN_Init(void);
void AppCAN_InitHeader(FDCAN_TxHeaderTypeDef *header, uint32_t identifier,
                       uint32_t data_length);
HAL_StatusTypeDef AppCAN_SendFrame(FDCAN_TxHeaderTypeDef *header,
                                   const uint8_t *payload);
void AppCAN_ScheduleBusOffRecovery(void);
void AppCAN_ServiceBusOff(void);
void AppCAN_HandleErrorStatus(uint32_t error_status);
void AppCAN_RegisterBusResetCallback(AppCAN_BusResetCallback callback);

#endif /* APP_CAN_H */
