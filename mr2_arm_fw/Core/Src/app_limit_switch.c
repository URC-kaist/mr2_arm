#include "app_limit_switch.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_can.h"
#include "main.h"

#define LIMIT_SW1_NC_MASK (1u << 0)
#define LIMIT_SW1_NO_MASK (1u << 1)
#define LIMIT_SW2_NC_MASK (1u << 2)
#define LIMIT_SW2_NO_MASK (1u << 3)
#define LIMIT_SWITCH_PERIODIC_MS 200u
#define LIMIT_SWITCH_FAULT_DEADTIME_MS 500u
#define CAN_ID_LIMIT_SWITCH1 0x181U
#define CAN_ID_LIMIT_SWITCH2 0x183U

typedef struct {
  uint8_t mask_nc;
  uint8_t mask_no;
  const char *name;
  FDCAN_TxHeaderTypeDef header;
  uint8_t frame[2];
  volatile uint8_t pending;
  uint8_t initialized;
  uint8_t last_state;
  uint8_t last_fault;
  uint32_t next_periodic_tick;
  uint32_t next_fault_allowed_tick;
} AppLimitSwitchChannel;

static AppLimitSwitchChannel limit_switch_channels[2] = {
    {.mask_nc = LIMIT_SW1_NC_MASK, .mask_no = LIMIT_SW1_NO_MASK, .name = "SW1"},
    {.mask_nc = LIMIT_SW2_NC_MASK, .mask_no = LIMIT_SW2_NO_MASK, .name = "SW2"},
};

static void AppLimitSwitch_ChannelQueue(AppLimitSwitchChannel *ch,
                                        uint8_t state, uint8_t fault,
                                        uint8_t log_change);
static uint8_t AppLimitSwitch_ReadPins(void);
static void AppLimitSwitch_OnBusReset(void);
static void AppLimitSwitch_RefreshHeaders(void);

void AppLimitSwitch_Init(void) {
  for (size_t i = 0; i < (sizeof(limit_switch_channels) /
                          sizeof(limit_switch_channels[0]));
       ++i) {
    AppLimitSwitchChannel *ch = &limit_switch_channels[i];
    ch->pending = 0u;
    ch->initialized = 0u;
    ch->last_state = 0u;
    ch->last_fault = 0u;
    ch->next_periodic_tick = 0u;
    ch->next_fault_allowed_tick = 0u;
    memset(ch->frame, 0, sizeof(ch->frame));
  }
  AppLimitSwitch_RefreshHeaders();
  AppCAN_RegisterBusResetCallback(AppLimitSwitch_OnBusReset);
}

void AppLimitSwitch_Poll(void) {
  uint32_t now = HAL_GetTick();
  uint8_t raw_mask = AppLimitSwitch_ReadPins();

  for (size_t idx = 0; idx < (sizeof(limit_switch_channels) /
                              sizeof(limit_switch_channels[0]));
       ++idx) {
    AppLimitSwitchChannel *ch = &limit_switch_channels[idx];
    uint8_t raw_nc = (uint8_t)((raw_mask & ch->mask_nc) != 0u ? 1u : 0u);
    uint8_t raw_no = (uint8_t)((raw_mask & ch->mask_no) != 0u ? 1u : 0u);
    uint8_t fault = (uint8_t)((raw_nc == raw_no) ? 1u : 0u);
    uint8_t state = ch->last_state;
    if (fault == 0u) {
      state = (uint8_t)((raw_no == 0u) ? 1u : 0u);
    }

    uint8_t first_sample = (ch->initialized == 0u) ? 1u : 0u;
    uint8_t state_changed =
        (fault == 0u)
            ? (uint8_t)((ch->initialized == 0u) ? 1u
                                                : (state != ch->last_state))
            : 0u;
    uint8_t fault_changed =
        (ch->initialized == 0u) ? 1u : (uint8_t)(fault != ch->last_fault);
    uint8_t queue_now = 0u;
    uint8_t log_change = 0u;

    if (first_sample != 0u) {
      ch->initialized = 1u;
      queue_now = 1u;
      log_change = 1u;
    } else if (fault != 0u) {
      if (fault_changed != 0u) {
        queue_now = 1u;
        log_change = 1u;
      } else if ((int32_t)(now - ch->next_fault_allowed_tick) >= 0) {
        queue_now = 1u;
      }
    } else {
      if ((state_changed != 0u) || (fault_changed != 0u)) {
        queue_now = 1u;
        log_change = 1u;
      } else if ((int32_t)(now - ch->next_periodic_tick) >= 0) {
        if (ch->pending == 0u) {
          queue_now = 1u;
        }
      }
    }

    if (queue_now != 0u) {
      ch->last_state = state;
      ch->last_fault = fault;
      if (fault != 0u) {
        ch->next_periodic_tick = now + LIMIT_SWITCH_FAULT_DEADTIME_MS;
        ch->next_fault_allowed_tick = now + LIMIT_SWITCH_FAULT_DEADTIME_MS;
      } else {
        ch->next_periodic_tick = now + LIMIT_SWITCH_PERIODIC_MS;
        ch->next_fault_allowed_tick = now;
      }
      AppLimitSwitch_ChannelQueue(ch, state, fault, log_change);
    }
  }
}

bool AppLimitSwitch_Service(void) {
  bool sent_frame = false;
  for (size_t i = 0; i < (sizeof(limit_switch_channels) /
                          sizeof(limit_switch_channels[0]));
       ++i) {
    AppLimitSwitchChannel *ch = &limit_switch_channels[i];
    if (ch->pending == 0U) {
      continue;
    }

    uint8_t payload[2] = {0};
    uint8_t have_frame = 0U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ch->pending != 0U) {
      memcpy(payload, ch->frame, sizeof(payload));
      ch->pending = 0U;
      have_frame = 1U;
    }
    if (primask == 0U) {
      __enable_irq();
    }

    if (have_frame == 0U) {
      continue;
    }

    HAL_StatusTypeDef tx_status = AppCAN_SendFrame(&ch->header, payload);
    if (tx_status == HAL_OK) {
      sent_frame = true;
      continue;
    }

    uint32_t primask2 = __get_PRIMASK();
    __disable_irq();
    memcpy(ch->frame, payload, sizeof(payload));
    ch->pending = 1U;
    if (primask2 == 0U) {
      __enable_irq();
    }
  }
  return sent_frame;
}

bool AppLimitSwitch_HasPending(void) {
  for (size_t i = 0; i < (sizeof(limit_switch_channels) /
                          sizeof(limit_switch_channels[0]));
       ++i) {
    if (limit_switch_channels[i].pending != 0U) {
      return true;
    }
  }
  return false;
}

static void AppLimitSwitch_RefreshHeaders(void) {
  AppCAN_InitHeader(&limit_switch_channels[0].header, CAN_ID_LIMIT_SWITCH1,
                    FDCAN_DLC_BYTES_2);
  AppCAN_InitHeader(&limit_switch_channels[1].header, CAN_ID_LIMIT_SWITCH2,
                    FDCAN_DLC_BYTES_2);
}

static void AppLimitSwitch_OnBusReset(void) {
  AppLimitSwitch_RefreshHeaders();
  AppLimitSwitch_Poll();
}

static uint8_t AppLimitSwitch_ReadPins(void) {
  uint8_t state = 0u;

  if (HAL_GPIO_ReadPin(SW1_NC_GPIO_Port, SW1_NC_Pin) == GPIO_PIN_SET) {
    state |= LIMIT_SW1_NC_MASK;
  }
  if (HAL_GPIO_ReadPin(SW1_NO_GPIO_Port, SW1_NO_Pin) == GPIO_PIN_SET) {
    state |= LIMIT_SW1_NO_MASK;
  }
  if (HAL_GPIO_ReadPin(SW2_NC_GPIO_Port, SW2_NC_Pin) == GPIO_PIN_SET) {
    state |= LIMIT_SW2_NC_MASK;
  }
  if (HAL_GPIO_ReadPin(SW2_NO_GPIO_Port, SW2_NO_Pin) == GPIO_PIN_SET) {
    state |= LIMIT_SW2_NO_MASK;
  }

  return state;
}

static void AppLimitSwitch_ChannelQueue(AppLimitSwitchChannel *ch,
                                        uint8_t state, uint8_t fault,
                                        uint8_t log_change) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  ch->frame[0] = state;
  ch->frame[1] = fault;
  ch->pending = 1u;
  if (primask == 0u) {
    __enable_irq();
  }

  if (log_change != 0u) {
    const char *state_str = (state != 0u) ? "closed" : "open";
    const char *fault_str = (fault != 0u) ? "active" : "none";
    printf("%s limit switch: state=%s fault=%s\r\n", ch->name, state_str,
           fault_str);
  }
}
