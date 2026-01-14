#include "app_config.h"

#include <string.h>

#define APP_CAN_BASE_ID_MAX 0x7FCu
#define APP_CAN_ID_MAX 0x7FFu

static AppConfig current_config;

void AppConfig_InitDefaults(AppConfig *cfg) {
  if (cfg == NULL) {
    return;
  }
  cfg->enable_sw1 = 1u;
  cfg->enable_sw2 = 1u;
  cfg->enable_encoder1 = 1u;
  cfg->enable_encoder2 = 1u;
  cfg->can_base_id = 0x180u;
  cfg->can_id_encoder1 = 0u;
  cfg->can_id_sw1 = 0u;
  cfg->can_id_encoder2 = 0u;
  cfg->can_id_sw2 = 0u;
}

uint8_t AppConfig_Set(const AppConfig *cfg) {
  if (cfg == NULL) {
    return 0u;
  }

  if (cfg->can_base_id > APP_CAN_BASE_ID_MAX) {
    return 0u;
  }
  if ((cfg->can_id_encoder1 > APP_CAN_ID_MAX) ||
      (cfg->can_id_sw1 > APP_CAN_ID_MAX) ||
      (cfg->can_id_encoder2 > APP_CAN_ID_MAX) ||
      (cfg->can_id_sw2 > APP_CAN_ID_MAX)) {
    return 0u;
  }

  current_config = *cfg;
  return 1u;
}

const AppConfig *AppConfig_Get(void) { return &current_config; }
