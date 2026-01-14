#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/*
 * Centralized runtime configuration used by app modules. All fields are
 * 0/1 for enable flags except can_base_id, which is an 11-bit standard CAN
 * identifier that serves as the base for all four status frames.
 *
 * Layout (relative to can_base_id):
 *   encoder1: base + 0
 *   limit switch 1: base + 1
 *   encoder2: base + 2
 *   limit switch 2: base + 3
 */
typedef struct {
  uint8_t enable_sw1;
  uint8_t enable_sw2;
  uint8_t enable_encoder1;
  uint8_t enable_encoder2;
  uint16_t can_base_id; /* must be <= 0x7FC to keep +3 within 11-bit range */
  /* Optional explicit IDs (0 = use base offsets). Must be <= 0x7FF. */
  uint16_t can_id_encoder1;
  uint16_t can_id_sw1;
  uint16_t can_id_encoder2;
  uint16_t can_id_sw2;
} AppConfig;

/* Populate a config struct with the compiled-in defaults. */
void AppConfig_InitDefaults(AppConfig *cfg);

/* Validate and store the provided config. Returns 1 on success, 0 on reject. */
uint8_t AppConfig_Set(const AppConfig *cfg);

/* Access the currently active configuration. */
const AppConfig *AppConfig_Get(void);

#endif /* APP_CONFIG_H */
