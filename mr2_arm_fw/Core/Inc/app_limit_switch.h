#ifndef APP_LIMIT_SWITCH_H
#define APP_LIMIT_SWITCH_H

#include <stdbool.h>
#include "app_config.h"

void AppLimitSwitch_Init(void);
void AppLimitSwitch_Poll(void);
bool AppLimitSwitch_Service(void);
bool AppLimitSwitch_HasPending(void);
void AppLimitSwitch_ApplyConfig(const AppConfig *cfg);

#endif /* APP_LIMIT_SWITCH_H */
