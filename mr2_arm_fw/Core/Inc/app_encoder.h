#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdbool.h>
#include "app_config.h"

void AppEncoder_Init(void);
void AppEncoder_RequestMeasurement(void);
void AppEncoder_OnSchedulerTick(void);
bool AppEncoder_Service(void);
void AppEncoder_ApplyConfig(const AppConfig *cfg);

#endif /* APP_ENCODER_H */
