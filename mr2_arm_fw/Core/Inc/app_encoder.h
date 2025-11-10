#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdbool.h>

void AppEncoder_Init(void);
void AppEncoder_RequestMeasurement(void);
void AppEncoder_OnSchedulerTick(void);
bool AppEncoder_Service(void);

#endif /* APP_ENCODER_H */
