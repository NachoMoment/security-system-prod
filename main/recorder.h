#pragma once
#include "esp_err.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t recorder_init(void);
bool      recorder_is_recording(void);
esp_err_t recorder_start(void);
esp_err_t recorder_stop(void);
#ifdef __cplusplus
}
#endif
