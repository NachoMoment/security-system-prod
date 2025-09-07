#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*now_rx_cb_t)(uint32_t seq, uint32_t ms);

esp_err_t now_rx_init(void);
void now_rx_on_trigger(now_rx_cb_t cb);

#ifdef __cplusplus
}
#endif
