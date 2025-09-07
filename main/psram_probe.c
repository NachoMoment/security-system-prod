#include "psram_probe.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#if CONFIG_SPIRAM
  #include "esp_psram.h"
#endif

static const char* TAG = "psram_probe";

void psram_probe_log(void) {
    size_t sz = 0;
#if CONFIG_SPIRAM
    sz = esp_psram_get_size();  // only available when SPIRAM support is enabled
#endif
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM configured: %s, size=%u bytes (%.2f MB), free SPIRAM=%u",
             (sz ? "YES" : "NO/ABSENT"),
             (unsigned)sz, sz / (1024.0 * 1024.0),
             (unsigned)free_spiram);
}
