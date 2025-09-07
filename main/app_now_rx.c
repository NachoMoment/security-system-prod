#include <string.h>
#include <stdint.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"

#include "recorder.h"
#include "app_now_rx.h"

static const char *TAG = "now_rx";
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 1
#endif

// ----- callback registration -----
static now_rx_cb_t s_cb = NULL;

void now_rx_on_trigger(now_rx_cb_t cb) {
  s_cb = cb;  // main.cc registers on_external_trigger() here
}

// ----- incoming packet shape -----
typedef struct __attribute__((packed)) {
  uint32_t seq;   // monotonically increasing sequence number from sender
  uint32_t ms;    // sender's ms timestamp (optional metadata)
} motion_msg_t;

// ----- simple de-dupe -----
static uint32_t s_last_seq = 0;

// IDF v5.x signature for esp_now_register_recv_cb()
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info; // not used, but available for sender MAC/channel/etc.

  if (len < (int)sizeof(motion_msg_t)) return;

  motion_msg_t msg;
  memcpy(&msg, data, sizeof(msg));

  // De-duplicate repeated packets
  if (msg.seq == s_last_seq) return;
  s_last_seq = msg.seq;

  ESP_LOGI(TAG, "Motion trigger — seq=%u ms=%u", (unsigned)msg.seq, (unsigned)msg.ms);

  if (s_cb) {
    s_cb(msg.seq, msg.ms);   // Preferred: hand off to main
  } else if (!recorder_is_recording()) {
    recorder_start();        // Simple fallback so link succeeds
  }
}

esp_err_t now_rx_init(void) {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  // Pin Wi-Fi to ESPNOW channel so sender/receiver match
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
  ESP_LOGI(TAG, "ESP-NOW RX ready on channel %d", ESPNOW_CHANNEL);
  return ESP_OK;
}
