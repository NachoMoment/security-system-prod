/* External-triggered camera/inference main
 * - Initializes camera once at boot (detect profile)
 * - Starts TF task on first ESP-NOW packet
 * - Extends while triggers continue
 * - Stops TF task after cooldown with no triggers (camera stays up)
 * - While the recorder is active, we pause the TF loop so the camera
 *   is owned by exactly one component at a time.
 */
// ----------------------- C++ / system headers (no extern "C") -----------------------
#include <cstring>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "esp_camera.h"

// ---------------------- Your pure-C headers (force C linkage) -----------------------
extern "C" {
  #include "main_functions.h"   // setup(), loop() - often C
  #include "psram_probe.h"      // C helpers
  #include "app_camera_esp.h"   // camera wrapper in C
  #include "recorder.h"         // recorder API in C
  #include "app_now_rx.h"       // ESP-NOW glue in C
  #include "record_gate.h"
}

static const char *TAG = "main";

// ==== tuneables ====
static const uint32_t kCooldownMs    = 60000;    // stop TF 60s after last trigger
static const uint32_t kTfTaskStack   = 16 * 1024;
static const UBaseType_t kTfTaskPrio = 8;

// ==== state ====
static TaskHandle_t s_tf_task = nullptr;
static esp_timer_handle_t s_stop_timer = nullptr;
static volatile int64_t s_last_trigger_us = 0;

// ---- TF task: mirrors the person_detection example structure ----
static void tf_task(void *){
  setup();                    // model init (camera is already up)
  for(;;){
    // If recorder is active, yield camera ownership and avoid calling loop()
    if (recorder_is_recording()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    loop();                   // one inference iteration (captures a frame)
    vTaskDelay(1);            // be nice to Wi-Fi/idle
  }
}

// ---- Stop timer callback: fires if no triggers for kCooldownMs ----
static void stop_cb(void*){
  const int64_t idle_us = esp_timer_get_time() - s_last_trigger_us;

  // Don't tear down TF while the recorder owns the camera; rearm instead.
  if (recorder_is_recording()) {
    esp_timer_stop(s_stop_timer);
    esp_timer_start_once(s_stop_timer, (uint64_t)kCooldownMs * 1000);
    return;
  }

  if (idle_us >= (int64_t)kCooldownMs * 1000) {
    if (s_tf_task) {
      ESP_LOGI(TAG, "Stopping TF task (cooldown %u ms)", (unsigned)kCooldownMs);
      rec_gate_reset();
      vTaskDelete(s_tf_task);
      s_tf_task = nullptr;

      // Keep camera up; re-probing it repeatedly is fragile and wastes time.
      // esp_camera_deinit();  // intentionally not called
    }
  }
}

// ---- Called by now_rx when a valid ESP-NOW packet arrives ----
static void on_external_trigger(uint32_t seq, uint32_t ms){
  s_last_trigger_us = esp_timer_get_time();

  if (!s_tf_task) {
    ESP_LOGI(TAG, "Trigger seq=%u ms=%u -> starting TF/camera",
             (unsigned)seq, (unsigned)ms);
    rec_gate_reset();
    xTaskCreate(tf_task, "tf_main", kTfTaskStack, nullptr, kTfTaskPrio, &s_tf_task);
  }

  // arm/extend the stop timer to kCooldownMs after the *latest* trigger
  if (s_stop_timer) {
    esp_timer_stop(s_stop_timer);
    esp_timer_start_once(s_stop_timer, (uint64_t)kCooldownMs * 1000);
  }
}

// Small helper to log this device's STA MAC (so you can set the sender's peer)
static void log_sta_mac(void){
  uint8_t mac[6] = {0};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    ESP_LOGI(TAG, "Receiver STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}

// Decode helpers just for logging
static const char* pixformat_name(pixformat_t f) {
  switch (f) {
    case PIXFORMAT_GRAYSCALE: return "GRAYSCALE";
    case PIXFORMAT_RGB565:    return "RGB565";
    case PIXFORMAT_YUV422:    return "YUV422";
    case PIXFORMAT_JPEG:      return "JPEG";
    default:                  return "UNK";
  }
}

static void camera_force_detect_profile(void) {
  // Force OV2640 into the tiny "detect" profile: QQVGA + GRAYSCALE.
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    ESP_LOGE("app_camera", "sensor get failed");
    return;
  }

  // Set grayscale + QQVGA. (These calls are safe even if already set.)
  s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
  s->set_framesize(s, FRAMESIZE_QQVGA);    // 160x120 => 19200 bytes in GRAYSCALE

  // Optional: small tweaks that often help with overflow/noise at tiny res.
  s->set_brightness(s, 1);                 // -2..2
  s->set_contrast(s,   0);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, GAINCEILING_2X);

  // Drain a couple of frames so the DMA/FIFO is in a steady state.
  for (int i = 0; i < 3; ++i) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // One more log line showing what the sensor currently reports
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE("app_camera", "sanity: fb NULL (no frame)");
    return;
  }
  ESP_LOGI("app_camera",
           "sanity: format=%s(%d) w=%d h=%d len=%d (expect GRAYSCALE,160x120,19200)",
           pixformat_name(fb->format), fb->format, fb->width, fb->height, fb->len);
  esp_camera_fb_return(fb);
}

extern "C" void app_main(void) {
  //psram_probe_log();  // optional PSRAM report

  // NVS bring-up (handle page exhaustion gracefully)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize camera ONCE at boot in the tiny detect profile.
  ESP_ERROR_CHECK(app_camera_init());

  camera_force_detect_profile();

  // Mount / prepare SD (non-fatal: continue if it times out; recorder will retry later)
  ret = recorder_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SD not ready (%s); continuing without it. We'll try again on the next motion trigger.",
             esp_err_to_name(ret));
  }

  // Bring up our ESP-NOW receiver (starts Wi-Fi STA and pins the channel)
  ESP_ERROR_CHECK(now_rx_init());
  //log_sta_mac();
  now_rx_on_trigger(on_external_trigger);

  // One-shot stop timer
  const esp_timer_create_args_t args = {.callback = &stop_cb, .name = "stop_timer"};
  ESP_ERROR_CHECK(esp_timer_create(&args, &s_stop_timer));

  // DO NOT start TF here — wait for external trigger packets
  vTaskDelete(nullptr);
}
