// main/app_camera_esp.c

#include <stdbool.h>
#include <string.h>
#include "sdkconfig.h"
#include "camera_pins.h"
#include "app_camera_esp.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

static const char *TAG = "app_camera";

// ----------------------------------------------------------------------------
// Internal state
// ----------------------------------------------------------------------------
static bool              s_cam_inited = false;
static app_cam_profile_t s_current    = APP_CAM_PROFILE_INVALID;

#ifndef FRAMESIZE_INVALID
#define FRAMESIZE_INVALID ((framesize_t)-1)
#endif

#ifndef XCLK_FREQ_HZ
#define XCLK_FREQ_HZ 20000000
#endif

#ifndef CAM_GRAB_MAX_TRIES
#define CAM_GRAB_MAX_TRIES 5
#endif

// ---- Bridge macros: if your pin header uses Y*_GPIO_NUM, alias to CAMERA_PIN_* ----
#ifndef CAMERA_PIN_D0
  #ifdef Y2_GPIO_NUM
    #define CAMERA_PIN_D0    Y2_GPIO_NUM
    #define CAMERA_PIN_D1    Y3_GPIO_NUM
    #define CAMERA_PIN_D2    Y4_GPIO_NUM
    #define CAMERA_PIN_D3    Y5_GPIO_NUM
    #define CAMERA_PIN_D4    Y6_GPIO_NUM
    #define CAMERA_PIN_D5    Y7_GPIO_NUM
    #define CAMERA_PIN_D6    Y8_GPIO_NUM
    #define CAMERA_PIN_D7    Y9_GPIO_NUM
    #define CAMERA_PIN_XCLK  XCLK_GPIO_NUM
    #define CAMERA_PIN_PCLK  PCLK_GPIO_NUM
    #define CAMERA_PIN_VSYNC VSYNC_GPIO_NUM
    #define CAMERA_PIN_HREF  HREF_GPIO_NUM
    #define CAMERA_PIN_SIOD  SIOD_GPIO_NUM
    #define CAMERA_PIN_SIOC  SIOC_GPIO_NUM
    #define CAMERA_PIN_RESET RESET_GPIO_NUM
    #define CAMERA_PIN_PWDN  PWDN_GPIO_NUM
  #endif
#endif

#ifndef CAMERA_PIN_D0
  #error "Camera pin macros not defined. Ensure camera_pins.h or Kconfig pin map is included."
#endif

// ----------------------------------------------------------------------------
// Small helpers
// ----------------------------------------------------------------------------
static void log_heaps(const char* where) {
  size_t iram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t ipsr = heap_caps_get_free_size(MALLOC_CAP_SPIRAM   | MALLOC_CAP_8BIT);
  size_t lir  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t lps  = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM   | MALLOC_CAP_8BIT);
  ESP_LOGI(TAG, "[HEAP] %-14s int=%uB (largest=%uB)  psram=%uB (largest=%uB)",
           where, (unsigned)iram, (unsigned)lir, (unsigned)ipsr, (unsigned)lps);
}


static inline bool driver_ready(void) { return esp_camera_sensor_get() != NULL; }

static void prime_frames(int count, const char* why) {
  for (int i = 0; i < count; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(8));
  }
  ESP_LOGI(TAG, "[PRIME] dropped %d frames (%s)", count, why ? why : "");
}

// ---------- Diagnostics helpers (must be before app_camera_diag_dump) ----------
static bool framesize_to_wh_safe(framesize_t fs, int *w, int *h) {
  *w = 0; *h = 0;
  switch (fs) {
#ifdef FRAMESIZE_96X96
    case FRAMESIZE_96X96: *w = 96;  *h = 96;  return true;
#endif
#ifdef FRAMESIZE_QQVGA
    case FRAMESIZE_QQVGA: *w = 160; *h = 120; return true;
#endif
#ifdef FRAMESIZE_QQVGA2
    case FRAMESIZE_QQVGA2:*w = 128; *h = 160; return true;
#endif
#ifdef FRAMESIZE_QVGA
    case FRAMESIZE_QVGA:  *w = 320; *h = 240; return true;
#endif
#ifdef FRAMESIZE_VGA
    case FRAMESIZE_VGA:   *w = 640; *h = 480; return true;
#endif
    default: return false;
  }
}

static int bpp_for_pixfmt(pixformat_t pf) {
  switch (pf) {
    case PIXFORMAT_GRAYSCALE: return 1;
    case PIXFORMAT_RGB565:    return 2;
    case PIXFORMAT_YUV422:    return 2;
    case PIXFORMAT_JPEG:      return -1; // compressed
    default:                  return -1;
  }
}

// ----------------------------------------------------------------------------
// Build config + start with profile
// ----------------------------------------------------------------------------
static void build_config(app_cam_profile_t prof, camera_config_t *cfg) {
  *cfg = (camera_config_t){
      .pin_pwdn     = CAMERA_PIN_PWDN,
      .pin_reset    = CAMERA_PIN_RESET,
      .pin_xclk     = CAMERA_PIN_XCLK,
      .pin_sccb_sda = CAMERA_PIN_SIOD,
      .pin_sccb_scl = CAMERA_PIN_SIOC,

      .pin_d7 = CAMERA_PIN_D7, .pin_d6 = CAMERA_PIN_D6,
      .pin_d5 = CAMERA_PIN_D5, .pin_d4 = CAMERA_PIN_D4,
      .pin_d3 = CAMERA_PIN_D3, .pin_d2 = CAMERA_PIN_D2,
      .pin_d1 = CAMERA_PIN_D1, .pin_d0 = CAMERA_PIN_D0,

      .pin_vsync = CAMERA_PIN_VSYNC,
      .pin_href  = CAMERA_PIN_HREF,
      .pin_pclk  = CAMERA_PIN_PCLK,

      .xclk_freq_hz = XCLK_FREQ_HZ,
      .ledc_timer   = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

#if CONFIG_ESP32S3_SPIRAM_SUPPORT
      .fb_location  = CAMERA_FB_IN_DRAM, // keep tiny GRAY frames in DRAM
#else
      .fb_location  = CAMERA_FB_IN_DRAM,
#endif
  };

  switch (prof) {
    case APP_CAM_PROFILE_DETECT_GRAY_96:
      cfg->pixel_format = PIXFORMAT_GRAYSCALE;
      cfg->frame_size   = FRAMESIZE_QQVGA;   // 160x120
      cfg->jpeg_quality = 12;
      cfg->fb_count     = 3;
      cfg->grab_mode    = CAMERA_GRAB_LATEST;
      cfg->fb_location  = CAMERA_FB_IN_DRAM; // tiny, keep in internal RAM
      break;

    case APP_CAM_PROFILE_RECORD_JPEG_VGA:
      cfg->pixel_format = PIXFORMAT_JPEG;
      cfg->frame_size   = FRAMESIZE_VGA;     // 640x480 JPEG
      cfg->jpeg_quality = 12;
      cfg->fb_count     = 1;                 // keep memory footprint small
      cfg->grab_mode    = CAMERA_GRAB_WHEN_EMPTY; // block for full frames
      cfg->fb_location  = CAMERA_FB_IN_PSRAM;     // <<< key change
      break;

    default:
      cfg->pixel_format = PIXFORMAT_GRAYSCALE;
      cfg->frame_size   = FRAMESIZE_QQVGA;
      cfg->jpeg_quality = 12;
      cfg->fb_count     = 3;
      cfg->grab_mode    = CAMERA_GRAB_LATEST;
      break;
  }
}

static esp_err_t camera_start_with_profile(app_cam_profile_t prof) {
  camera_config_t cfg;
  build_config(prof, &cfg);

  ESP_LOGI(TAG, "camera init: xclk=%uHz, fmt=%d, size=%d",
           (unsigned)cfg.xclk_freq_hz, (int)cfg.pixel_format, (int)cfg.frame_size);

  log_heaps("before init");
  esp_err_t err = esp_camera_init(&cfg);
  log_heaps(err == ESP_OK ? "after init OK" : "after init FAIL");
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
    return err;
  }

  // Match sensor registers to the requested profile.
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    if (prof == APP_CAM_PROFILE_RECORD_JPEG_VGA) {
      s->set_pixformat(s, PIXFORMAT_JPEG);
      s->set_framesize(s, FRAMESIZE_VGA);
      s->set_quality(s, 12);
    } else {
      s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
      s->set_framesize(s, FRAMESIZE_QQVGA);  // must match DMA layout
      // Optional mild tuning; safe on OV2640:
      // s->set_brightness(s, 0);
      // s->set_contrast(s,   0);
      // s->set_saturation(s, 0);
      // s->set_whitebal(s,   0);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(30));
  prime_frames(3, "after init");

  s_current    = prof;
  s_cam_inited = true;
  return ESP_OK;
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

esp_err_t app_camera_deinit_safe(void) {
  if (!s_cam_inited && !driver_ready()) return ESP_OK;
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_camera_deinit failed: %s", esp_err_to_name(err));
  }
  s_cam_inited = false;
  s_current    = APP_CAM_PROFILE_INVALID;
  ESP_LOGI(TAG, "camera deinit");
  return err;
}

esp_err_t app_camera_init(void) {
  if (s_cam_inited && !driver_ready()) {
    ESP_LOGW(TAG, "camera flag set but driver not ready; clearing");
    s_cam_inited = false;
    s_current    = APP_CAM_PROFILE_INVALID;
  }

  if (s_cam_inited && driver_ready()) {
    ESP_LOGI(TAG, "camera already initialized; skipping re-init");
    return ESP_OK;
  }

  esp_err_t err = camera_start_with_profile(APP_CAM_PROFILE_DETECT_GRAY_96);

  if (err == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "camera reported INVALID_STATE; deinit + retry");
    (void)app_camera_deinit_safe();
    vTaskDelay(pdMS_TO_TICKS(40));
    err = camera_start_with_profile(APP_CAM_PROFILE_DETECT_GRAY_96);
  }

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "camera init ok (detect profile)");
    // One-shot probe, logs exactly what the driver produces right now.
    // This helps confirm we’re getting GRAYSCALE 160x120 len=19200.
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW(TAG, "sanity: fb == NULL (right after init)");
    } else {
      ESP_LOGI(TAG, "sanity: format=%d w=%d h=%d len=%d (expect GRAYSCALE,160x120,19200)",
               (int)fb->format, fb->width, fb->height, (int)fb->len);
      esp_camera_fb_return(fb);
    }
  } else {
    ESP_LOGE(TAG, "camera init failed: %s", esp_err_to_name(err));
  }
  return err;
}

void app_camera_deinit(void) {
  (void)app_camera_deinit_safe();
}

esp_err_t app_camera_apply_profile(app_cam_profile_t prof) {
  // If not active, just bring it up in the requested profile.
  if (!s_cam_inited || !driver_ready()) {
    ESP_LOGI(TAG, "apply_profile: driver not active; init with requested profile");
    return camera_start_with_profile(prof);
  }
  if (s_current == prof) return ESP_OK;

  (void)app_camera_deinit_safe();
  vTaskDelay(pdMS_TO_TICKS(30));
  esp_err_t err = camera_start_with_profile(prof);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "profile -> %s",
             (prof == APP_CAM_PROFILE_RECORD_JPEG_VGA) ? "RECORD (JPEG VGA)"
                                                       : "DETECT (GRAY QQVGA)");
  }
  return err;
}

// ---------- Diagnostic dump ----------
void app_camera_diag_dump(const char* where)
{
  sensor_t *s = esp_camera_sensor_get();

  ESP_LOGI(TAG, "[CAM-DIAG] %-20s | inited=%d profile=%d",
           (where ? where : "?"), s_cam_inited ? 1 : 0, (int)s_current);

  if (!s) {
    ESP_LOGW(TAG, "[CAM-DIAG] sensor=NULL (driver not ready)");
    return;
  }

  int cfg_w = 0, cfg_h = 0;
  const bool fs_ok   = framesize_to_wh_safe(s->status.framesize, &cfg_w, &cfg_h);
  const int  exp_bpp = bpp_for_pixfmt(s->pixformat);

  ESP_LOGI(TAG, "[CAM-DIAG] sensor id MIDH=0x%02x MIDL=0x%02x PID=0x%02x VER=0x%02x",
           s->id.MIDH, s->id.MIDL, s->id.PID, s->id.VER);

  ESP_LOGI(TAG, "[CAM-DIAG] pixfmt=%d framesize=%d (%s) (%dx%d) bpp=%d xclk=%u",
           (int)s->pixformat, (int)s->status.framesize,
           fs_ok ? "known" : "unknown", cfg_w, cfg_h, exp_bpp, (unsigned)XCLK_FREQ_HZ);

  if (fs_ok && exp_bpp > 0) {
    ESP_LOGI(TAG, "[CAM-DIAG] expected frame bytes=%d", cfg_w * cfg_h * exp_bpp);
  }

  // Probe one frame (non-fatal). If NULL, we just report the timeout.
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGW(TAG, "[CAM-DIAG] fb_get(): NULL (timeout)");
    return;
  }

  int inferred_bpp = -1;
  if      ((int)fb->len == fb->width * fb->height)     inferred_bpp = 1;
  else if ((int)fb->len == fb->width * fb->height * 2) inferred_bpp = 2;

  const int expected_len = (fs_ok && exp_bpp > 0) ? (cfg_w * cfg_h * exp_bpp) : -1;
  const char *verdict = (expected_len > 0 && expected_len != (int)fb->len) ? "MISMATCH" : "OK";

  ESP_LOGI(TAG, "[CAM-DIAG] fb: %dx%d len=%u pf(enum)=%d inferred_bpp=%d expected=%d  %s",
           fb->width, fb->height, (unsigned)fb->len, (int)fb->format,
           inferred_bpp, expected_len, verdict);

  esp_camera_fb_return(fb);
}

/* Handy kick for callers after repeated fb_get() timeouts. */
esp_err_t app_camera_kick_detect(void) {
  ESP_LOGW(TAG, "[KICK] reinit DETECT profile after consecutive fb_get() timeouts");
  (void)app_camera_deinit_safe();
  vTaskDelay(pdMS_TO_TICKS(40));
  return camera_start_with_profile(APP_CAM_PROFILE_DETECT_GRAY_96);
}

// ----------------------------------------------------------------------------
// NEW: Safe acquisition helper (guarantees GRAY 160x120 len=19200)
// ----------------------------------------------------------------------------
// Returns a frame only if it is GRAYSCALE 160x120 and len==19200.
// On failure, it drops bad frames (returning them to the driver) and tries again.
// Caller must esp_camera_fb_return(*out_fb) when done.
bool app_camera_acquire_gray160x120(camera_fb_t **out_fb, int max_tries) {
  if (!out_fb) return false;
  const int tries = (max_tries > 0 ? max_tries : CAM_GRAB_MAX_TRIES);

  for (int i = 1; i <= tries; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW(TAG, "grab try %d/%d: fb == NULL", i, tries);
      continue;
    }

    const bool ok = (fb->format == PIXFORMAT_GRAYSCALE) &&
                    (fb->width  == 160) &&
                    (fb->height == 120) &&
                    (fb->len    == 160 * 120);

    if (ok) {
      ESP_LOGD(TAG, "grab try %d/%d: OK fmt=%d w=%d h=%d len=%d",
               i, tries, (int)fb->format, fb->width, fb->height, (int)fb->len);
      *out_fb = fb;
      return true;
    }

    ESP_LOGW(TAG,
      "grab try %d/%d: DROP bad frame fmt=%d w=%d h=%d len=%d (expect GRAYSCALE,160x120,19200)",
      i, tries, (int)fb->format, fb->width, fb->height, (int)fb->len);
    esp_camera_fb_return(fb);
  }

  ESP_LOGE(TAG, "failed to acquire a valid 160x120 GRAYSCALE frame after %d tries", tries);
  return false;
}
