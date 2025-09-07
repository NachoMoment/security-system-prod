// app_camera_esp.h
// Copyright 2019 The TensorFlow Authors.
// Licensed under the Apache License, Version 2.0

#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_

#include "esp_err.h"
#include "esp_camera.h"
#include "sensor.h"
#include "esp_log.h"
#include "esp_system.h"
// NOTE: esp_main.h is not a standard ESP-IDF header; remove it to avoid include failures.
// #include "esp_main.h"

/* -----------------------------------------------------------------------------
   Default capture profile macros

   These define the *boot/detect* format; your runtime code can switch profiles
   (e.g., to JPEG VGA) via app_camera_apply_profile().
----------------------------------------------------------------------------- */
#if defined(DISPLAY_SUPPORT)
  #define CAMERA_PIXEL_FORMAT PIXFORMAT_RGB565
#else
  #define CAMERA_PIXEL_FORMAT PIXFORMAT_GRAYSCALE
#endif

#define CAMERA_FRAME_SIZE FRAMESIZE_QQVGA   // detect profile default
#define XCLK_FREQ_HZ 10000000               // external clock

/* -----------------------------------------------------------------------------
   Pin maps via Kconfig
----------------------------------------------------------------------------- */
#if CONFIG_CAMERA_MODULE_WROVER_KIT
  #define CAMERA_MODULE_NAME "Wrover Kit"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET -1
  #define CAMERA_PIN_XCLK  21
  #define CAMERA_PIN_SIOD  26
  #define CAMERA_PIN_SIOC  27
  #define CAMERA_PIN_D7    35
  #define CAMERA_PIN_D6    34
  #define CAMERA_PIN_D5    39
  #define CAMERA_PIN_D4    36
  #define CAMERA_PIN_D3    19
  #define CAMERA_PIN_D2    18
  #define CAMERA_PIN_D1    5
  #define CAMERA_PIN_D0    4
  #define CAMERA_PIN_VSYNC 25
  #define CAMERA_PIN_HREF  23
  #define CAMERA_PIN_PCLK  22

#elif CONFIG_CAMERA_MODULE_ESP_EYE
  #define CAMERA_MODULE_NAME "ESP-EYE"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET -1
  #define CAMERA_PIN_XCLK  4
  #define CAMERA_PIN_SIOD  18
  #define CAMERA_PIN_SIOC  23
  #define CAMERA_PIN_D7    36
  #define CAMERA_PIN_D6    37
  #define CAMERA_PIN_D5    38
  #define CAMERA_PIN_D4    39
  #define CAMERA_PIN_D3    35
  #define CAMERA_PIN_D2    14
  #define CAMERA_PIN_D1    13
  #define CAMERA_PIN_D0    34
  #define CAMERA_PIN_VSYNC 5
  #define CAMERA_PIN_HREF  27
  #define CAMERA_PIN_PCLK  25

#elif CONFIG_CAMERA_MODULE_ESP_S2_KALUGA
  #define CAMERA_MODULE_NAME "ESP-S2-KALUGA"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET -1
  #define CAMERA_PIN_XCLK  1
  #define CAMERA_PIN_SIOD  8
  #define CAMERA_PIN_SIOC  7
  #define CAMERA_PIN_D7    38
  #define CAMERA_PIN_D6    21
  #define CAMERA_PIN_D5    40
  #define CAMERA_PIN_D4    39
  #define CAMERA_PIN_D3    42
  #define CAMERA_PIN_D2    41
  #define CAMERA_PIN_D1    37
  #define CAMERA_PIN_D0    36
  #define CAMERA_PIN_VSYNC 2
  #define CAMERA_PIN_HREF  3
  #define CAMERA_PIN_PCLK  33

#elif CONFIG_CAMERA_MODULE_ESP_S3_EYE
  #define CAMERA_MODULE_NAME "ESP-S3-EYE"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET -1
  #define CAMERA_PIN_VSYNC 6
  #define CAMERA_PIN_HREF  7
  #define CAMERA_PIN_PCLK  13
  #define CAMERA_PIN_XCLK  15
  #define CAMERA_PIN_SIOD  4
  #define CAMERA_PIN_SIOC  5
  #define CAMERA_PIN_D0    11
  #define CAMERA_PIN_D1    9
  #define CAMERA_PIN_D2    8
  #define CAMERA_PIN_D3    10
  #define CAMERA_PIN_D4    12
  #define CAMERA_PIN_D5    18
  #define CAMERA_PIN_D6    17
  #define CAMERA_PIN_D7    16

#elif CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
  #define CAMERA_MODULE_NAME "ESP-DEVCAM"
  #define CAMERA_PIN_PWDN  32
  #define CAMERA_PIN_RESET 33
  #define CAMERA_PIN_XCLK  4
  #define CAMERA_PIN_SIOD  18
  #define CAMERA_PIN_SIOC  23
  #define CAMERA_PIN_D7    36
  #define CAMERA_PIN_D6    19
  #define CAMERA_PIN_D5    21
  #define CAMERA_PIN_D4    39
  #define CAMERA_PIN_D3    35
  #define CAMERA_PIN_D2    14
  #define CAMERA_PIN_D1    13
  #define CAMERA_PIN_D0    34
  #define CAMERA_PIN_VSYNC 5
  #define CAMERA_PIN_HREF  27
  #define CAMERA_PIN_PCLK  25

#elif CONFIG_CAMERA_MODULE_M5STACK_PSRAM
  #define CAMERA_MODULE_NAME "M5STACK-PSRAM"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET 15
  #define CAMERA_PIN_XCLK  27
  #define CAMERA_PIN_SIOD  25
  #define CAMERA_PIN_SIOC  23
  #define CAMERA_PIN_D7    19
  #define CAMERA_PIN_D6    36
  #define CAMERA_PIN_D5    18
  #define CAMERA_PIN_D4    39
  #define CAMERA_PIN_D3    5
  #define CAMERA_PIN_D2    34
  #define CAMERA_PIN_D1    35
  #define CAMERA_PIN_D0    32
  #define CAMERA_PIN_VSYNC 22
  #define CAMERA_PIN_HREF  26
  #define CAMERA_PIN_PCLK  21

#elif CONFIG_CAMERA_MODULE_M5STACK_WIDE
  #define CAMERA_MODULE_NAME "M5STACK-WIDE"
  #define CAMERA_PIN_PWDN  -1
  #define CAMERA_PIN_RESET 15
  #define CAMERA_PIN_XCLK  27
  #define CAMERA_PIN_SIOD  22
  #define CAMERA_PIN_SIOC  23
  #define CAMERA_PIN_D7    19
  #define CAMERA_PIN_D6    36
  #define CAMERA_PIN_D5    18
  #define CAMERA_PIN_D4    39
  #define CAMERA_PIN_D3    5
  #define CAMERA_PIN_D2    34
  #define CAMERA_PIN_D1    35
  #define CAMERA_PIN_D0    32
  #define CAMERA_PIN_VSYNC 25
  #define CAMERA_PIN_HREF  26
  #define CAMERA_PIN_PCLK  21

#elif CONFIG_CAMERA_MODULE_AI_THINKER
  #define CAMERA_MODULE_NAME "AI-THINKER"
  #define CAMERA_PIN_PWDN  32
  #define CAMERA_PIN_RESET -1
  #define CAMERA_PIN_XCLK  0
  #define CAMERA_PIN_SIOD  26
  #define CAMERA_PIN_SIOC  27
  #define CAMERA_PIN_D7    35
  #define CAMERA_PIN_D6    34
  #define CAMERA_PIN_D5    39
  #define CAMERA_PIN_D4    36
  #define CAMERA_PIN_D3    21
  #define CAMERA_PIN_D2    19
  #define CAMERA_PIN_D1    18
  #define CAMERA_PIN_D0    5
  #define CAMERA_PIN_VSYNC 25
  #define CAMERA_PIN_HREF  23
  #define CAMERA_PIN_PCLK  22

#elif CONFIG_CAMERA_MODULE_CUSTOM
  #define CAMERA_MODULE_NAME "CUSTOM"
  #define CAMERA_PIN_PWDN  CONFIG_CAMERA_PIN_PWDN
  #define CAMERA_PIN_RESET CONFIG_CAMERA_PIN_RESET
  #define CAMERA_PIN_XCLK  CONFIG_CAMERA_PIN_XCLK
  #define CAMERA_PIN_SIOD  CONFIG_CAMERA_PIN_SIOD
  #define CAMERA_PIN_SIOC  CONFIG_CAMERA_PIN_SIOC
  #define CAMERA_PIN_D7    CONFIG_CAMERA_PIN_Y9
  #define CAMERA_PIN_D6    CONFIG_CAMERA_PIN_Y8
  #define CAMERA_PIN_D5    CONFIG_CAMERA_PIN_Y7
  #define CAMERA_PIN_D4    CONFIG_CAMERA_PIN_Y6
  #define CAMERA_PIN_D3    CONFIG_CAMERA_PIN_Y5
  #define CAMERA_PIN_D2    CONFIG_CAMERA_PIN_Y4
  #define CAMERA_PIN_D1    CONFIG_CAMERA_PIN_Y3
  #define CAMERA_PIN_D0    CONFIG_CAMERA_PIN_Y2
  #define CAMERA_PIN_VSYNC CONFIG_CAMERA_PIN_VSYNC
  #define CAMERA_PIN_HREF  CONFIG_CAMERA_PIN_HREF
  #define CAMERA_PIN_PCLK  CONFIG_CAMERA_PIN_PCLK
#endif

/* -----------------------------------------------------------------------------
   Runtime camera profiles (types should live outside extern "C")
----------------------------------------------------------------------------- */
typedef enum {
  APP_CAM_PROFILE_INVALID           = -1, // sentinel for "no camera active"
  APP_CAM_PROFILE_DETECT_GRAY_96    = 0,  // 96x96 GRAYSCALE (tiny detect)
  APP_CAM_PROFILE_RECORD_JPEG_VGA        // VGA JPEG (recording)
} app_cam_profile_t;

/* -----------------------------------------------------------------------------
   C-linkage prototypes (this is key for C++ callers)
----------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the camera using the active pin map and the DETECT profile. */
esp_err_t app_camera_init(void);

/** Switch camera parameters at runtime (no driver re-init). */
esp_err_t app_camera_apply_profile(app_cam_profile_t profile);

/** Optional: raw deinit */
void      app_camera_deinit(void);

/** Safe deinit that keeps internal init-state in sync. */
esp_err_t app_camera_deinit_safe(void);

void app_camera_diag_dump(const char* where);

bool app_camera_acquire_gray160x120(camera_fb_t **out_fb, int max_tries);

/** Force a clean reinit into DETECT profile (used after repeated timeouts). */
esp_err_t app_camera_kick_detect(void);


#ifdef __cplusplus
} // extern "C"
#endif

#endif  // TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_
