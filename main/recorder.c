// main/recorder.c — SDMMC (1-bit) on ESP32-S3-CAM with on-mount self-test write.
// Board pins: CLK=GPIO39, CMD=GPIO38, D0=GPIO40.

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <sys/unistd.h>
#include <sys/stat.h>   // for mkdir

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "driver/gpio.h"
#include "esp_camera.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "recorder.h"
#include "record_gate.h"
#include "app_camera_esp.h"

#include "esp_timer.h"

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t)-1)
#endif

static const char *TAG = "recorder";

static const char *SD_MOUNT_PT = "/sdcard";

#define SDMMC_CLK_GPIO  GPIO_NUM_39
#define SDMMC_CMD_GPIO  GPIO_NUM_38
#define SDMMC_D0_GPIO   GPIO_NUM_40

static sdmmc_card_t *s_card   = NULL;
static bool          s_sd_ok  = false;
static atomic_bool   s_recording = false;

#ifndef RECORD_COOLDOWN_MS
#define RECORD_COOLDOWN_MS 9000   // 9s tweak as needed
#endif

static int64_t s_last_record_start_us = 0;

static inline int64_t now_ms(void) {
  return esp_timer_get_time() / 1000;  // monotonic ms since boot
}

static void log_heap(const char* where) {
  multi_heap_info_t a={0}, b={0};
  heap_caps_get_info(&a, MALLOC_CAP_INTERNAL);
  heap_caps_get_info(&b, MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "[HEAP] %s int=%uB(lrg=%u)  psram=%uB(lrg=%u)",
           where?where:"?", (unsigned)a.total_free_bytes,(unsigned)a.largest_free_block,
           (unsigned)b.total_free_bytes,(unsigned)b.largest_free_block);
}

static void ensure_rec_dir(void) {
  char dir[32];
  snprintf(dir, sizeof(dir), "%s/REC", SD_MOUNT_PT);
  int rc = mkdir(dir, 0777);
  if (rc < 0 && errno != EEXIST) {
    ESP_LOGW(TAG, "mkdir('%s') failed errno=%d (ignoring)", dir, errno);
  }
}


static void sd_pullups(void) {
  // SD spec: CMD/DAT need PU (CLK must NOT be pulled up)
  gpio_set_pull_mode(SDMMC_CMD_GPIO, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(SDMMC_D0_GPIO,  GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(SDMMC_CLK_GPIO, GPIO_FLOATING);
  vTaskDelay(pdMS_TO_TICKS(5));
}

// Query FATFS capacity on /sdcard without statvfs
static esp_err_t sdcard_query_space(uint64_t *total_bytes, uint64_t *free_bytes)
{
  if (!total_bytes || !free_bytes) return ESP_ERR_INVALID_ARG;
  // base_path must match what you passed to esp_vfs_fat_sdmmc_mount()
  return esp_vfs_fat_info(SD_MOUNT_PT, total_bytes, free_bytes);
}

// Try a tiny write immediately after mounting, to prove the FS is usable.
static esp_err_t sd_self_test_write(void) {
  // Also prints capacity (useful to confirm FAT32 and LFN are working)
  uint64_t tot=0, free=0;
  esp_err_t q = sdcard_query_space(&tot, &free);
  if (q == ESP_OK) {
    ESP_LOGI(TAG, "SD: total=%" PRIu64 "B  free=%" PRIu64 "B (%.1f%%)",
             tot, free, tot ? (100.0*(double)free/(double)tot) : 0.0);
  } else {
    ESP_LOGW(TAG, "SD: esp_vfs_fat_info failed: %s", esp_err_to_name(q));
  }

  char test_path[64];
  // keep filename 8.3-friendly just in case LFN is off
  snprintf(test_path, sizeof(test_path), "%s/PROBE.TXT", SD_MOUNT_PT);

  FILE *f = fopen(test_path, "wb");
  if (!f) {
    ESP_LOGE(TAG, "SD SELF-TEST: fopen('%s') errno=%d", test_path, errno);
    return ESP_FAIL;
  }
  const char msg[] = "ok\n";
  size_t w = fwrite(msg, 1, sizeof msg, f);
  fflush(f);
  fsync(fileno(f));
  fclose(f);
  if (w != sizeof msg) {
    ESP_LOGE(TAG, "SD SELF-TEST: short write %u/%u", (unsigned)w, (unsigned)sizeof msg);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "SD SELF-TEST: wrote %u bytes -> %s", (unsigned)w, test_path);
  return ESP_OK;
}

static esp_err_t try_mount_sdmmc(void) {
  sd_pullups();

  // Mount config
  esp_vfs_fat_sdmmc_mount_config_t mcfg = {
    .format_if_mount_failed   = false,   // don’t auto-format
    .max_files                = 8,
    .allocation_unit_size     = 0,
    .disk_status_check_enable = true,
  };

  // Host (400 kHz init; then limit run speed for signal margin)
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = 4000;             // conservative post-init clock

  // Slot/pins (1-bit mode on S3)
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk   = SDMMC_CLK_GPIO;
  slot.cmd   = SDMMC_CMD_GPIO;
  slot.d0    = SDMMC_D0_GPIO;
  slot.d1    = GPIO_NUM_NC;
  slot.d2    = GPIO_NUM_NC;
  slot.d3    = GPIO_NUM_NC;

  ESP_LOGI(TAG, "SD: mounting at %s (SDMMC 1-bit, CLK=%d CMD=%d D0=%d) ...",
           SD_MOUNT_PT, SDMMC_CLK_GPIO, SDMMC_CMD_GPIO, SDMMC_D0_GPIO);

  esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_PT, &host, &slot, &mcfg, &s_card);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "SD: mount failed: %s (0x%x)", esp_err_to_name(ret), ret);
    s_sd_ok = false;
    return ret;
  }

  s_sd_ok = true;
  ESP_LOGI(TAG, "SD: mount OK");
  sdmmc_card_print_info(stdout, s_card);

  // Immediate sanity check write
  esp_err_t probe = sd_self_test_write();
  if (probe != ESP_OK) {
    ESP_LOGE(TAG, "SD: self-test write FAILED — unmounting (likely exFAT/GPT or bad media).");
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_PT, s_card);
    s_card = NULL;
    s_sd_ok = false;
    return ESP_FAIL;
  } else {
    ESP_LOGI(TAG, "SD: self-test OK — filesystem is writable.");
  }
  return ESP_OK;
}

static esp_err_t ensure_sd(void) {
  if (s_sd_ok) return ESP_OK;
  return try_mount_sdmmc();
}

static esp_err_t write_file_bytes(const char* path, const void* data, size_t len) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    ESP_LOGE(TAG, "fopen('%s') errno=%d", path, errno);
    return ESP_FAIL;
  }
  size_t w = fwrite(data, 1, len, f);
  fflush(f);
  fsync(fileno(f));
  fclose(f);
  if (w != len) {
    ESP_LOGE(TAG, "short write %u/%u -> %s", (unsigned)w, (unsigned)len, path);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "WROTE %u bytes -> %s", (unsigned)w, path);
  return ESP_OK;
}

static esp_err_t save_jpeg(const uint8_t* jpg, size_t len) {
  if (ensure_sd() != ESP_OK) {
    ESP_LOGW(TAG, "save_jpeg: SD not mounted; aborting write");
    return ESP_FAIL;
  }
  ensure_rec_dir();

  // 8.3 filename: IMGnnnnnnn.JPG (7 digits -> total 10.3 incl. extension)
  unsigned long ms = (unsigned long)(esp_timer_get_time() / 1000ULL);
  unsigned long id = ms % 10000000UL; // 0..9,999,999 (7 digits)

  char path[64];
  snprintf(path, sizeof(path), "%s/REC/IMG%07lu.JPG", SD_MOUNT_PT, id);

  // avoid accidental collision: bump once if it already exists
  FILE *t = fopen(path, "rb");
  if (t) { fclose(t); id = (id + 1) % 10000000UL; 
           snprintf(path, sizeof(path), "%s/REC/IMG%07lu.JPG", SD_MOUNT_PT, id); }

  return write_file_bytes(path, jpg, len);
}


// -------------- public API ----------------

esp_err_t recorder_init(void) {
  ESP_LOGI(TAG, "recorder_init()");
  log_heap("before SD probe");
  (void)ensure_sd();                 // try once at boot
  if (!s_sd_ok)
    ESP_LOGW(TAG, "SD NOT mounted at %s; will retry on recorder_start()", SD_MOUNT_PT);
  log_heap("after SD probe");
  return ESP_OK;
}

bool recorder_is_recording(void) { return atomic_load(&s_recording); }

esp_err_t recorder_start(void) {
  // Already recording? no-op
  if (atomic_exchange(&s_recording, true)) return ESP_OK;

  // Cooldown check (monotonic time)
  const int64_t now = now_ms();
  const int64_t last = s_last_record_start_us;
  const int64_t elapsed = (last > 0) ? (now - last) : INT64_MAX;

  if (last > 0 && elapsed < RECORD_COOLDOWN_MS) {
    const int64_t remain = RECORD_COOLDOWN_MS - elapsed;
    ESP_LOGI(TAG, "recorder_start() suppressed by cooldown: %lld ms remaining",
             (long long)remain);
    atomic_store(&s_recording, false);   // release the recording flag since we didn't start
    return ESP_OK;                       // not an error; intentional suppression
  }

  // Accept this start attempt and begin cooldown window now
  s_last_record_start_us = now;

  ESP_LOGI(TAG, "recorder_start() (cooldown passed)");
  rec_gate_reset();

  (void)ensure_sd();                 // retry mount every time we start

  // Switch camera to JPEG VGA profile and prime it
  esp_err_t e = app_camera_apply_profile(APP_CAM_PROFILE_RECORD_JPEG_VGA);
  ESP_LOGI(TAG, "apply_profile(RECORD_JPEG_VGA) -> %s", esp_err_to_name(e));
  if (e != ESP_OK) { atomic_store(&s_recording, false); return e; }

  for (int i = 0; i < 2; ++i) {
    camera_fb_t *p = esp_camera_fb_get(); if (p) esp_camera_fb_return(p);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Grab one JPEG and save
  camera_fb_t *fb = NULL;
  const int kMaxTries = 5; const size_t kMinJpegSz = 1024;
  for (int t = 1; t <= kMaxTries; ++t) {
    fb = esp_camera_fb_get();
    if (!fb) { ESP_LOGW(TAG, "grab try %d/%d: fb=NULL", t, kMaxTries); vTaskDelay(pdMS_TO_TICKS(30)); continue; }
    ESP_LOGI(TAG, "grab try %d/%d: fmt=%d w=%d h=%d len=%u",
             t, kMaxTries, (int)fb->format, fb->width, fb->height, (unsigned)fb->len);
    if (fb->format == PIXFORMAT_JPEG && fb->len >= kMinJpegSz) break;
    esp_camera_fb_return(fb); fb = NULL; vTaskDelay(pdMS_TO_TICKS(20));
  }

  esp_err_t werr = ESP_FAIL;
  if (fb && fb->format == PIXFORMAT_JPEG) {
    werr = save_jpeg(fb->buf, fb->len);
  } else {
    ESP_LOGW(TAG, "no valid JPEG frame to save");
  }
  if (fb) esp_camera_fb_return(fb);

  (void)app_camera_apply_profile(APP_CAM_PROFILE_DETECT_GRAY_96);
  atomic_store(&s_recording, false);
  return werr;
}

esp_err_t recorder_stop(void) {
  if (!atomic_exchange(&s_recording, false)) return ESP_OK;
  ESP_LOGI(TAG, "recorder_stop()");
  rec_gate_reset();
  return ESP_OK;
}

void recorder_sd_remount(void) {
  ESP_LOGW(TAG, "recorder_sd_remount(): will retry SD on next start");
  s_sd_ok = false;
}
