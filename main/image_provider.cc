// Captures a frame from esp_camera and resizes to wanted_w x wanted_h,
// producing int8 zero-centered grayscale (-128..127) for TFLM.
#include <cstring>
#include "esp_log.h"
#include "esp_camera.h"
#include "app_camera_esp.h"       // <- our wrapper with app_camera_acquire_gray160x120()
#include "image_provider.h"
#include "model_settings.h"
#include "tensorflow/lite/c/common.h"

static const char* IP_TAG = "image_provider";

// Center-crop 160x120 -> 96x96 and zero-center to int8
static inline void crop160x120_to_96x96_int8(const uint8_t* src, int8_t* dst) {
  const int x0 = (160 - 96) / 2;  // 32
  const int y0 = (120 - 96) / 2;  // 12
  int di = 0;
  for (int y = 0; y < 96; ++y) {
    const uint8_t* row = src + (y0 + y) * 160 + x0;
    for (int x = 0; x < 96; ++x) {
      dst[di++] = static_cast<int8_t>(int(row[x]) - 128);
    }
  }
}

TfLiteStatus GetImage(int wanted_w, int wanted_h, int wanted_ch, int8_t* out) {
  if (wanted_ch != 1 || out == nullptr) {
    ESP_LOGE(IP_TAG, "[ACQ] bad args wanted=%dx%dx%d out=%p",
             wanted_w, wanted_h, wanted_ch, (void*)out);
    return kTfLiteError;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGW(IP_TAG, "[ACQ] fb_get() returned NULL");
    return kTfLiteError;
  }

  // Always give the buffer back before returning from this function.
  auto defer_return = [&]() { esp_camera_fb_return(fb); };

  // Accept only grayscale frames
  if (fb->format != PIXFORMAT_GRAYSCALE) {
    ESP_LOGW(IP_TAG, "[ACQ] unexpected pf=%d (need GRAYSCALE=3) %dx%d len=%u",
             (int)fb->format, fb->width, fb->height, (unsigned)fb->len);
    defer_return();
    return kTfLiteError;
  }

  const bool ok160 = (fb->width == 160 && fb->height == 120 && fb->len == 160 * 120);
  const bool ok96  = (fb->width ==  96 && fb->height ==  96 && fb->len ==  96 *  96);

  if (!(ok160 || ok96)) {
    ESP_LOGW(IP_TAG, "[ACQ] unexpected size %dx%d len=%u (want 160x120 or 96x96 GRAY)",
             fb->width, fb->height, (unsigned)fb->len);
    defer_return();
    return kTfLiteError;
  }

  if (wanted_w == 96 && wanted_h == 96) {
    if (ok96) {
      // Just convert 0..255 -> -128..127
      for (int i = 0; i < 96 * 96; ++i) out[i] = static_cast<int8_t>(int(fb->buf[i]) - 128);
      ESP_LOGI(IP_TAG, "acq ok %dx%d len=%u (no crop)", fb->width, fb->height, (unsigned)fb->len);
      defer_return();
      return kTfLiteOk;
    } else { // ok160
      crop160x120_to_96x96_int8(fb->buf, out);
      ESP_LOGI(IP_TAG, "acq ok %dx%d len=%u (cropped->96x96)",
               fb->width, fb->height, (unsigned)fb->len);
      defer_return();
      return kTfLiteOk;
    }
  }

  // Generic path: nearest-neighbor resample to any requested WxH (still GRAY)
  const int src_w = fb->width, src_h = fb->height;
  const uint8_t* src = fb->buf;
  for (int y = 0; y < wanted_h; ++y) {
    int sy = (y * src_h) / wanted_h;
    const uint8_t* row = src + sy * src_w;
    for (int x = 0; x < wanted_w; ++x) {
      int sx = (x * src_w) / wanted_w;
      out[y * wanted_w + x] = static_cast<int8_t>(int(row[sx]) - 128);
    }
  }
  ESP_LOGI(IP_TAG, "acq ok %dx%d len=%u (resampled->%dx%d)",
           fb->width, fb->height, (unsigned)fb->len, wanted_w, wanted_h);
  defer_return();
  return kTfLiteOk;
}
