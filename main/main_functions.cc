// main_functions.cc

#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <algorithm>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_camera_esp.h"   // app_camera_* APIs
#include "esp_camera.h"

#include "image_provider.h"
#include "detection_responder.h"
#include "model_settings.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ------- Resolver selection (no extra header unless you enable AllOps) -------
#ifndef TFLM_USE_ALLOPS
#define TFLM_USE_ALLOPS 0
#endif

#if TFLM_USE_ALLOPS
  #include "tensorflow/lite/micro/all_ops_resolver.h"
#else
  #include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#endif
// -----------------------------------------------------------------------------

extern "C" {
  extern const unsigned char g_person_detect_model_data[];
  extern const int g_person_detect_model_data_len;

  #include "recorder.h"   // recorder_is_recording()
}

static const char* TAG = "tf";

#ifndef PERSON_TRIG_PCT
#define PERSON_TRIG_PCT    55
#endif
#ifndef PERSON_CONSEC_FRAMES
#define PERSON_CONSEC_FRAMES 2
#endif

static bool s_gate_open = false;
static int  s_gate_consec = 0;

extern "C" bool detector_gate_is_open(void) { return s_gate_open; }
extern "C" void detector_gate_reset(void) { s_gate_open = false; s_gate_consec = 0; }

#ifndef TENSOR_ARENA_BYTES
#define TENSOR_ARENA_BYTES (1024 * 1024)  // 1 MB
#endif

__attribute__((section(".ext_ram.bss"), aligned(16)))
static uint8_t s_tensor_arena[TENSOR_ARENA_BYTES];

static const tflite::Model*      s_model       = nullptr;
static tflite::MicroInterpreter* s_interpreter = nullptr;
static TfLiteTensor*             s_input       = nullptr;
static TfLiteTensor*             s_output      = nullptr;

// ------------------------- Static resolver (important!) ----------------------
#if TFLM_USE_ALLOPS
static tflite::AllOpsResolver s_resolver;
static inline void InitResolver() {
  ESP_LOGI(TAG, "Using AllOpsResolver (static) @ %p", (void*)&s_resolver);
}
#else
static tflite::MicroMutableOpResolver<12> s_resolver;
static inline void InitResolver() {
  ESP_LOGI(TAG, "Using MicroMutableOpResolver (static) @ %p", (void*)&s_resolver);
  auto add = [](TfLiteStatus st, const char* name){
    if (st != kTfLiteOk) ESP_LOGE(TAG, "resolver: Add%s() FAILED", name);
  };
  add(s_resolver.AddConv2D(),           "Conv2D");
  add(s_resolver.AddDepthwiseConv2D(),  "DepthwiseConv2D");
  add(s_resolver.AddMaxPool2D(),        "MaxPool2D");
  add(s_resolver.AddAveragePool2D(),    "AveragePool2D");
  add(s_resolver.AddFullyConnected(),   "FullyConnected");
  add(s_resolver.AddReshape(),          "Reshape");
  add(s_resolver.AddSoftmax(),          "Softmax");
  add(s_resolver.AddQuantize(),         "Quantize");
  add(s_resolver.AddDequantize(),       "Dequantize");
}
#endif
// -----------------------------------------------------------------------------

static inline float score01(const TfLiteTensor* t, int idx) {
  switch ((int)t->type) {
    case kTfLiteInt8:    return (float(t->data.int8[idx]) + 128.0f) / 255.0f;
    case kTfLiteUInt8:   return float(t->data.uint8[idx]) / 255.0f;
    case kTfLiteFloat32: return t->data.f[idx];
    default:             return 0.0f;
  }
}

static void camera_recover_detect(bool hard = false) {
  if (recorder_is_recording()) {
    ESP_LOGI(TAG, "skip camera recover: recorder is active");
    return;
  }
  if (hard) {
    app_camera_deinit_safe();
    vTaskDelay(pdMS_TO_TICKS(50));
    (void)app_camera_init();
    vTaskDelay(pdMS_TO_TICKS(40));
  }
  (void)app_camera_apply_profile(APP_CAM_PROFILE_DETECT_GRAY_96);
  vTaskDelay(pdMS_TO_TICKS(30));
  if (camera_fb_t* fb = esp_camera_fb_get()) esp_camera_fb_return(fb);
}

extern "C" void setup() {
  esp_log_level_set("app_camera",     ESP_LOG_INFO);
  esp_log_level_set("image_provider", ESP_LOG_INFO);
  esp_log_level_set("tf",             ESP_LOG_INFO);

#ifndef TFLM_SKIP_CAMERA_INIT
  esp_err_t cam_err = app_camera_init();
  if (cam_err != ESP_OK && cam_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "app_camera_init failed: 0x%x", cam_err);
  }
  camera_recover_detect(/*hard=*/false);
#else
  ESP_LOGI(TAG, "TFLM_SKIP_CAMERA_INIT=1 — assuming camera was initialized at boot");
#endif

  ESP_LOGI(TAG, "arena @ %p (%d bytes) [PSRAM]", s_tensor_arena, (int)sizeof(s_tensor_arena));

  s_model = tflite::GetModel(g_person_detect_model_data);
  if (!s_model) { ESP_LOGE(TAG, "GetModel() returned null"); return; }
  if (s_model->version() != TFLITE_SCHEMA_VERSION) {
    ESP_LOGE(TAG, "Model schema %d != runtime %d", (int)s_model->version(), (int)TFLITE_SCHEMA_VERSION);
    return;
  }

  InitResolver();

  static tflite::MicroInterpreter interp(
      s_model, s_resolver,
      s_tensor_arena, sizeof(s_tensor_arena),
      /*resource_variables=*/nullptr, /*profiler=*/nullptr,
      /*allocate_permanent_buffers_at_init=*/true);
  s_interpreter = &interp;

  ESP_LOGI(TAG, "interpreter @ %p (resolver @ %p)", (void*)s_interpreter, (void*)&s_resolver);

  if (s_interpreter->AllocateTensors() != kTfLiteOk) {
    ESP_LOGE(TAG, "AllocateTensors() failed");
    return;
  }

  if (s_interpreter->arena_used_bytes() > 0) {
    ESP_LOGI(TAG, "arena used ~%d / %d",
             (int)s_interpreter->arena_used_bytes(), (int)sizeof(s_tensor_arena));
  }

  s_input  = s_interpreter->input(0);
  s_output = s_interpreter->output(0);

  if (s_input && s_input->dims && s_input->dims->size >= 4) {
    ESP_LOGI(TAG, "input:  type=%d dims=[%d,%d,%d,%d] bytes=%d ptr=%p",
             (int)s_input->type,
             (int)s_input->dims->data[0], (int)s_input->dims->data[1],
             (int)s_input->dims->data[2], (int)s_input->dims->data[3],
             (int)s_input->bytes,
             (void*)( (s_input->type==kTfLiteInt8) ? s_input->data.int8 :
                      (s_input->type==kTfLiteUInt8)? s_input->data.uint8 :
                      (void*)s_input->data.f ));
  }
  if (s_output && s_output->dims && s_output->dims->size >= 2) {
    ESP_LOGI(TAG, "output: type=%d dims0=%d dims1=%d",
             (int)s_output->type,
             (int)s_output->dims->data[0], (int)s_output->dims->data[1]);
  }

  // One-time dry-run — also robust to non-int8 input
  if (s_input && s_output) {
    std::memset(s_input->data.raw, 0, s_input->bytes);
    ESP_LOGI(TAG, "dry-run Invoke()");
    TfLiteStatus st = s_interpreter->Invoke();
    if (st != kTfLiteOk) ESP_LOGE(TAG, "dry-run Invoke failed (status=%d)", (int)st);
    else                 ESP_LOGI(TAG, "dry-run OK");
  }

  detector_gate_reset();
}

extern "C" void loop() {
  if (!s_interpreter || !s_input || !s_output) {
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }
  if (recorder_is_recording()) {
    vTaskDelay(pdMS_TO_TICKS(100));
    return;
  }

  static int consecutive_fail = 0;

  const size_t nbytes = (size_t)kNumCols * (size_t)kNumRows * (size_t)kNumChannels;
  static int8_t  scratch[(kNumCols * kNumRows * kNumChannels) + 8];
  uint32_t      *canary = (uint32_t*)(scratch + nbytes);
  canary[0] = 0xDEADBEEF;
  canary[1] = 0xA5A5A5A5;

  ESP_LOGI(TAG,
           "pre-GetImage dst=%p nbytes=%u tensor.bytes=%d tensor.ptr(int8)=%p",
           (void*)scratch, (unsigned)nbytes,
           s_input ? (int)s_input->bytes : -1,
           (s_input && s_input->type == kTfLiteInt8) ? (void*)s_input->data.int8 : (void*)nullptr);

  if (GetImage(kNumCols, kNumRows, kNumChannels, scratch) != kTfLiteOk) {
    consecutive_fail++;
    ESP_LOGW(TAG, "GetImage failed (%d)", consecutive_fail);
    if (consecutive_fail >= 2) {
      ESP_LOGW(TAG, "Soft camera recover after consecutive GetImage fails");
      camera_recover_detect(/*hard=*/false);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    return;
  }
  consecutive_fail = 0;
  ESP_LOGI(TAG, "post-GetImage ok");

  if (canary[0] != 0xDEADBEEF || canary[1] != 0xA5A5A5A5) {
    ESP_LOGE(TAG, "OVERFLOW: GetImage() wrote past %u bytes!", (unsigned)nbytes);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  const int model_h = (s_input->dims && s_input->dims->size >= 4) ? s_input->dims->data[1] : -1;
  const int model_w = (s_input->dims && s_input->dims->size >= 4) ? s_input->dims->data[2] : -1;
  const int model_c = (s_input->dims && s_input->dims->size >= 4) ? s_input->dims->data[3] : -1;

  const int wanted_h = kNumRows;
  const int wanted_w = kNumCols;
  const int wanted_c = kNumChannels;

  const size_t elem_size =
      (s_input->type == kTfLiteInt8)    ? sizeof(int8_t)  :
      (s_input->type == kTfLiteUInt8)   ? sizeof(uint8_t) :
      (s_input->type == kTfLiteFloat32) ? sizeof(float)   : 0;

  if (elem_size == 0) {
    ESP_LOGE(TAG, "Unsupported input type: %d", (int)s_input->type);
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }

  const size_t tensor_bytes = (size_t)s_input->bytes;
  const size_t tensor_elems = tensor_bytes / elem_size;
  const size_t wanted_elems = (size_t)wanted_w * (size_t)wanted_h * (size_t)wanted_c;
  const size_t copy_elems   = (wanted_elems < tensor_elems) ? wanted_elems : tensor_elems;

  if (model_w != wanted_w || model_h != wanted_h || model_c != wanted_c) {
    ESP_LOGW(TAG,
             "Model/wanted dims differ: model=%dx%dx%d wanted=%dx%dx%d "
             "(tensor_bytes=%u elem_size=%u tensor_elems=%u wanted_elems=%u)",
             model_w, model_h, model_c, wanted_w, wanted_h, wanted_c,
             (unsigned)tensor_bytes, (unsigned)elem_size,
             (unsigned)tensor_elems, (unsigned)wanted_elems);
  }

  if (copy_elems == 0) {
    ESP_LOGE(TAG, "Input tensor has zero capacity (bytes=%u type=%d)",
             (unsigned)tensor_bytes, (int)s_input->type);
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }

  switch (s_input->type) {
    case kTfLiteInt8:
      std::memcpy(s_input->data.int8, scratch, copy_elems * sizeof(int8_t));
      break;
    case kTfLiteUInt8: {
      uint8_t* dst = s_input->data.uint8;
      for (size_t i = 0; i < copy_elems; ++i) {
        int v = (int)scratch[i] + 128;
        dst[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
      }
      break;
    }
    case kTfLiteFloat32: {
      float* dst = s_input->data.f;
      for (size_t i = 0; i < copy_elems; ++i) {
        dst[i] = (float(((int)scratch[i]) + 128) / 255.0f);
      }
      break;
    }
    default:
      ESP_LOGE(TAG, "Unsupported input type (late): %d", (int)s_input->type);
      vTaskDelay(pdMS_TO_TICKS(50));
      return;
  }

  if (copy_elems < wanted_elems) {
    ESP_LOGW(TAG, "Truncated input: wrote %u of %u elements (tensor had %u)",
             (unsigned)copy_elems, (unsigned)wanted_elems, (unsigned)tensor_elems);
  }

  ESP_LOGI(TAG, "about to Invoke()");
  TfLiteStatus st = s_interpreter->Invoke();
  if (st != kTfLiteOk) {
    ESP_LOGE(TAG, "Invoke failed (status=%d)", (int)st);
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }
  ESP_LOGI(TAG, "Invoke OK");

  if (!s_output || !s_output->dims || s_output->dims->size < 2 || s_output->dims->data[1] < 2) {
    ESP_LOGE(TAG, "Bad output tensor shape/type");
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }

  const float no_person = score01(s_output, 0);
  const float person    = score01(s_output, 1);
  ESP_LOGI(TAG, "scores: person=%.1f%% no_person=%.1f%%", person*100.0f, no_person*100.0f);

  const float trig_pct_f = (float)PERSON_TRIG_PCT;
  const bool over = (person * 100.0f) >= trig_pct_f;

  if (over) {
    s_gate_consec = std::min(s_gate_consec + 1, 1000000);
    if (!s_gate_open && s_gate_consec >= PERSON_CONSEC_FRAMES) {
      s_gate_open = true;
      ESP_LOGI(TAG, "GATE OPEN (person=%.1f%% >= %.0f%% for %d frames)",
               person * 100.0f, trig_pct_f, PERSON_CONSEC_FRAMES);
    }
  } else {
    s_gate_consec = 0;
  }

  RespondToDetection(person, no_person);
  vTaskDelay(pdMS_TO_TICKS(10));
}
