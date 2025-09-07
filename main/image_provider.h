// image_provider.h
#pragma once
#include <cstdint>
#include "tensorflow/lite/c/common.h"  // TfLiteStatus, kTfLiteOk/kTfLiteError

// Writes HWC (row-major) grayscale bytes (CH=1) into `out` as int8 zero-centered (-128..127).
// Returns kTfLiteOk on success, kTfLiteError on failure (e.g., no frame).
TfLiteStatus GetImage(int wanted_w, int wanted_h, int wanted_ch, int8_t* out);
