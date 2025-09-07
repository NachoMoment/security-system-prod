// main/record_gate.c
#include "esp_log.h"
#include <stdbool.h>
#include <math.h>

static const char* TAG = "rec_gate";

static bool  s_allow  = false;
static int   s_consec = 0;

#ifndef GATE_CONSEC
#define GATE_CONSEC 2
#endif

static inline int pct01(float x) {
  if (x < 0.f) x = 0.f; else if (x > 1.f) x = 1.f;
  return (int)(x * 100.f + 0.5f);
}

void rec_gate_reset(void) {
  s_allow = false;
  s_consec = 0;
  ESP_LOGI(TAG, "RESET -> allow=%d, consec=%d", s_allow, s_consec);
}

void rec_gate_note_person_score(float score_0_to_1, float threshold_0_to_1) {
  const int score_pct = pct01(score_0_to_1);
  const int thr_pct   = pct01(threshold_0_to_1);

  if (score_0_to_1 >= threshold_0_to_1) {
    if (!s_allow) s_consec++;
    if (!s_allow && s_consec >= GATE_CONSEC) {
      s_allow = true;
      ESP_LOGI(TAG, "ARM: score=%d%% >= thr=%d%% for %d frames -> allow=1",
               score_pct, thr_pct, GATE_CONSEC);
    } else {
      ESP_LOGD(TAG, "UP: score=%d%%/%d%% consec=%d allow=%d",
               score_pct, thr_pct, s_consec, s_allow);
    }
  } else {
    if (!s_allow && s_consec != 0) {
      ESP_LOGD(TAG, "DROP: score=%d%% < thr=%d%% -> consec=0", score_pct, thr_pct);
    }
    if (!s_allow) s_consec = 0;
  }
}

bool rec_gate_should_record(void) {
  ESP_LOGD(TAG, "QUERY allow=%d consec=%d", s_allow, s_consec);
  return s_allow;
}
