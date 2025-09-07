// main/detection_responder.cc
#include <algorithm>
#include "esp_log.h"
#include "detection_responder.h"

extern "C" {
  #include "recorder.h"
  #include "record_gate.h"
}

static const char* TAG = "detect";

#ifndef PERSON_TRIG_PCT
#define PERSON_TRIG_PCT 55
#endif
#ifndef PERSON_STOP_PCT
#define PERSON_STOP_PCT 20
#endif

static inline int pct01(float x) {
  if (x < 0.f) x = 0.f; else if (x > 1.f) x = 1.f;
  return int(x * 100.f + 0.5f);
}

extern "C" void RespondToDetection(float person_score, float no_person_score) {
  const int person_pct    = pct01(person_score);
  const int no_person_pct = pct01(no_person_score);

  // Always log raw scores so we can match every frame with the gate state.
  ESP_LOGI(TAG, "scores: person=%d%% no=%d%% (thr=%d%%) rec=%d",
           person_pct, no_person_pct, PERSON_TRIG_PCT, recorder_is_recording());

  // Tell the gate the latest score with the start threshold.
  rec_gate_note_person_score(person_score, PERSON_TRIG_PCT / 100.0f);

  const bool allow = rec_gate_should_record();

  // START logic: only start when allowed and not already recording
  if (allow && !recorder_is_recording()) {
    ESP_LOGI(TAG, "gate=ALLOW -> recorder_start()");
    recorder_start();
    return;  // nothing else to do this frame
  }

  // STOP logic: if recording and score fell well below stop threshold
  if (recorder_is_recording() && person_pct <= PERSON_STOP_PCT) {
    ESP_LOGI(TAG, "low-score stop: person=%d%% <= stop=%d%% -> recorder_stop()",
             person_pct, PERSON_STOP_PCT);
    recorder_stop();
    return;
  }

  // Idle debug line (comment out if chatty)
  ESP_LOGD(TAG, "gate=%s recorder=%d",
           allow ? "ALLOW" : "HOLD", recorder_is_recording());
}
