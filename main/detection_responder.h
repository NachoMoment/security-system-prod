#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// Call on every inference
void RespondToDetection(float person_score, float no_person_score);

#ifdef __cplusplus
}
#endif
