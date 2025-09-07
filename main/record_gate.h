// main/record_gate.h
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void rec_gate_reset(void);
bool rec_gate_should_record(void);
void rec_gate_note_detection(bool is_person);
void rec_gate_note_person_score(float score_0_to_1, float threshold_0_to_1);

#ifdef __cplusplus
}
#endif
