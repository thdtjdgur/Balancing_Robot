#pragma once

void init_rx28();
void rx28_task(void *pvParameters);
void get_balanced_angles(float target_H, int16_t *out_hip, int16_t *out_knee);