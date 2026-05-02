#pragma once

void init_hc06();
void parse_joystick_data(char* data);
void hc06_event_task(void *pvParameters);
void data_tx();