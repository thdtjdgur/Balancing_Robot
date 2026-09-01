#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#define UM982_UART_PORT UART_NUM_2
#define UM982_TX_PIN    17
#define UM982_RX_PIN    18
#define UM982_BAUD_RATE 115200

esp_err_t RTK_Bridge_Init(void);
bool RTK_Bridge_ForwardRTCM(const uint8_t *frame, uint16_t length);
void Debug_Log(const char *fmt, ...);

uint32_t RTK_Bridge_GetForwardedFrames(void);
uint32_t RTK_Bridge_GetForwardedBytes(void);
uint32_t RTK_Bridge_GetDroppedFrames(void);
uint32_t RTK_Bridge_GetUM982Bytes(void);
uint32_t RTK_Bridge_GetUM982RxOverflowBytes(void);
uint8_t RTK_Bridge_GetGGAQuality(void);
bool RTK_Bridge_HasRTK(void);
