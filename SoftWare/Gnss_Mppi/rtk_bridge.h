#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#define UM982_UART_PORT UART_NUM_2
#define UM982_TX_PIN    17
#define UM982_RX_PIN    18
#define UM982_BAUD_RATE 115200

typedef struct {
    bool valid;
    float latitude;
    float longitude;
    uint8_t quality;
    float differential_age;
    uint32_t received_ms;
} RTK_Bridge_GGA;

esp_err_t RTK_Bridge_Init(void);
bool RTK_Bridge_ForwardRTCM(const uint8_t *frame, uint16_t length);
void Debug_Log(const char *fmt, ...);

uint32_t RTK_Bridge_GetForwardedFrames(void);
uint32_t RTK_Bridge_GetForwardedBytes(void);
uint32_t RTK_Bridge_GetDroppedFrames(void);
uint32_t RTK_Bridge_GetUM982Bytes(void);
uint32_t RTK_Bridge_GetUM982Lines(void);
uint32_t RTK_Bridge_GetGGALines(void);
uint32_t RTK_Bridge_GetNMEAOverflows(void);
uint32_t RTK_Bridge_GetUM982RxOverflowBytes(void);
uint8_t RTK_Bridge_GetGGAQuality(void);
bool RTK_Bridge_HasRTK(void);
uint32_t RTK_Bridge_GetGGAAgeMs(void);
bool RTK_Bridge_GetLatestGGA(RTK_Bridge_GGA *gga_out);

bool RTK_Bridge_GetHeadingRad(float *heading_rad_out);
float RTK_Bridge_GetHeadingDeg(void);
bool RTK_Bridge_HasHeading(void);
uint32_t RTK_Bridge_GetHeadingAgeMs(void);

bool RTK_Bridge_HasFreshRTK(uint32_t max_age_ms);
bool RTK_Bridge_HasFreshHeading(uint32_t max_age_ms);
