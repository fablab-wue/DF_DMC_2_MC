#pragma once

// Board limits, pin map, and DMC/SliderMC unit scale.
// USB CDC is binary DMC only — do not print debug text on Serial.

#include <Arduino.h>
#include <cmath>
#include <cstdint>

#ifndef SLIDERDMC_STATUS_LED_MODE
#define SLIDERDMC_STATUS_LED_MODE 1
#endif

namespace sliderdmc {

#ifdef SIMULATE
constexpr bool kSimulateMc = true;
#else
constexpr bool kSimulateMc = false;
#endif

constexpr bool kUseClassicLed = (SLIDERDMC_STATUS_LED_MODE == 0);
constexpr bool kUseNeoPixel = (SLIDERDMC_STATUS_LED_MODE == 1);

constexpr int kMaxAxes = 6;
constexpr int kMaxUploadFrames = 2048;
constexpr int kMaxPathSamples = 4096;
constexpr int kDmxChannels = 512;

// DMC wire has no unit field. Tell DragonFrame Arc: steps per unit = 1000.
// 1000 DMC steps = 1 mm (linear) or 1 deg (servo); same scale for speed/accel.
constexpr float kDmcStepsPerUnit = 1000.0f;

constexpr uint8_t kMcUartTxPin = 12;  // UART TX to SliderMC
constexpr uint8_t kMcUartRxPin = 13;  // UART RX from SliderMC
constexpr uint32_t kMcUartBaud = 115200;
constexpr uint32_t kMcStartupTimeoutMs = 2000;
constexpr uint32_t kMcConfigIdleMs = 50;

constexpr uint8_t kStatusLedPin = LED_BUILTIN;
constexpr uint8_t kNeoPixelPin = 16;  // WS2812 on RP2040-Zero
constexpr uint8_t kNeoPixelBrightness = 32;
constexpr uint32_t kStatusLedHeartbeatMs = 500;
constexpr uint32_t kStatusLedFastBlinkMs = 100;
constexpr uint32_t kPositionReportMs = 100;

constexpr uint8_t kGioOutPins[4] = {1, 2, 3, 4};  // DMC GIO OUT bits 0–3 (OC + pull-up)
constexpr uint8_t kGioInPins[4] = {5, 6, 7, 8};   // DMC GIO IN bits 0–3 (pull-up)
constexpr uint8_t kCameraShutterPin = 9;          // shutter OC + pull-up (with MC CT)
constexpr uint8_t kBuzzerPin = 10;                // preroll/bloop (with MC BE)
constexpr uint8_t kMovePin = 11;                  // MOVE: high while verbose status is moving
constexpr uint8_t kDmxTxPin = 0;                  // DMX512 TX (PIO UART; GP0 is UART0, same as MC Serial1)
constexpr uint8_t kDmxPwmPins[6] = {29, 28, 27, 26, 15, 14};  // DMX1..DMX6, channels 1–6
constexpr uint32_t kDmxPwmHz = 18000;

constexpr uint8_t kHelloVersionMajor = 1;
constexpr uint8_t kHelloVersionMinor = 2;
constexpr uint8_t kHelloVersionRev = 3;
constexpr uint16_t kHelloProtocolVersion = 2;

inline float dmcStepsToMc(int32_t steps) { return static_cast<float>(steps) / kDmcStepsPerUnit; }

inline int32_t mcToDmcSteps(float units) { return static_cast<int32_t>(lroundf(units * kDmcStepsPerUnit)); }

}  // namespace sliderdmc
