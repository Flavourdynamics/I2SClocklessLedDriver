#pragma once

#include <stddef.h>
#include <stdint.h>

#include "colorarrangement.h"

/**
 * ShiftRegisterClocklessLedDriver
 *
 * ESP32-P4 PARLIO driver for expanding WS2812 data outputs through a
 * 74HC595-style serial-in/parallel-out shift register.
 *
 * Wiring:
 *   SER   -> shift-register serial data input
 *   SRCLK -> shift-register shift clock
 *   RCLK  -> shift-register storage/latch clock
 *
 * One SER pin gives 8 independent WS2812 data lanes. Multiple SER pins can
 * share duplicated SRCLK/RCLK pins; for example 10 SER + 2 SRCLK + 2 RCLK
 * uses 14 PARLIO data bits and expands to 80 independent lanes. The external
 * LED buffer layout matches I2SClocklessLedDriver: strips are stored
 * sequentially, and each pixel is raw RGB/RGBW data.
 */
class ShiftRegisterClocklessLedDriver {
 public:
  struct Pins {
    uint8_t ser;
    uint8_t srclk;
    uint8_t rclk;
  };

  struct ParallelPins {
    const uint8_t* serPins;
    uint8_t serCount;
    const uint8_t* srclkPins;
    uint8_t srclkCount;
    const uint8_t* rclkPins;
    uint8_t rclkCount;
  };

  ShiftRegisterClocklessLedDriver() = default;
  ShiftRegisterClocklessLedDriver(const ShiftRegisterClocklessLedDriver&) = delete;
  ShiftRegisterClocklessLedDriver& operator=(const ShiftRegisterClocklessLedDriver&) = delete;
  ~ShiftRegisterClocklessLedDriver();

  bool initled(uint8_t* leds,
               Pins pins,
               uint8_t numOutputs,
               uint16_t numLedPerOutput,
               ColorArrangement colorArrangement = ORDER_GRB);

  bool initled(uint8_t* leds,
               ParallelPins pins,
               uint8_t numOutputs,
               uint16_t numLedPerOutput,
               ColorArrangement colorArrangement = ORDER_GRB);

  bool initled(uint8_t* leds,
               const uint8_t* serPins,
               uint8_t serCount,
               const uint8_t* srclkPins,
               uint8_t srclkCount,
               const uint8_t* rclkPins,
               uint8_t rclkCount,
               uint8_t numOutputs,
               uint16_t numLedPerOutput,
               ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, ParallelPins{serPins, serCount, srclkPins, srclkCount, rclkPins, rclkCount}, numOutputs, numLedPerOutput, colorArrangement);
  }

  bool initled(uint8_t* leds,
               uint8_t serPin,
               uint8_t srclkPin,
               uint8_t rclkPin,
               uint8_t numOutputs,
               uint16_t numLedPerOutput,
               ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, Pins{serPin, srclkPin, rclkPin}, numOutputs, numLedPerOutput, colorArrangement);
  }

  bool begin(uint8_t* leds,
             Pins pins,
             uint8_t numOutputs,
             uint16_t numLedPerOutput,
             ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, pins, numOutputs, numLedPerOutput, colorArrangement);
  }

  bool begin(uint8_t* leds,
             ParallelPins pins,
             uint8_t numOutputs,
             uint16_t numLedPerOutput,
             ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, pins, numOutputs, numLedPerOutput, colorArrangement);
  }

  bool begin(uint8_t* leds,
             const uint8_t* serPins,
             uint8_t serCount,
             const uint8_t* srclkPins,
             uint8_t srclkCount,
             const uint8_t* rclkPins,
             uint8_t rclkCount,
             uint8_t numOutputs,
             uint16_t numLedPerOutput,
             ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, serPins, serCount, srclkPins, srclkCount, rclkPins, rclkCount, numOutputs, numLedPerOutput, colorArrangement);
  }

  bool begin(uint8_t* leds,
             uint8_t serPin,
             uint8_t srclkPin,
             uint8_t rclkPin,
             uint8_t numOutputs,
             uint16_t numLedPerOutput,
             ColorArrangement colorArrangement = ORDER_GRB) {
    return initled(leds, serPin, srclkPin, rclkPin, numOutputs, numLedPerOutput, colorArrangement);
  }

  void end();

  void setBrightness(uint8_t brightness);
  uint8_t getBrightness() const { return brightness_; }

  void clear();
  void setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue);
  void setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);
  void setPixelByOutput(uint8_t output, uint16_t ledIndex, uint8_t red, uint8_t green, uint8_t blue);
  void setPixelByOutput(uint8_t output, uint16_t ledIndex, uint8_t red, uint8_t green, uint8_t blue, uint8_t white);

  bool showPixels();
  bool showPixels(uint8_t* newLeds);
  bool show() { return showPixels(); }

  bool isReady() const { return initialized_; }
  const char* lastError() const { return lastError_; }
  size_t frameBytes() const { return frameBytes_; }
  size_t frameSamples() const { return frameSamples_; }
  uint32_t shiftSampleHz() const { return SHIFT_SAMPLE_HZ; }
  uint32_t shiftClockHz() const { return SHIFT_SAMPLE_HZ / 2u; }
  uint8_t serCount() const { return serCount_; }
  uint8_t outputCount() const { return numOutputs_; }

 private:
  static constexpr uint8_t MAX_SER_PINS = 10;
  static constexpr uint8_t MAX_CLOCK_PINS = 2;
  static constexpr uint8_t MAX_LATCH_PINS = 2;
  static constexpr uint8_t MAX_OUTPUTS = MAX_SER_PINS * 8;
  static constexpr uint8_t SHIFT_REGISTER_BITS = 8;
  static constexpr uint8_t CHANNELS_MAX = 5;
  static constexpr uint32_t WS2812_BIT_HZ = 800000;
  static constexpr uint32_t SHIFT_SAMPLE_HZ = 64000000;
  static constexpr uint32_t MAX_PARLIO_TRANSFER_BYTES = 65535;

  static constexpr uint8_t SHIFT_SAMPLES_PER_BIT = 2;
  static constexpr uint8_t SHIFT_SETTLE_SAMPLES = 1;
  static constexpr uint8_t LATCH_HIGH_SAMPLES = 2;
  static constexpr uint8_t LATCH_LOW_SAMPLES = 1;

  static uint16_t nsToSamples(uint32_t ns);

  bool configurePins(ParallelPins pins, uint8_t numOutputs);
  bool configureColor(ColorArrangement colorArrangement);
  bool configureTiming();
  bool allocateFrameBuffer();
  bool initHardware();
  void releaseHardware();
  bool buildBaseFrame();
  bool buildFrame(size_t& samples);
  bool transmitFrame(size_t samples);
  size_t bytesForSamples(size_t samples) const;

  void putSample(uint16_t sample);
  void padUntil(size_t targetSample);
  void appendShiftRegisterUpdate(const uint8_t* outputBytes);
  void appendShiftRegisterUpdateSamples(const uint16_t* serSamples);
  void appendOutputLatchAt(size_t latchSample, const uint8_t* outputBytes);
  void appendOutputLatchSamplesAt(size_t latchSample, const uint16_t* serSamples);

  uint8_t rawChannelForWireChannel(uint8_t wireChannel) const;
  uint8_t wireByte(uint8_t output, uint16_t ledIndex, uint8_t wireChannel) const;
  void serSamplesForWireChannel(uint16_t ledIndex, uint8_t wireChannel, uint16_t serSamples[8][SHIFT_REGISTER_BITS]) const;
  uint8_t scaleBrightness(uint8_t value) const;
  void setError(const char* message);

  uint8_t serPins_[MAX_SER_PINS] = {};
  uint8_t srclkPins_[MAX_CLOCK_PINS] = {};
  uint8_t rclkPins_[MAX_LATCH_PINS] = {};
  uint16_t serSampleMasks_[MAX_SER_PINS] = {};
  uint8_t activeOutputBytes_[MAX_SER_PINS] = {};
  uint8_t serCount_ = 0;
  uint8_t srclkCount_ = 0;
  uint8_t rclkCount_ = 0;
  uint8_t signalCount_ = 0;
  uint8_t dataWidth_ = 4;
  uint8_t bytesPerSample_ = 1;
  uint16_t srclkSampleMask_ = 0;
  uint16_t rclkSampleMask_ = 0;

  uint8_t* leds_ = nullptr;
  uint8_t* txBuffer_ = nullptr;
  void* txUnit_ = nullptr;

  uint8_t numOutputs_ = 0;
  uint16_t numLedPerOutput_ = 0;
  uint32_t totalLeds_ = 0;
  uint8_t channelsPerLight_ = 3;
  uint8_t offsetRed_ = 1;
  uint8_t offsetGreen_ = 0;
  uint8_t offsetBlue_ = 2;
  uint8_t offsetWhite_ = UINT8_MAX;
  uint8_t offsetWhite2_ = UINT8_MAX;
  uint8_t brightness_ = 255;
  uint8_t brightnessMap_[256] = {};

  uint16_t bitSamples_ = 0;
  uint16_t t0hSamples_ = 0;
  uint16_t t1hSamples_ = 0;
  uint16_t resetSamples_ = 0;
  uint8_t latchEdgeOffset_ = 0;
  uint8_t shiftUpdateSamples_ = 0;
  size_t frameSamples_ = 0;
  size_t frameBytes_ = 0;
  size_t sampleCursor_ = 0;

  bool timingOverrun_ = false;
  bool baseFrameReady_ = false;
  bool initialized_ = false;
  const char* lastError_ = nullptr;
};
