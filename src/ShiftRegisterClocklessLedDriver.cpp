#include "ShiftRegisterClocklessLedDriver.h"

#include <stdlib.h>
#include <string.h>

#include "Arduino.h"

#if defined(CONFIG_IDF_TARGET_ESP32P4)
#include "driver/parlio_tx.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "soc/soc_caps.h"
#if SHIFT_REGISTER_EXPERIMENTAL_SINGLE_TRANSACTION
#include "esp_cache.h"
#include "esp_memory_utils.h"
#endif
#define SHIFT_REGISTER_DRIVER_HAS_PARLIO 1
#else
#define SHIFT_REGISTER_DRIVER_HAS_PARLIO 0
#endif

ShiftRegisterClocklessLedDriver::~ShiftRegisterClocklessLedDriver() {
  end();
}

bool ShiftRegisterClocklessLedDriver::initled(uint8_t* leds,
                                              Pins pins,
                                              uint8_t numOutputs,
                                              uint16_t numLedPerOutput,
                                              ColorArrangement colorArrangement) {
  uint8_t serPins[] = {pins.ser};
  uint8_t srclkPins[] = {pins.srclk};
  uint8_t rclkPins[] = {pins.rclk};
  return initled(leds, ParallelPins{serPins, 1, srclkPins, 1, rclkPins, 1}, numOutputs, numLedPerOutput, colorArrangement);
}

bool ShiftRegisterClocklessLedDriver::initled(uint8_t* leds,
                                              ParallelPins pins,
                                              uint8_t numOutputs,
                                              uint16_t numLedPerOutput,
                                              ColorArrangement colorArrangement) {
  end();
  lastError_ = nullptr;

  if (leds == nullptr) {
    setError("initled: leds buffer is null");
    return false;
  }
  if (numLedPerOutput == 0) {
    setError("initled: numLedPerOutput must be > 0");
    return false;
  }

  if (!configurePins(pins, numOutputs)) {
    end();
    return false;
  }

  leds_ = leds;
  numLedPerOutput_ = numLedPerOutput;
  totalLeds_ = static_cast<uint32_t>(numOutputs_) * numLedPerOutput_;

  if (!configureColor(colorArrangement)) {
    end();
    return false;
  }
  setBrightness(brightness_);
  if (!configureTiming()) {
    end();
    return false;
  }
  if (!allocateFrameBuffer()) {
    end();
    return false;
  }
  if (!initHardware()) {
    end();
    return false;
  }

  initialized_ = true;
  return true;
}

void ShiftRegisterClocklessLedDriver::end() {
  releaseHardware();

  if (txBuffer_ != nullptr) {
#if SHIFT_REGISTER_DRIVER_HAS_PARLIO
    heap_caps_free(txBuffer_);
#else
    free(txBuffer_);
#endif
    txBuffer_ = nullptr;
  }

  memset(serPins_, 0, sizeof(serPins_));
  memset(srclkPins_, 0, sizeof(srclkPins_));
  memset(rclkPins_, 0, sizeof(rclkPins_));
  memset(serSampleMasks_, 0, sizeof(serSampleMasks_));
  memset(activeOutputBytes_, 0, sizeof(activeOutputBytes_));
  serCount_ = 0;
  srclkCount_ = 0;
  rclkCount_ = 0;
  signalCount_ = 0;
  dataWidth_ = 4;
  bytesPerSample_ = 1;
  srclkSampleMask_ = 0;
  rclkSampleMask_ = 0;
  leds_ = nullptr;
  numOutputs_ = 0;
  numLedPerOutput_ = 0;
  totalLeds_ = 0;
  channelsPerLight_ = 3;
  offsetRed_ = 1;
  offsetGreen_ = 0;
  offsetBlue_ = 2;
  offsetWhite_ = UINT8_MAX;
  offsetWhite2_ = UINT8_MAX;
  bitSamples_ = 0;
  t0hSamples_ = 0;
  t1hSamples_ = 0;
  resetSamples_ = 0;
  latchEdgeOffset_ = 0;
  shiftUpdateSamples_ = 0;
  frameSamples_ = 0;
  frameBytes_ = 0;
  sampleCursor_ = 0;
  timingOverrun_ = false;
  baseFrameReady_ = false;
  initialized_ = false;
}

void ShiftRegisterClocklessLedDriver::clear() {
  if (leds_ == nullptr || totalLeds_ == 0) return;
  memset(leds_, 0, static_cast<size_t>(totalLeds_) * channelsPerLight_);
}

void ShiftRegisterClocklessLedDriver::setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue) {
  setPixel(pos, red, green, blue, 0);
}

void ShiftRegisterClocklessLedDriver::setPixel(uint32_t pos, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
  if (leds_ == nullptr || pos >= totalLeds_) return;

  uint8_t* pixel = leds_ + (static_cast<size_t>(pos) * channelsPerLight_);
  pixel[0] = red;
  pixel[1] = green;
  pixel[2] = blue;
  if (channelsPerLight_ > 3) pixel[3] = white;
  if (channelsPerLight_ > 4) pixel[4] = 0;
}

void ShiftRegisterClocklessLedDriver::setPixelByOutput(uint8_t output, uint16_t ledIndex, uint8_t red, uint8_t green, uint8_t blue) {
  setPixelByOutput(output, ledIndex, red, green, blue, 0);
}

void ShiftRegisterClocklessLedDriver::setPixelByOutput(uint8_t output, uint16_t ledIndex, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
  if (output >= numOutputs_ || ledIndex >= numLedPerOutput_) return;
  setPixel(static_cast<uint32_t>(output) * numLedPerOutput_ + ledIndex, red, green, blue, white);
}

void ShiftRegisterClocklessLedDriver::setBrightness(uint8_t brightness) {
  brightness_ = brightness;
  for (uint16_t i = 0; i < 256; i++) {
    brightnessMap_[i] = static_cast<uint8_t>((i * brightness_ + 127u) / 255u);
  }
}

bool ShiftRegisterClocklessLedDriver::showPixels(uint8_t* newLeds) {
  if (newLeds == nullptr) {
    setError("showPixels: leds buffer is null");
    return false;
  }
  leds_ = newLeds;
  return showPixels();
}

bool ShiftRegisterClocklessLedDriver::showPixels() {
  if (!initialized_) {
    setError("showPixels: driver is not initialized");
    return false;
  }
  if (leds_ == nullptr) {
    setError("showPixels: leds buffer is null");
    return false;
  }

  size_t samples = 0;
  if (!buildFrame(samples)) return false;
  return transmitFrame(samples);
}

uint16_t ShiftRegisterClocklessLedDriver::nsToSamples(uint32_t ns) {
  return static_cast<uint16_t>((static_cast<uint64_t>(SHIFT_SAMPLE_HZ) * ns + 999999999ull) / 1000000000ull);
}

bool ShiftRegisterClocklessLedDriver::configurePins(ParallelPins pins, uint8_t numOutputs) {
  if (pins.serPins == nullptr || pins.srclkPins == nullptr || pins.rclkPins == nullptr) {
    setError("initled: pin arrays must not be null");
    return false;
  }
  if (pins.serCount == 0 || pins.serCount > MAX_SER_PINS) {
    setError("initled: serCount must be 1..10");
    return false;
  }
  if (pins.srclkCount == 0 || pins.srclkCount > MAX_CLOCK_PINS) {
    setError("initled: srclkCount must be 1..2");
    return false;
  }
  if (pins.rclkCount == 0 || pins.rclkCount > MAX_LATCH_PINS) {
    setError("initled: rclkCount must be 1..2");
    return false;
  }
  if (numOutputs == 0 || numOutputs > MAX_OUTPUTS || numOutputs > pins.serCount * SHIFT_REGISTER_BITS) {
    setError("initled: numOutputs exceeds available shift-register outputs");
    return false;
  }

  signalCount_ = pins.serCount + pins.srclkCount + pins.rclkCount;
  if (signalCount_ > 16) {
    setError("initled: PARLIO signal count exceeds 16 data lines");
    return false;
  }

  serCount_ = pins.serCount;
  srclkCount_ = pins.srclkCount;
  rclkCount_ = pins.rclkCount;
  numOutputs_ = numOutputs;

  for (uint8_t i = 0; i < serCount_; i++) {
    serPins_[i] = pins.serPins[i];
    serSampleMasks_[i] = static_cast<uint16_t>(1u << i);
  }

  uint8_t signalIndex = serCount_;
  for (uint8_t i = 0; i < srclkCount_; i++) {
    srclkPins_[i] = pins.srclkPins[i];
    srclkSampleMask_ |= static_cast<uint16_t>(1u << signalIndex++);
  }
  for (uint8_t i = 0; i < rclkCount_; i++) {
    rclkPins_[i] = pins.rclkPins[i];
    rclkSampleMask_ |= static_cast<uint16_t>(1u << signalIndex++);
  }

  memset(activeOutputBytes_, 0, sizeof(activeOutputBytes_));
  for (uint8_t output = 0; output < numOutputs_; output++) {
    activeOutputBytes_[output / SHIFT_REGISTER_BITS] |= static_cast<uint8_t>(1u << (output % SHIFT_REGISTER_BITS));
  }

  if (signalCount_ <= 4) {
    dataWidth_ = 4;
    bytesPerSample_ = 0;  // 4-bit samples are packed two per byte.
  } else if (signalCount_ <= 8) {
    dataWidth_ = 8;
    bytesPerSample_ = 1;
  } else {
    dataWidth_ = 16;
    bytesPerSample_ = 2;
  }

  return true;
}

bool ShiftRegisterClocklessLedDriver::configureColor(ColorArrangement colorArrangement) {
  uint8_t channels = 0;
  uint8_t r = UINT8_MAX;
  uint8_t g = UINT8_MAX;
  uint8_t b = UINT8_MAX;
  uint8_t w = UINT8_MAX;
  uint8_t w2 = UINT8_MAX;
  applyColorArrangement(colorArrangement, channels, r, g, b, w, w2);

  if (channels < 3 || channels > CHANNELS_MAX) {
    setError("initled: unsupported color arrangement");
    return false;
  }

  channelsPerLight_ = channels;
  offsetRed_ = r;
  offsetGreen_ = g;
  offsetBlue_ = b;
  offsetWhite_ = w;
  offsetWhite2_ = w2;
  return true;
}

bool ShiftRegisterClocklessLedDriver::configureTiming() {
  if ((SHIFT_SAMPLE_HZ % WS2812_BIT_HZ) != 0) {
    setError("initled: shift sample clock must divide evenly by 800 kHz");
    return false;
  }

  bitSamples_ = static_cast<uint16_t>(SHIFT_SAMPLE_HZ / WS2812_BIT_HZ);
  t0hSamples_ = nsToSamples(400);
  t1hSamples_ = nsToSamples(800);
  resetSamples_ = nsToSamples(80000);
  latchEdgeOffset_ = SHIFT_REGISTER_BITS * SHIFT_SAMPLES_PER_BIT + SHIFT_SETTLE_SAMPLES;
  shiftUpdateSamples_ = latchEdgeOffset_ + LATCH_HIGH_SAMPLES + LATCH_LOW_SAMPLES;

  if (t0hSamples_ <= latchEdgeOffset_) {
    setError("initled: T0H is too short for one shift-register update");
    return false;
  }
  if ((t1hSamples_ - t0hSamples_) <= shiftUpdateSamples_) {
    setError("initled: T0H to T1H gap is too short");
    return false;
  }
  if ((bitSamples_ - t1hSamples_) <= shiftUpdateSamples_) {
    setError("initled: T1H to next-bit gap is too short");
    return false;
  }

  const uint64_t ws2812BitsPerFrame = static_cast<uint64_t>(numLedPerOutput_) * channelsPerLight_ * 8u;
  uint64_t samples = static_cast<uint64_t>(resetSamples_) +
                     (ws2812BitsPerFrame * bitSamples_) +
                     resetSamples_ +
                     shiftUpdateSamples_;
  if (dataWidth_ == 4 && (samples & 1ull) != 0) samples++;
  const uint64_t bytes = (samples * dataWidth_ + 7u) / 8u;

  if (samples > static_cast<uint64_t>(SIZE_MAX) || bytes > static_cast<uint64_t>(SIZE_MAX)) {
    setError("initled: frame is too large");
    return false;
  }

  frameSamples_ = static_cast<size_t>(samples);
  frameBytes_ = static_cast<size_t>(bytes);
  return true;
}

bool ShiftRegisterClocklessLedDriver::allocateFrameBuffer() {
#if SHIFT_REGISTER_DRIVER_HAS_PARLIO
#if SHIFT_REGISTER_EXPERIMENTAL_SINGLE_TRANSACTION
  txBuffer_ = static_cast<uint8_t*>(
      heap_caps_calloc(frameBytes_, 1, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (txBuffer_ == nullptr) {
    txBuffer_ = static_cast<uint8_t*>(
        heap_caps_calloc(frameBytes_, 1,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_CACHE_ALIGNED));
  }
#else
  txBuffer_ = static_cast<uint8_t*>(
      heap_caps_calloc_prefer(frameBytes_, 1, 2,
                              MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL,
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_CACHE_ALIGNED));
#endif
#else
  txBuffer_ = nullptr;
#endif
  if (txBuffer_ == nullptr) {
    setError("initled: failed to allocate PARLIO frame buffer");
    return false;
  }
  return true;
}

bool ShiftRegisterClocklessLedDriver::initHardware() {
#if SHIFT_REGISTER_DRIVER_HAS_PARLIO
  for (uint8_t i = 0; i < serCount_; i++) {
    pinMode(serPins_[i], OUTPUT);
    digitalWrite(serPins_[i], LOW);
  }
  for (uint8_t i = 0; i < srclkCount_; i++) {
    pinMode(srclkPins_[i], OUTPUT);
    digitalWrite(srclkPins_[i], LOW);
  }
  for (uint8_t i = 0; i < rclkCount_; i++) {
    pinMode(rclkPins_[i], OUTPUT);
    digitalWrite(rclkPins_[i], LOW);
  }

  parlio_tx_unit_config_t config = {};
  config.clk_src = PARLIO_CLK_SRC_DEFAULT;
  config.data_width = dataWidth_;
  config.clk_in_gpio_num = gpio_num_t(-1);
  config.valid_gpio_num = gpio_num_t(-1);
  config.clk_out_gpio_num = gpio_num_t(-1);
  for (int i = 0; i < SOC_PARLIO_TX_UNIT_MAX_DATA_WIDTH; i++) {
    config.data_gpio_nums[i] = gpio_num_t(-1);
  }
  if (dataWidth_ > SOC_PARLIO_TX_UNIT_MAX_DATA_WIDTH) {
    setError("initled: requested PARLIO width is not supported by this target");
    return false;
  }

  uint8_t signalIndex = 0;
  for (uint8_t i = 0; i < serCount_; i++) {
    config.data_gpio_nums[signalIndex++] = gpio_num_t(serPins_[i]);
  }
  for (uint8_t i = 0; i < srclkCount_; i++) {
    config.data_gpio_nums[signalIndex++] = gpio_num_t(srclkPins_[i]);
  }
  for (uint8_t i = 0; i < rclkCount_; i++) {
    config.data_gpio_nums[signalIndex++] = gpio_num_t(rclkPins_[i]);
  }
  config.output_clk_freq_hz = SHIFT_SAMPLE_HZ;
  config.valid_start_delay = 0;
  config.valid_stop_delay = 0;
  config.dma_burst_size = 64;
  config.trans_queue_depth = 16;
#if SHIFT_REGISTER_EXPERIMENTAL_SINGLE_TRANSACTION
  config.max_transfer_size = frameBytes_;
#else
  config.max_transfer_size = MAX_PARLIO_TRANSFER_BYTES;
#endif
  config.flags.clk_gate_en = 0;
  config.flags.io_loop_back = 0;
  config.flags.allow_pd = 0;
  config.flags.invert_valid_out = 0;

  parlio_tx_unit_handle_t txUnit = nullptr;
  esp_err_t err = parlio_new_tx_unit(&config, &txUnit);
  if (err != ESP_OK) {
    setError("initled: parlio_new_tx_unit failed");
    return false;
  }

  err = parlio_tx_unit_enable(txUnit);
  if (err != ESP_OK) {
    parlio_del_tx_unit(txUnit);
    setError("initled: parlio_tx_unit_enable failed");
    return false;
  }

  txUnit_ = txUnit;
  return true;
#else
  setError("initled: shift-register WS2812 driver requires ESP32-P4 PARLIO");
  return false;
#endif
}

void ShiftRegisterClocklessLedDriver::releaseHardware() {
#if SHIFT_REGISTER_DRIVER_HAS_PARLIO
  if (txUnit_ == nullptr) return;

  parlio_tx_unit_handle_t txUnit = reinterpret_cast<parlio_tx_unit_handle_t>(txUnit_);
  parlio_tx_unit_wait_all_done(txUnit, portMAX_DELAY);
  parlio_tx_unit_disable(txUnit);
  parlio_del_tx_unit(txUnit);
  txUnit_ = nullptr;
#else
  txUnit_ = nullptr;
#endif
}

bool ShiftRegisterClocklessLedDriver::buildBaseFrame() {
  memset(txBuffer_, 0, frameBytes_);
  sampleCursor_ = 0;
  timingOverrun_ = false;

  uint8_t zeroBytes[MAX_SER_PINS] = {};

  appendShiftRegisterUpdate(zeroBytes);

  const size_t dataStartSample = resetSamples_;
  uint32_t wsBitIndex = 0;

  for (uint16_t ledIndex = 0; ledIndex < numLedPerOutput_; ledIndex++) {
    for (uint8_t wireChannel = 0; wireChannel < channelsPerLight_; wireChannel++) {
      for (uint8_t bit = 0; bit < 8; bit++) {
        const size_t bitStart = dataStartSample + (static_cast<size_t>(wsBitIndex) * bitSamples_);

        appendOutputLatchAt(bitStart, activeOutputBytes_);
        appendOutputLatchAt(bitStart + t1hSamples_, zeroBytes);

        wsBitIndex++;
      }
    }
  }

  padUntil(dataStartSample +
           (static_cast<size_t>(numLedPerOutput_) * channelsPerLight_ * 8u * bitSamples_) +
           resetSamples_);
  padUntil(frameSamples_);

  if (timingOverrun_) {
    setError("showPixels: base frame buffer overrun");
    return false;
  }

  baseFrameReady_ = true;
  return true;
}

bool ShiftRegisterClocklessLedDriver::buildFrame(size_t& samples) {
  if (!baseFrameReady_ && !buildBaseFrame()) return false;

  sampleCursor_ = 0;
  timingOverrun_ = false;

  uint16_t serSamples[8][SHIFT_REGISTER_BITS] = {};

  const size_t dataStartSample = resetSamples_;
  uint32_t wsBitIndex = 0;

  for (uint16_t ledIndex = 0; ledIndex < numLedPerOutput_; ledIndex++) {
    for (uint8_t wireChannel = 0; wireChannel < channelsPerLight_; wireChannel++) {
      serSamplesForWireChannel(ledIndex, wireChannel, serSamples);
      for (uint8_t bit = 0; bit < 8; bit++) {
        const size_t bitStart = dataStartSample + (static_cast<size_t>(wsBitIndex) * bitSamples_);
        appendOutputLatchSamplesAt(bitStart + t0hSamples_, serSamples[bit]);
        wsBitIndex++;
      }
    }
  }

  if (timingOverrun_) {
    setError("showPixels: frame patch overrun");
    return false;
  }

  samples = sampleCursor_;
  if (samples < frameSamples_) samples = frameSamples_;
  return true;
}

bool ShiftRegisterClocklessLedDriver::transmitFrame(size_t samples) {
#if SHIFT_REGISTER_DRIVER_HAS_PARLIO
  parlio_tx_unit_handle_t txUnit = reinterpret_cast<parlio_tx_unit_handle_t>(txUnit_);
  if (txUnit == nullptr) {
    setError("showPixels: PARLIO TX unit is not initialized");
    return false;
  }

  parlio_transmit_config_t txConfig = {};
  txConfig.idle_value = 0x00;
  txConfig.flags.queue_nonblocking = 0;
  txConfig.flags.loop_transmission = 0;

#if SHIFT_REGISTER_EXPERIMENTAL_SINGLE_TRANSACTION
  const size_t transmitBytes = (samples * dataWidth_ + 7u) / 8u;
  if (esp_ptr_external_ram(txBuffer_)) {
    const int syncFlags = ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                          ESP_CACHE_MSYNC_FLAG_UNALIGNED;
    const esp_err_t syncErr = esp_cache_msync(txBuffer_, transmitBytes, syncFlags);
    if (syncErr != ESP_OK) {
      setError("showPixels: esp_cache_msync failed");
      return false;
    }
  }

  const esp_err_t transmitErr =
      parlio_tx_unit_transmit(txUnit, txBuffer_, samples * dataWidth_, &txConfig);
  if (transmitErr != ESP_OK) {
    setError("showPixels: parlio_tx_unit_transmit failed");
    return false;
  }
#else
  size_t maxSamplesPerChunk = (MAX_PARLIO_TRANSFER_BYTES * 8u) / dataWidth_;
  if (bitSamples_ > 1 && maxSamplesPerChunk > bitSamples_) {
    maxSamplesPerChunk = (maxSamplesPerChunk / bitSamples_) * bitSamples_;
  }
  if (dataWidth_ == 4) {
    maxSamplesPerChunk &= ~size_t(1u);
  }
  if (maxSamplesPerChunk == 0) {
    setError("showPixels: PARLIO chunk size is too small");
    return false;
  }

  const uint8_t* chunk = txBuffer_;
  size_t remainingSamples = samples;
  while (remainingSamples > 0) {
    size_t chunkSamples = remainingSamples;
    if (chunkSamples > maxSamplesPerChunk) {
      chunkSamples = maxSamplesPerChunk;
    }
    if (dataWidth_ == 4 && (chunkSamples & 1u) != 0 && chunkSamples > 1) {
      chunkSamples--;
    }

    const esp_err_t transmitErr =
        parlio_tx_unit_transmit(txUnit, chunk, chunkSamples * dataWidth_, &txConfig);
    if (transmitErr != ESP_OK) {
      setError("showPixels: parlio_tx_unit_transmit failed");
      return false;
    }

    chunk += bytesForSamples(chunkSamples);
    remainingSamples -= chunkSamples;
  }
#endif

  const esp_err_t err = parlio_tx_unit_wait_all_done(txUnit, portMAX_DELAY);
  if (err != ESP_OK) {
    setError("showPixels: parlio_tx_unit_wait_all_done failed");
    return false;
  }

  return true;
#else
  (void)samples;
  setError("showPixels: shift-register WS2812 driver requires ESP32-P4 PARLIO");
  return false;
#endif
}

size_t ShiftRegisterClocklessLedDriver::bytesForSamples(size_t samples) const {
  return (samples * dataWidth_ + 7u) / 8u;
}

void ShiftRegisterClocklessLedDriver::putSample(uint16_t sample) {
  if (sampleCursor_ >= frameSamples_) {
    timingOverrun_ = true;
    return;
  }

  if (dataWidth_ == 4) {
    const size_t byteIndex = sampleCursor_ >> 1;
    if ((sampleCursor_ & 1u) == 0) {
      txBuffer_[byteIndex] = (txBuffer_[byteIndex] & 0xF0u) | (sample & 0x0Fu);
    } else {
      txBuffer_[byteIndex] = (txBuffer_[byteIndex] & 0x0Fu) | ((sample & 0x0Fu) << 4);
    }
  } else if (dataWidth_ == 8) {
    txBuffer_[sampleCursor_] = static_cast<uint8_t>(sample);
  } else {
    reinterpret_cast<uint16_t*>(txBuffer_)[sampleCursor_] = sample;
  }
  sampleCursor_++;
}

void ShiftRegisterClocklessLedDriver::padUntil(size_t targetSample) {
  if (targetSample > frameSamples_) {
    sampleCursor_ = frameSamples_;
    timingOverrun_ = true;
    return;
  }
  if (sampleCursor_ < targetSample) sampleCursor_ = targetSample;
}

void ShiftRegisterClocklessLedDriver::appendShiftRegisterUpdate(const uint8_t* outputBytes) {
  uint16_t serSamples[SHIFT_REGISTER_BITS] = {};
  for (int bit = SHIFT_REGISTER_BITS - 1; bit >= 0; bit--) {
    const uint8_t sampleIndex = SHIFT_REGISTER_BITS - 1 - bit;
    for (uint8_t ser = 0; ser < serCount_; ser++) {
      if ((outputBytes[ser] & (1u << bit)) != 0) {
        serSamples[sampleIndex] |= serSampleMasks_[ser];
      }
    }
  }
  appendShiftRegisterUpdateSamples(serSamples);
}

void ShiftRegisterClocklessLedDriver::appendShiftRegisterUpdateSamples(const uint16_t* serSamples) {
  if (dataWidth_ == 16) {
    if (sampleCursor_ + shiftUpdateSamples_ > frameSamples_) {
      sampleCursor_ = frameSamples_;
      timingOverrun_ = true;
      return;
    }

    uint16_t* out = reinterpret_cast<uint16_t*>(txBuffer_) + sampleCursor_;
    for (uint8_t i = 0; i < SHIFT_REGISTER_BITS; i++) {
      const uint16_t serSample = serSamples[i];
      *out++ = serSample;
      *out++ = serSample | srclkSampleMask_;
    }

    *out++ = 0;
    for (uint8_t i = 0; i < LATCH_HIGH_SAMPLES; i++) {
      *out++ = rclkSampleMask_;
    }
    for (uint8_t i = 0; i < LATCH_LOW_SAMPLES; i++) {
      *out++ = 0;
    }
    sampleCursor_ += shiftUpdateSamples_;
    return;
  }

  for (uint8_t i = 0; i < SHIFT_REGISTER_BITS; i++) {
    const uint16_t serSample = serSamples[i];
    putSample(serSample);
    putSample(serSample | srclkSampleMask_);
  }

  putSample(0);
  for (uint8_t i = 0; i < LATCH_HIGH_SAMPLES; i++) {
    putSample(rclkSampleMask_);
  }
  for (uint8_t i = 0; i < LATCH_LOW_SAMPLES; i++) {
    putSample(0);
  }
}

void ShiftRegisterClocklessLedDriver::appendOutputLatchAt(size_t latchSample, const uint8_t* outputBytes) {
  if (latchSample < latchEdgeOffset_) {
    timingOverrun_ = true;
    return;
  }

  const size_t updateStart = latchSample - latchEdgeOffset_;
  if (sampleCursor_ > updateStart) {
    timingOverrun_ = true;
  } else {
    padUntil(updateStart);
  }
  appendShiftRegisterUpdate(outputBytes);
}

void ShiftRegisterClocklessLedDriver::appendOutputLatchSamplesAt(size_t latchSample, const uint16_t* serSamples) {
  if (latchSample < latchEdgeOffset_) {
    timingOverrun_ = true;
    return;
  }

  const size_t updateStart = latchSample - latchEdgeOffset_;
  if (sampleCursor_ > updateStart) {
    timingOverrun_ = true;
  } else {
    padUntil(updateStart);
  }
  appendShiftRegisterUpdateSamples(serSamples);
}

uint8_t ShiftRegisterClocklessLedDriver::rawChannelForWireChannel(uint8_t wireChannel) const {
  if (wireChannel == offsetRed_) return 0;
  if (wireChannel == offsetGreen_) return 1;
  if (wireChannel == offsetBlue_) return 2;
  if (wireChannel == offsetWhite_) return 3;
  if (wireChannel == offsetWhite2_) return 4;
  return 0;
}

uint8_t ShiftRegisterClocklessLedDriver::wireByte(uint8_t output, uint16_t ledIndex, uint8_t wireChannel) const {
  const uint8_t rawChannel = rawChannelForWireChannel(wireChannel);
  const size_t pixelOffset = (static_cast<size_t>(output) * numLedPerOutput_ + ledIndex) * channelsPerLight_;
  return scaleBrightness(leds_[pixelOffset + rawChannel]);
}

void ShiftRegisterClocklessLedDriver::serSamplesForWireChannel(uint16_t ledIndex, uint8_t wireChannel, uint16_t serSamples[8][SHIFT_REGISTER_BITS]) const {
  memset(serSamples, 0, 8 * SHIFT_REGISTER_BITS * sizeof(uint16_t));

  const uint8_t rawChannel = rawChannelForWireChannel(wireChannel);
  const size_t outputStride = static_cast<size_t>(numLedPerOutput_) * channelsPerLight_;
  const uint8_t* pixel = leds_ + (static_cast<size_t>(ledIndex) * channelsPerLight_) + rawChannel;

  for (uint8_t output = 0; output < numOutputs_; output++) {
    const uint8_t value = scaleBrightness(*pixel);
    const uint8_t ser = output / SHIFT_REGISTER_BITS;
    const uint8_t shiftIndex = SHIFT_REGISTER_BITS - 1 - (output % SHIFT_REGISTER_BITS);
    const uint16_t serMask = serSampleMasks_[ser];

    uint8_t bitMask = 0x80;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((value & bitMask) != 0) serSamples[bit][shiftIndex] |= serMask;
      bitMask >>= 1;
    }

    pixel += outputStride;
  }
}

uint8_t ShiftRegisterClocklessLedDriver::scaleBrightness(uint8_t value) const {
  return brightnessMap_[value];
}

void ShiftRegisterClocklessLedDriver::setError(const char* message) {
  lastError_ = message;
}
