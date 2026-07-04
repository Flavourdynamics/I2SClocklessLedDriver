#include <ShiftRegisterClocklessLedDriver.h>

// Starshroud-Contact board map from MCU_SER_1..MCU_SER_10.
// SER_PINS[0] drives driver lanes 0..7, SER_PINS[1] drives lanes 8..15,
// etc. Board LED_O labels are mapped to those lanes below.
static constexpr uint8_t SER_PINS[10] = {
    50, 28, 29, 30, 31,
    26, 23, 22, 21, 20,
};

// The board fans these MCU nets out to left/right shifted clock/latch domains.
static constexpr uint8_t SRCLK_PINS[1] = {51};
static constexpr uint8_t RCLK_PINS[1] = {52};

static constexpr uint8_t NUM_OUTPUTS = 80;
static constexpr uint16_t NUM_LEDS_PER_OUTPUT = 80;
static constexpr uint32_t TOTAL_LEDS = static_cast<uint32_t>(NUM_OUTPUTS) * NUM_LEDS_PER_OUTPUT;

static uint8_t leds[TOTAL_LEDS * 3];
static ShiftRegisterClocklessLedDriver driver;

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

static Rgb colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return Rgb{static_cast<uint8_t>(255 - pos * 3), 0, static_cast<uint8_t>(pos * 3)};
  }
  if (pos < 170) {
    pos -= 85;
    return Rgb{0, static_cast<uint8_t>(pos * 3), static_cast<uint8_t>(255 - pos * 3)};
  }
  pos -= 170;
  return Rgb{static_cast<uint8_t>(pos * 3), static_cast<uint8_t>(255 - pos * 3), 0};
}

static constexpr uint8_t driverOutputForBoardLabel(uint8_t oneBasedLabel) {
  const uint8_t zeroBasedLabel = oneBasedLabel - 1;
  if (oneBasedLabel < 41) {
    return zeroBasedLabel;
  }

  const uint8_t shiftRegisterBase = static_cast<uint8_t>((zeroBasedLabel / 8u) * 8u);
  const uint8_t shiftRegisterOffset = static_cast<uint8_t>(zeroBasedLabel % 8u);
  return shiftRegisterOffset == 0
             ? shiftRegisterBase
             : static_cast<uint8_t>(shiftRegisterBase + 8u - shiftRegisterOffset);
}

static_assert(driverOutputForBoardLabel(1) == 0, "SHFT1 QA should map to lane 0");
static_assert(driverOutputForBoardLabel(8) == 7, "SHFT1 QH should map to lane 7");
static_assert(driverOutputForBoardLabel(41) == 40, "SHFT6 QA should map to lane 40");
static_assert(driverOutputForBoardLabel(42) == 47, "SHFT6 QH should map to lane 47");
static_assert(driverOutputForBoardLabel(48) == 41, "SHFT6 QB should map to lane 41");
static_assert(driverOutputForBoardLabel(49) == 48, "SHFT7 QA should map to lane 48");
static_assert(driverOutputForBoardLabel(56) == 49, "SHFT7 QB should map to lane 49");

static void renderPattern(uint8_t phase) {
  for (uint8_t label = 1; label <= NUM_OUTPUTS; label++) {
    const uint8_t output = driverOutputForBoardLabel(label);
    for (uint16_t i = 0; i < NUM_LEDS_PER_OUTPUT; i++) {
      Rgb c = colorWheel(static_cast<uint8_t>(phase + (label - 1) * 3 + i * 7));
      uint8_t* pixel = leds + ((static_cast<size_t>(output) * NUM_LEDS_PER_OUTPUT + i) * 3u);
      pixel[0] = c.r;
      pixel[1] = c.g;
      pixel[2] = c.b;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!driver.initled(leds,
                      SER_PINS, 10,
                      SRCLK_PINS, 1,
                      RCLK_PINS, 1,
                      NUM_OUTPUTS,
                      NUM_LEDS_PER_OUTPUT,
                      ORDER_GRB)) {
    Serial.printf("Shift-register driver init failed: %s\n", driver.lastError());
    return;
  }

  driver.setBrightness(10);
  driver.clear();
  driver.showPixels();

  Serial.printf("80x80 shift-register test: %lu LEDs, frame buffer %u bytes\n",
                static_cast<unsigned long>(TOTAL_LEDS),
                static_cast<unsigned>(driver.frameBytes()));
}

void loop() {
  static uint8_t phase = 0;
  static uint32_t lastLogMs = 0;
  static uint32_t frames = 0;
  static uint32_t failedFrames = 0;
  static uint64_t renderUs = 0;
  static uint64_t showUs = 0;

  const uint32_t now = millis();

  const uint32_t renderStartUs = micros();
  renderPattern(phase++);
  const uint32_t showStartUs = micros();
  if (driver.showPixels()) {
    frames++;
  } else {
    failedFrames++;
  }
  const uint32_t showEndUs = micros();
  renderUs += static_cast<uint32_t>(showStartUs - renderStartUs);
  showUs += static_cast<uint32_t>(showEndUs - showStartUs);

  const uint32_t elapsed = now - lastLogMs;
  if (elapsed >= 1000) {
    const uint32_t attempts = frames + failedFrames;
    const float avgRenderMs = attempts ? (renderUs / 1000.0f) / attempts : 0.0f;
    const float avgShowMs = attempts ? (showUs / 1000.0f) / attempts : 0.0f;
    Serial.printf("FPS: %.2f | render: %.2f ms | show: %.2f ms | frames: %lu | failed: %lu | LEDs: %lu | frame bytes: %u\n",
                  frames * 1000.0f / elapsed,
                  avgRenderMs,
                  avgShowMs,
                  static_cast<unsigned long>(frames),
                  static_cast<unsigned long>(failedFrames),
                  static_cast<unsigned long>(TOTAL_LEDS),
                  static_cast<unsigned>(driver.frameBytes()));
    frames = 0;
    failedFrames = 0;
    renderUs = 0;
    showUs = 0;
    lastLogMs = now;
  }
}
