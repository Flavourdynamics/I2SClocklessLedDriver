#include <ShiftRegisterClocklessLedDriver.h>

// Replace these with the board's actual 10 SER GPIOs.
// SER_PINS[0] drives outputs 0..7, SER_PINS[1] drives outputs 8..15, etc.
static constexpr uint8_t SER_PINS[10] = {
    4, 6, 7, 8, 9,
    10, 11, 12, 13, 14,
};

// The two clock/latch domains are pulsed with duplicate timing.
static constexpr uint8_t SRCLK_PINS[2] = {25, 26};
static constexpr uint8_t RCLK_PINS[2] = {24, 27};
static constexpr uint8_t PIN_BLINK = 5;

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

static void renderPattern(uint8_t phase) {
  for (uint8_t output = 0; output < NUM_OUTPUTS; output++) {
    for (uint16_t i = 0; i < NUM_LEDS_PER_OUTPUT; i++) {
      Rgb c = colorWheel(static_cast<uint8_t>(phase + output * 3 + i * 7));
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

  pinMode(PIN_BLINK, OUTPUT);
  digitalWrite(PIN_BLINK, LOW);

  if (!driver.initled(leds,
                      SER_PINS, 10,
                      SRCLK_PINS, 2,
                      RCLK_PINS, 2,
                      NUM_OUTPUTS,
                      NUM_LEDS_PER_OUTPUT,
                      ORDER_GRB)) {
    Serial.printf("Shift-register driver init failed: %s\n", driver.lastError());
    return;
  }

  driver.setBrightness(32);
  driver.clear();
  driver.showPixels();

  Serial.printf("80x80 shift-register test: %lu LEDs, frame buffer %u bytes\n",
                static_cast<unsigned long>(TOTAL_LEDS),
                static_cast<unsigned>(driver.frameBytes()));
}

void loop() {
  static uint8_t phase = 0;
  static uint32_t lastBlinkMs = 0;
  static uint32_t lastLogMs = 0;
  static uint32_t frames = 0;
  static uint32_t failedFrames = 0;
  static uint64_t renderUs = 0;
  static uint64_t showUs = 0;
  static bool blinkState = false;

  const uint32_t now = millis();
  if (now - lastBlinkMs >= 1000) {
    lastBlinkMs = now;
    blinkState = !blinkState;
    digitalWrite(PIN_BLINK, blinkState ? HIGH : LOW);
  }

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
