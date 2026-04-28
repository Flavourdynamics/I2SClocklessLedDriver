#include <ShiftRegisterClocklessLedDriver.h>

static constexpr uint8_t PIN_SER = 4;
static constexpr uint8_t PIN_SRCLK = 25;
static constexpr uint8_t PIN_RCLK = 24;
static constexpr uint8_t PIN_BLINK = 5;

static constexpr uint8_t NUM_OUTPUTS = 8;
static constexpr uint16_t NUM_LEDS_PER_OUTPUT = 30;

static uint8_t leds[NUM_OUTPUTS * NUM_LEDS_PER_OUTPUT * 3];
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
      Rgb c = colorWheel(static_cast<uint8_t>(phase + output * 24 + i * 7));
      driver.setPixelByOutput(output, i, c.r, c.g, c.b);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BLINK, OUTPUT);
  digitalWrite(PIN_BLINK, LOW);

  if (!driver.initled(leds, PIN_SER, PIN_SRCLK, PIN_RCLK, NUM_OUTPUTS, NUM_LEDS_PER_OUTPUT, ORDER_GRB)) {
    Serial.printf("Shift-register driver init failed: %s\n", driver.lastError());
    return;
  }

  driver.setBrightness(32);
  driver.clear();
  driver.showPixels();
}

void loop() {
  static uint8_t phase = 0;
  static uint32_t lastBlinkMs = 0;
  static bool blinkState = false;

  const uint32_t now = millis();
  if (now - lastBlinkMs >= 1000) {
    lastBlinkMs = now;
    blinkState = !blinkState;
    digitalWrite(PIN_BLINK, blinkState ? HIGH : LOW);
  }

  renderPattern(phase++);
  driver.showPixels();
  delay(40);
}
