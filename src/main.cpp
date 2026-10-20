#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#if defined(ARDUINO_ARCH_NRF52)
#include <nrf.h>
#endif

#ifndef USER_BUTTON_PIN
#define USER_BUTTON_PIN D1
#endif

namespace {

constexpr uint8_t MatrixPin = D0;
constexpr uint8_t MatrixWidth = 10;
constexpr uint8_t MatrixHeight = 6;
constexpr uint16_t LedCount = MatrixWidth * MatrixHeight;
constexpr uint8_t ImuWhoAmIRegister = 0x0F;
constexpr uint8_t ImuControl1Register = 0x10;
constexpr uint8_t ImuControl2Register = 0x11;
constexpr uint8_t ImuControl3Register = 0x12;
constexpr uint8_t ImuAccelOutputRegister = 0x28;
constexpr uint8_t ImuIdentity = 0x69;
constexpr uint32_t FrameIntervalMs = 33;
constexpr uint32_t ActiveSampleIntervalMs = 20;
constexpr uint32_t IdleSampleIntervalMs = 80;
constexpr uint32_t SleepAfterMs = 12000;
constexpr uint32_t DebounceMs = 25;
constexpr uint32_t DoubleClickMs = 300;
constexpr uint32_t HoldMs = 800;
constexpr float MotionThresholdG = 0.10f;
constexpr float AccelerationPerLsbG = 0.000061f;

const uint8_t BrightnessLevels[] = {12, 28, 48};
constexpr uint8_t BrightnessLevelCount = sizeof(BrightnessLevels) / sizeof(BrightnessLevels[0]);

Adafruit_NeoPixel pixels(LedCount, MatrixPin, NEO_GRB + NEO_KHZ800);

float dye[LedCount] = {};
float dyeScratch[LedCount] = {};
float velocityX[LedCount] = {};
float velocityY[LedCount] = {};
float velocityXScratch[LedCount] = {};
float velocityYScratch[LedCount] = {};

uint8_t imuAddress = 0;
uint8_t colorMode = 0;
uint8_t brightnessMode = 1;
uint8_t pendingClicks = 0;
uint32_t buttonChangedAt = 0;
uint32_t buttonPressedAt = 0;
uint32_t clickDeadline = 0;
uint32_t lastImuSampleAt = 0;
uint32_t lastFrameAt = 0;
uint32_t lastMovementAt = 0;
bool imuAvailable = false;
bool buttonRawPressed = false;
bool buttonPressed = false;
bool holdHandled = false;
bool deviceOn = true;
bool sleeping = false;
bool screenIsBlank = false;
bool gravityInitialized = false;
bool slowImuRate = false;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float linearX = 0.0f;
float linearY = 0.0f;
float motionScore = 0.0f;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

uint16_t pixelIndex(uint8_t column, uint8_t row) {
#if defined(MATRIX_SERPENTINE) && MATRIX_SERPENTINE
  const uint8_t mappedColumn = (row & 1U) ? (MatrixWidth - 1U - column) : column;
  return static_cast<uint16_t>(row) * MatrixWidth + mappedColumn;
#else
  return static_cast<uint16_t>(row) * MatrixWidth + column;
#endif
}

float sampleField(const float* field, float column, float row) {
  column = clampFloat(column, 0.0f, MatrixWidth - 1.001f);
  row = clampFloat(row, 0.0f, MatrixHeight - 1.001f);

  const uint8_t column0 = static_cast<uint8_t>(column);
  const uint8_t row0 = static_cast<uint8_t>(row);
  const uint8_t column1 = column0 + 1U < MatrixWidth ? column0 + 1U : MatrixWidth - 1U;
  const uint8_t row1 = row0 + 1U < MatrixHeight ? row0 + 1U : MatrixHeight - 1U;
  const float columnFraction = column - column0;
  const float rowFraction = row - row0;

  const float top = field[row0 * MatrixWidth + column0] * (1.0f - columnFraction) +
                    field[row0 * MatrixWidth + column1] * columnFraction;
  const float bottom = field[row1 * MatrixWidth + column0] * (1.0f - columnFraction) +
                       field[row1 * MatrixWidth + column1] * columnFraction;
  return top * (1.0f - rowFraction) + bottom * rowFraction;
}

bool readImuRegister(uint8_t address, uint8_t registerAddress, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(address), 1) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

bool writeImuRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool setImuRate(bool slow) {
  if (!imuAvailable || slow == slowImuRate) {
    return imuAvailable;
  }
  if (!writeImuRegister(ImuControl1Register, slow ? 0x10 : 0x40)) {
    return false;
  }
  slowImuRate = slow;
  return true;
}

bool beginImu() {
  Wire.begin();
  Wire.setClock(400000);

  const uint8_t candidateAddresses[] = {0x6A, 0x6B};
  for (uint8_t candidateIndex = 0; candidateIndex < 2; ++candidateIndex) {
    const uint8_t address = candidateAddresses[candidateIndex];
    uint8_t identity = 0;
    if (readImuRegister(address, ImuWhoAmIRegister, identity) && identity == ImuIdentity) {
      imuAddress = address;
      imuAvailable = true;
      break;
    }
  }

  if (!imuAvailable) {
    return false;
  }

  return writeImuRegister(ImuControl3Register, 0x44) &&
         writeImuRegister(ImuControl2Register, 0x00) &&
         writeImuRegister(ImuControl1Register, 0x40);
}

bool readAcceleration(float& sampleX, float& sampleY, float& sampleZ) {
  Wire.beginTransmission(imuAddress);
  Wire.write(ImuAccelOutputRegister);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(imuAddress), 6) != 6) {
    return false;
  }

  const uint8_t lowX = Wire.read();
  const uint8_t highX = Wire.read();
  const uint8_t lowY = Wire.read();
  const uint8_t highY = Wire.read();
  const uint8_t lowZ = Wire.read();
  const uint8_t highZ = Wire.read();
  const int16_t rawX = static_cast<int16_t>(static_cast<uint16_t>(lowX) | (static_cast<uint16_t>(highX) << 8));
  const int16_t rawY = static_cast<int16_t>(static_cast<uint16_t>(lowY) | (static_cast<uint16_t>(highY) << 8));
  const int16_t rawZ = static_cast<int16_t>(static_cast<uint16_t>(lowZ) | (static_cast<uint16_t>(highZ) << 8));

  sampleX = rawX * AccelerationPerLsbG;
  sampleY = rawY * AccelerationPerLsbG;
  sampleZ = rawZ * AccelerationPerLsbG;
  return true;
}

void seedFluid() {
  for (uint16_t index = 0; index < LedCount; ++index) {
    dye[index] *= 0.35f;
    velocityX[index] *= 0.35f;
    velocityY[index] *= 0.35f;
  }
  dye[(MatrixHeight / 2U) * MatrixWidth + MatrixWidth / 2U] = 0.9f;
}

void stepFluid(float deltaSeconds) {
  const float forceX = clampFloat(gravityY * 0.85f + linearY * 2.2f, -2.0f, 2.0f);
  const float forceY = clampFloat(-gravityX * 0.85f - linearX * 2.2f, -2.0f, 2.0f);
  const float timeScale = clampFloat(deltaSeconds * 30.0f, 0.4f, 1.8f);

  for (uint8_t row = 0; row < MatrixHeight; ++row) {
    for (uint8_t column = 0; column < MatrixWidth; ++column) {
      const uint16_t index = row * MatrixWidth + column;
      const float previousColumn = column - velocityX[index] * deltaSeconds * 2.0f;
      const float previousRow = row - velocityY[index] * deltaSeconds * 2.0f;
      velocityXScratch[index] = sampleField(velocityX, previousColumn, previousRow) * 0.985f + forceX * deltaSeconds * 2.6f;
      velocityYScratch[index] = sampleField(velocityY, previousColumn, previousRow) * 0.985f + forceY * deltaSeconds * 2.6f;
    }
  }

  for (uint16_t index = 0; index < LedCount; ++index) {
    velocityX[index] = clampFloat(velocityXScratch[index], -2.8f, 2.8f);
    velocityY[index] = clampFloat(velocityYScratch[index], -2.8f, 2.8f);
  }

  for (uint8_t row = 0; row < MatrixHeight; ++row) {
    for (uint8_t column = 0; column < MatrixWidth; ++column) {
      const uint16_t index = row * MatrixWidth + column;
      const float previousColumn = column - velocityX[index] * deltaSeconds * 2.0f;
      const float previousRow = row - velocityY[index] * deltaSeconds * 2.0f;
      dyeScratch[index] = sampleField(dye, previousColumn, previousRow);
    }
  }

  for (uint8_t row = 0; row < MatrixHeight; ++row) {
    for (uint8_t column = 0; column < MatrixWidth; ++column) {
      const uint16_t index = row * MatrixWidth + column;
      float neighborSum = 0.0f;
      uint8_t neighborCount = 0;
      if (column > 0) {
        neighborSum += dyeScratch[index - 1U];
        ++neighborCount;
      }
      if (column + 1U < MatrixWidth) {
        neighborSum += dyeScratch[index + 1U];
        ++neighborCount;
      }
      if (row > 0) {
        neighborSum += dyeScratch[index - MatrixWidth];
        ++neighborCount;
      }
      if (row + 1U < MatrixHeight) {
        neighborSum += dyeScratch[index + MatrixWidth];
        ++neighborCount;
      }

      const float diffusion = neighborCount ? neighborSum / neighborCount : dyeScratch[index];
      dye[index] = clampFloat((dyeScratch[index] * 0.94f + diffusion * 0.06f) * 0.997f, 0.0f, 1.0f);
      velocityX[index] *= 1.0f - 0.018f * timeScale;
      velocityY[index] *= 1.0f - 0.018f * timeScale;
    }
  }

  if (motionScore > MotionThresholdG) {
    const float sourceColumnPosition = MatrixWidth * 0.5f + clampFloat(linearX * 2.4f, -3.0f, 3.0f);
    const float sourceRowPosition = MatrixHeight * 0.5f + clampFloat(linearY * 2.4f, -2.0f, 2.0f);
    const uint8_t sourceColumn = static_cast<uint8_t>(clampFloat(sourceColumnPosition, 0.0f, MatrixWidth - 1.0f));
    const uint8_t sourceRow = static_cast<uint8_t>(clampFloat(sourceRowPosition, 0.0f, MatrixHeight - 1.0f));
    const uint16_t sourceIndex = sourceRow * MatrixWidth + sourceColumn;
    dye[sourceIndex] = clampFloat(dye[sourceIndex] + clampFloat(motionScore * 0.45f, 0.04f, 0.25f), 0.0f, 1.0f);
    velocityX[sourceIndex] += forceX * 0.035f;
    velocityY[sourceIndex] += forceY * 0.035f;
  }
}

uint32_t colorForDye(float amount) {
  static const uint16_t baseHues[] = {43000, 0, 21000, 9000};
  static const uint16_t hueRanges[] = {9000, 12000, 14000, 20000};
  const float visibleAmount = clampFloat((amount - 0.025f) * 1.18f, 0.0f, 1.0f);
  if (visibleAmount <= 0.0f) {
    return 0;
  }

  const uint16_t hue = baseHues[colorMode] + static_cast<uint16_t>(visibleAmount * hueRanges[colorMode]);
  const uint8_t value = static_cast<uint8_t>(clampFloat(visibleAmount * 255.0f, 0.0f, 255.0f));
  return Adafruit_NeoPixel::ColorHSV(hue, 235, value);
}

void renderFluid() {
  for (uint8_t row = 0; row < MatrixHeight; ++row) {
    for (uint8_t column = 0; column < MatrixWidth; ++column) {
      const uint16_t fieldIndex = row * MatrixWidth + column;
      pixels.setPixelColor(pixelIndex(column, row), colorForDye(dye[fieldIndex]));
    }
  }
  pixels.show();
  screenIsBlank = false;
}

void blankMatrix() {
  if (!screenIsBlank) {
    pixels.clear();
    pixels.show();
    screenIsBlank = true;
  }
}

void toggleDevice() {
  deviceOn = !deviceOn;
  pendingClicks = 0;
  if (deviceOn) {
    sleeping = false;
    lastMovementAt = millis();
    lastFrameAt = lastMovementAt;
    seedFluid();
    setImuRate(false);
  } else {
    sleeping = true;
    setImuRate(true);
    blankMatrix();
  }
}

void processClicks(uint32_t now) {
  if (pendingClicks == 0 || static_cast<int32_t>(now - clickDeadline) < 0) {
    return;
  }

  if (pendingClicks == 1) {
    colorMode = (colorMode + 1U) % 4U;
  } else if (pendingClicks == 2) {
    brightnessMode = (brightnessMode + 1U) % BrightnessLevelCount;
    pixels.setBrightness(BrightnessLevels[brightnessMode]);
  } else {
    colorMode = (colorMode + pendingClicks) % 4U;
  }

  pendingClicks = 0;
  if (deviceOn && !sleeping) {
    renderFluid();
  }
}

void updateButton(uint32_t now) {
  const bool rawPressed = digitalRead(USER_BUTTON_PIN) == LOW;
  if (rawPressed != buttonRawPressed) {
    buttonRawPressed = rawPressed;
    buttonChangedAt = now;
  }

  if (rawPressed != buttonPressed && now - buttonChangedAt >= DebounceMs) {
    buttonPressed = rawPressed;
    if (buttonPressed) {
      buttonPressedAt = now;
      holdHandled = false;
    } else if (!holdHandled) {
      ++pendingClicks;
      clickDeadline = now + DoubleClickMs;
    }
  }

  if (buttonPressed && !holdHandled && now - buttonPressedAt >= HoldMs) {
    holdHandled = true;
    toggleDevice();
  }

  processClicks(now);
}

void updateMotion(uint32_t now) {
  float sampleX = 0.0f;
  float sampleY = 0.0f;
  float sampleZ = 1.0f;
  if (!imuAvailable || !readAcceleration(sampleX, sampleY, sampleZ)) {
    motionScore = 0.0f;
    return;
  }

  if (!gravityInitialized) {
    gravityX = sampleX;
    gravityY = sampleY;
    gravityZ = sampleZ;
    gravityInitialized = true;
  }

  linearX = sampleX - gravityX;
  linearY = sampleY - gravityY;
  const float linearZ = sampleZ - gravityZ;
  motionScore = sqrtf(linearX * linearX + linearY * linearY + linearZ * linearZ);

  const float gravityAlpha = sleeping ? 0.08f : 0.025f;
  gravityX += (sampleX - gravityX) * gravityAlpha;
  gravityY += (sampleY - gravityY) * gravityAlpha;
  gravityZ += (sampleZ - gravityZ) * gravityAlpha;

  if (motionScore > MotionThresholdG) {
    lastMovementAt = now;
    if (sleeping && deviceOn) {
      sleeping = false;
      setImuRate(false);
      seedFluid();
      lastFrameAt = now;
    }
  }
}

void waitForInterrupt() {
#if defined(ARDUINO_ARCH_NRF52)
  __WFE();
#else
  delay(1);
#endif
}

}  // namespace

void setup() {
  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
  pixels.begin();
  pixels.setBrightness(BrightnessLevels[brightnessMode]);
  pixels.clear();
  pixels.show();
  beginImu();

  const uint32_t now = millis();
  buttonRawPressed = digitalRead(USER_BUTTON_PIN) == LOW;
  buttonPressed = buttonRawPressed;
  buttonChangedAt = now;
  buttonPressedAt = now;
  lastMovementAt = now;
  lastFrameAt = now;
  lastImuSampleAt = now;
  seedFluid();
}

void loop() {
  const uint32_t now = millis();
  updateButton(now);

  if (!deviceOn) {
    blankMatrix();
    waitForInterrupt();
    return;
  }

  const uint32_t sampleInterval = sleeping ? IdleSampleIntervalMs : ActiveSampleIntervalMs;
  if (now - lastImuSampleAt >= sampleInterval) {
    lastImuSampleAt = now;
    updateMotion(now);
  }

  if (!sleeping && now - lastMovementAt >= SleepAfterMs) {
    sleeping = true;
    setImuRate(true);
    blankMatrix();
  }

  if (sleeping) {
    waitForInterrupt();
    return;
  }

  if (now - lastFrameAt >= FrameIntervalMs) {
    const float deltaSeconds = clampFloat((now - lastFrameAt) * 0.001f, 0.01f, 0.08f);
    lastFrameAt = now;
    stepFluid(deltaSeconds);
    renderFluid();
  }

  waitForInterrupt();
}