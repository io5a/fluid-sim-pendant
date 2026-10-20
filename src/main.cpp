#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>

#if defined(ARDUINO_ARCH_NRF52)
#include <nrf.h>
#endif

namespace {

constexpr uint8_t MatrixPin = D0;
constexpr uint8_t MatrixWidth = 6;
constexpr uint8_t MatrixHeight = 10;
constexpr uint8_t FluidWidth = MatrixWidth;
constexpr uint8_t FluidHeight = MatrixHeight;
constexpr uint8_t InitialFluidWidth = 3;
constexpr uint8_t InitialFluidHeight = 6;
constexpr uint8_t InitialFluidStartColumn = (FluidWidth - InitialFluidWidth) / 2U;
constexpr uint8_t InitialFluidStartRow = FluidHeight - InitialFluidHeight;
constexpr uint16_t LedCount = MatrixWidth * MatrixHeight;
constexpr uint16_t FluidCellCount = FluidWidth * FluidHeight;
constexpr uint8_t UGridWidth = FluidWidth + 1U;
constexpr uint8_t UGridHeight = FluidHeight;
constexpr uint8_t VGridWidth = FluidWidth;
constexpr uint8_t VGridHeight = FluidHeight + 1U;
constexpr uint16_t UGridCount = UGridWidth * UGridHeight;
constexpr uint16_t VGridCount = VGridWidth * VGridHeight;
constexpr uint8_t ParticlesPerCell = 4;
constexpr uint16_t ParticleCount = InitialFluidWidth * InitialFluidHeight * ParticlesPerCell;
constexpr uint8_t PressureIterations = 32;
constexpr uint8_t ParticleSeparationIterations = 4;
constexpr float PicBlend = 0.05f;
constexpr float Pi = 3.14159265f;
constexpr float FluidSpeedScale = 8.0f;
constexpr float InitialVortexStrength = 6.0f;
constexpr float SideWallRestitution = 0.95f;
constexpr float TopBottomRestitution = 0.45f;
constexpr float ParticleRadius = 0.26f;
constexpr float MinimumParticleDistance = 2.0f * ParticleRadius;
constexpr uint8_t ImuControl1Register = 0x10;
constexpr uint8_t ImuControl2Register = 0x11;
constexpr uint8_t ImuAddress = 0x6A;
constexpr uint32_t FrameIntervalMs = 10;
constexpr uint32_t ActiveSampleIntervalMs = 20;
constexpr uint32_t IdleSampleIntervalMs = 40;
constexpr uint32_t DebugPrintIntervalMs = 100;
constexpr uint32_t FpsWindowMs = 1000;
constexpr uint32_t ShakeCooldownMs = 1200;
constexpr float HardShakeThresholdG = 2.00f;
constexpr float ShakeReleaseThresholdG = 0.20f;
constexpr uint8_t LedBrightness = 5;

struct FluidParticle {
  float x;
  float y;
  float velocityX;
  float velocityY;
};

Adafruit_NeoPixel pixels(LedCount, MatrixPin, NEO_GRB + NEO_KHZ800);
LSM6DS3 imu(I2C_MODE, ImuAddress);

FluidParticle particles[ParticleCount] = {};
float uGrid[UGridCount] = {};
float uGridBefore[UGridCount] = {};
float uWeight[UGridCount] = {};
float vGrid[VGridCount] = {};
float vGridBefore[VGridCount] = {};
float vWeight[VGridCount] = {};
float pressure[FluidCellCount] = {};
float pressureScratch[FluidCellCount] = {};
float divergence[FluidCellCount] = {};
float cellDensity[FluidCellCount] = {};
uint8_t cellParticleCount[FluidCellCount] = {};

uint32_t lastImuSampleAt = 0;
uint32_t lastFrameAt = 0;
uint32_t lastShakeAt = 0;
uint32_t lastDebugPrintAt = 0;
uint32_t fpsWindowStartedAt = 0;
uint32_t lastFrameIntervalMs = 0;
uint32_t lastSolverTimeUs = 0;
uint32_t lastLedShowTimeUs = 0;
uint16_t framesInFpsWindow = 0;
uint16_t measuredFps = 0;
bool imuAvailable = false;
bool deviceOn = true;
bool shakeArmed = false;
bool screenIsBlank = false;
bool gravityInitialized = false;
bool slowImuRate = false;
bool gyroInitialized = false;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float linearX = 0.0f;
float linearY = 0.0f;
float fluidAccelerationX = 0.0f;
float fluidAccelerationY = 0.0f;
float motionScore = 0.0f;
float gyroXDegreesPerSecond = 0.0f;
float gyroYDegreesPerSecond = 0.0f;
float gyroZDegreesPerSecond = 0.0f;
float previousGyroZRadiansPerSecond = 0.0f;
float angularAccelerationZ = 0.0f;
uint32_t lastGyroSampleAt = 0;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

uint16_t pixelIndex(uint8_t column, uint8_t row) {
  const uint8_t physicalRow = MatrixHeight - 1U - row;
#if defined(MATRIX_SERPENTINE) && MATRIX_SERPENTINE
  const uint8_t mappedColumn = (physicalRow & 1U) ? (MatrixWidth - 1U - column) : column;
  return static_cast<uint16_t>(physicalRow) * MatrixWidth + mappedColumn;
#else
  return static_cast<uint16_t>(physicalRow) * MatrixWidth + column;
#endif
}

uint16_t uIndex(uint8_t column, uint8_t row) {
  return static_cast<uint16_t>(row) * UGridWidth + column;
}

uint16_t vIndex(uint8_t column, uint8_t row) {
  return static_cast<uint16_t>(row) * VGridWidth + column;
}

float sampleGrid(const float* grid, uint8_t gridWidth, uint8_t gridHeight, float column, float row) {
  column = clampFloat(column, 0.0f, gridWidth - 1.001f);
  row = clampFloat(row, 0.0f, gridHeight - 1.001f);

  const uint8_t column0 = static_cast<uint8_t>(column);
  const uint8_t row0 = static_cast<uint8_t>(row);
  const uint8_t column1 = column0 + 1U < gridWidth ? column0 + 1U : gridWidth - 1U;
  const uint8_t row1 = row0 + 1U < gridHeight ? row0 + 1U : gridHeight - 1U;
  const float columnFraction = column - column0;
  const float rowFraction = row - row0;

  const float top = grid[row0 * gridWidth + column0] * (1.0f - columnFraction) +
                    grid[row0 * gridWidth + column1] * columnFraction;
  const float bottom = grid[row1 * gridWidth + column0] * (1.0f - columnFraction) +
                       grid[row1 * gridWidth + column1] * columnFraction;
  return top * (1.0f - rowFraction) + bottom * rowFraction;
}

void scatterGrid(float* grid, float* weights, uint8_t gridWidth, uint8_t gridHeight,
                 float column, float row, float value) {
  column = clampFloat(column, 0.0f, gridWidth - 1.001f);
  row = clampFloat(row, 0.0f, gridHeight - 1.001f);
  const uint8_t column0 = static_cast<uint8_t>(column);
  const uint8_t row0 = static_cast<uint8_t>(row);
  const uint8_t column1 = column0 + 1U < gridWidth ? column0 + 1U : gridWidth - 1U;
  const uint8_t row1 = row0 + 1U < gridHeight ? row0 + 1U : gridHeight - 1U;
  const float columnFraction = column - column0;
  const float rowFraction = row - row0;
  const float weightsForCorners[4] = {
      (1.0f - columnFraction) * (1.0f - rowFraction),
      columnFraction * (1.0f - rowFraction),
      (1.0f - columnFraction) * rowFraction,
      columnFraction * rowFraction};
  const uint16_t indices[4] = {
      static_cast<uint16_t>(row0 * gridWidth + column0),
      static_cast<uint16_t>(row0 * gridWidth + column1),
      static_cast<uint16_t>(row1 * gridWidth + column0),
      static_cast<uint16_t>(row1 * gridWidth + column1)};

  for (uint8_t corner = 0; corner < 4; ++corner) {
    const float weight = weightsForCorners[corner];
    grid[indices[corner]] += value * weight;
    weights[indices[corner]] += weight;
  }
}

bool writeImuRegister(uint8_t registerAddress, uint8_t value) {
  Wire1.beginTransmission(ImuAddress);
  Wire1.write(registerAddress);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

bool beginImu() {
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 2;
  imu.settings.accelSampleRate = 104;
  imu.settings.accelBandWidth = 100;
  imu.settings.gyroEnabled = 1;
  imu.settings.gyroRange = 500;
  imu.settings.gyroSampleRate = 104;
  imu.settings.gyroBandWidth = 100;
  imu.settings.tempEnabled = 0;

  imuAvailable = imu.begin() == IMU_SUCCESS;
  return imuAvailable;
}

bool readAcceleration(float& sampleX, float& sampleY, float& sampleZ) {
  const uint16_t errorCountBefore = imu.allOnesCounter + imu.nonSuccessCounter;
  sampleX = imu.readFloatAccelX();
  sampleY = imu.readFloatAccelY();
  sampleZ = imu.readFloatAccelZ();
  return imu.allOnesCounter + imu.nonSuccessCounter == errorCountBefore;
}

bool setImuRate(bool lowPower) {
  if (!imuAvailable || lowPower == slowImuRate) {
    return imuAvailable;
  }
  const uint8_t rateBits = lowPower ? 0x20 : 0x40;
  if (!writeImuRegister(ImuControl1Register, rateBits) ||
      !writeImuRegister(ImuControl2Register, rateBits | 0x04)) {
    return false;
  }
  slowImuRate = lowPower;
  return true;
}

void enforceGridBoundaries() {
  for (uint8_t row = 0; row < UGridHeight; ++row) {
    uGrid[uIndex(0, row)] = 0.0f;
    uGrid[uIndex(FluidWidth, row)] = 0.0f;
  }
  for (uint8_t column = 0; column < VGridWidth; ++column) {
    vGrid[vIndex(column, 0)] = 0.0f;
    vGrid[vIndex(column, FluidHeight)] = 0.0f;
  }
}

void rasterizeParticles() {
  for (uint16_t index = 0; index < FluidCellCount; ++index) {
    cellParticleCount[index] = 0;
  }

  for (const FluidParticle& particle : particles) {
    const uint8_t column = static_cast<uint8_t>(clampFloat(particle.x, 0.0f, FluidWidth - 1.0f));
    const uint8_t row = static_cast<uint8_t>(clampFloat(particle.y, 0.0f, FluidHeight - 1.0f));
    const uint16_t index = row * FluidWidth + column;
    ++cellParticleCount[index];
  }

  for (uint16_t index = 0; index < FluidCellCount; ++index) {
    const uint8_t count = cellParticleCount[index];
    cellDensity[index] = clampFloat(static_cast<float>(count) / ParticlesPerCell, 0.0f, 1.0f);
  }
}

float streamFunction(float column, float row) {
  const float localColumn = column - InitialFluidStartColumn;
  const float localRow = row - InitialFluidStartRow;
  if (localColumn < 0.0f || localColumn > InitialFluidWidth ||
      localRow < 0.0f || localRow > InitialFluidHeight) {
    return 0.0f;
  }
  return InitialVortexStrength * FluidSpeedScale * sinf(Pi * localColumn / InitialFluidWidth) *
         sinf(Pi * localRow / InitialFluidHeight);
}

void seedFluid() {
  for (uint8_t row = 0; row < UGridHeight; ++row) {
    for (uint8_t column = 0; column < UGridWidth; ++column) {
      uGrid[uIndex(column, row)] = streamFunction(column, row + 1U) - streamFunction(column, row);
    }
  }
  for (uint8_t row = 0; row < VGridHeight; ++row) {
    for (uint8_t column = 0; column < VGridWidth; ++column) {
      vGrid[vIndex(column, row)] = -(streamFunction(column + 1U, row) - streamFunction(column, row));
    }
  }
  enforceGridBoundaries();

  uint16_t particleIndex = 0;

  for (uint8_t row = 0; row < InitialFluidHeight; ++row) {
    for (uint8_t column = 0; column < InitialFluidWidth; ++column) {
      for (uint8_t subRow = 0; subRow < 2; ++subRow) {
        for (uint8_t subColumn = 0; subColumn < 2; ++subColumn) {
          FluidParticle& particle = particles[particleIndex++];
          particle.x = InitialFluidStartColumn + column + (subColumn ? 0.75f : 0.25f);
          particle.y = InitialFluidStartRow + row + (subRow ? 0.75f : 0.25f);
          particle.velocityX = sampleGrid(uGrid, UGridWidth, UGridHeight, particle.x, particle.y - 0.5f);
          particle.velocityY = sampleGrid(vGrid, VGridWidth, VGridHeight, particle.x - 0.5f, particle.y);
        }
      }
    }
  }

  rasterizeParticles();
}

void transferParticlesToGrid() {
  rasterizeParticles();

  for (uint16_t index = 0; index < UGridCount; ++index) {
    uGrid[index] = 0.0f;
    uWeight[index] = 0.0f;
  }
  for (uint16_t index = 0; index < VGridCount; ++index) {
    vGrid[index] = 0.0f;
    vWeight[index] = 0.0f;
  }

  for (const FluidParticle& particle : particles) {
    scatterGrid(uGrid, uWeight, UGridWidth, UGridHeight, particle.x, particle.y - 0.5f, particle.velocityX);
    scatterGrid(vGrid, vWeight, VGridWidth, VGridHeight, particle.x - 0.5f, particle.y, particle.velocityY);
  }

  for (uint16_t index = 0; index < UGridCount; ++index) {
    uGrid[index] = uWeight[index] > 0.000001f ? uGrid[index] / uWeight[index] : 0.0f;
  }
  for (uint16_t index = 0; index < VGridCount; ++index) {
    vGrid[index] = vWeight[index] > 0.000001f ? vGrid[index] / vWeight[index] : 0.0f;
  }

  enforceGridBoundaries();
  for (uint16_t index = 0; index < UGridCount; ++index) {
    uGridBefore[index] = uGrid[index];
  }
  for (uint16_t index = 0; index < VGridCount; ++index) {
    vGridBefore[index] = vGrid[index];
  }
}

void projectGridVelocity() {
  for (uint8_t row = 0; row < FluidHeight; ++row) {
    for (uint8_t column = 0; column < FluidWidth; ++column) {
      const uint16_t index = row * FluidWidth + column;
      if (cellParticleCount[index] == 0) {
        divergence[index] = 0.0f;
        pressure[index] = 0.0f;
        continue;
      }
      divergence[index] = uGrid[uIndex(column + 1U, row)] - uGrid[uIndex(column, row)] +
                           vGrid[vIndex(column, row + 1U)] - vGrid[vIndex(column, row)];
      pressure[index] = 0.0f;
    }
  }

  for (uint8_t iteration = 0; iteration < PressureIterations; ++iteration) {
    for (uint8_t row = 0; row < FluidHeight; ++row) {
      for (uint8_t column = 0; column < FluidWidth; ++column) {
        const uint16_t index = row * FluidWidth + column;
        if (cellParticleCount[index] == 0) {
          pressureScratch[index] = 0.0f;
          continue;
        }
        float neighborPressure = 0.0f;
        uint8_t neighborCount = 0;
        if (column > 0) {
          neighborPressure += pressure[row * FluidWidth + column - 1U];
          ++neighborCount;
        }
        if (column + 1U < FluidWidth) {
          neighborPressure += pressure[row * FluidWidth + column + 1U];
          ++neighborCount;
        }
        if (row > 0) {
          neighborPressure += pressure[(row - 1U) * FluidWidth + column];
          ++neighborCount;
        }
        if (row + 1U < FluidHeight) {
          neighborPressure += pressure[(row + 1U) * FluidWidth + column];
          ++neighborCount;
        }
        pressureScratch[index] = (neighborPressure - divergence[index]) / neighborCount;
      }
    }
    for (uint16_t index = 0; index < FluidCellCount; ++index) {
      pressure[index] = pressureScratch[index];
    }
  }

  for (uint8_t row = 0; row < FluidHeight; ++row) {
    for (uint8_t column = 1; column < FluidWidth; ++column) {
      const uint16_t leftCell = row * FluidWidth + column - 1U;
      const uint16_t rightCell = leftCell + 1U;
      if (cellParticleCount[leftCell] == 0 && cellParticleCount[rightCell] == 0) {
        uGrid[uIndex(column, row)] = 0.0f;
      } else {
        const float leftPressure = cellParticleCount[leftCell] ? pressure[leftCell] : 0.0f;
        const float rightPressure = cellParticleCount[rightCell] ? pressure[rightCell] : 0.0f;
        uGrid[uIndex(column, row)] -= rightPressure - leftPressure;
      }
    }
  }
  for (uint8_t row = 1; row < FluidHeight; ++row) {
    for (uint8_t column = 0; column < FluidWidth; ++column) {
      const uint16_t upperCell = (row - 1U) * FluidWidth + column;
      const uint16_t lowerCell = upperCell + FluidWidth;
      if (cellParticleCount[upperCell] == 0 && cellParticleCount[lowerCell] == 0) {
        vGrid[vIndex(column, row)] = 0.0f;
      } else {
        const float upperPressure = cellParticleCount[upperCell] ? pressure[upperCell] : 0.0f;
        const float lowerPressure = cellParticleCount[lowerCell] ? pressure[lowerCell] : 0.0f;
        vGrid[vIndex(column, row)] -= lowerPressure - upperPressure;
      }
    }
  }
  enforceGridBoundaries();
}

void separateParticles() {
  const float minimumDistanceSquared = MinimumParticleDistance * MinimumParticleDistance;
  const float minimumColumn = ParticleRadius;
  const float maximumColumn = FluidWidth - ParticleRadius;
  const float minimumRow = ParticleRadius;
  const float maximumRow = FluidHeight - ParticleRadius;

  for (uint8_t iteration = 0; iteration < ParticleSeparationIterations; ++iteration) {
    for (uint16_t first = 0; first < ParticleCount; ++first) {
      for (uint16_t second = first + 1U; second < ParticleCount; ++second) {
        FluidParticle& firstParticle = particles[first];
        FluidParticle& secondParticle = particles[second];
        float deltaX = secondParticle.x - firstParticle.x;
        float deltaY = secondParticle.y - firstParticle.y;
        float distanceSquared = deltaX * deltaX + deltaY * deltaY;
        if (distanceSquared >= minimumDistanceSquared) {
          continue;
        }
        if (distanceSquared < 0.000001f) {
          deltaX = ((first + second) & 1U) ? 0.001f : -0.001f;
          deltaY = ((first ^ second) & 1U) ? 0.001f : -0.001f;
          distanceSquared = deltaX * deltaX + deltaY * deltaY;
        }

        const float distance = sqrtf(distanceSquared);
        const float correction = 0.5f * (MinimumParticleDistance - distance) / distance;
        deltaX *= correction;
        deltaY *= correction;
        firstParticle.x -= deltaX;
        firstParticle.y -= deltaY;
        secondParticle.x += deltaX;
        secondParticle.y += deltaY;
      }
    }

    for (FluidParticle& particle : particles) {
      particle.x = clampFloat(particle.x, minimumColumn, maximumColumn);
      particle.y = clampFloat(particle.y, minimumRow, maximumRow);
    }
  }
}

void stepFluid(float deltaSeconds) {
  transferParticlesToGrid();

  const float gravityPlane = sqrtf(gravityX * gravityX + gravityY * gravityY);
  const float inverseGravityPlane = gravityPlane > 0.15f ? 1.0f / gravityPlane : 0.0f;
  const float screenDownX = inverseGravityPlane > 0.0f ? -gravityX * inverseGravityPlane : 1.0f;
  const float screenDownY = inverseGravityPlane > 0.0f ? -gravityY * inverseGravityPlane : 0.0f;
  const float screenRightX = -screenDownY;
  const float screenRightY = screenDownX;
  const float linearRight = -linearX * screenRightX - linearY * screenRightY;
  const float linearDown = -linearX * screenDownX - linearY * screenDownY;
  fluidAccelerationX = FluidSpeedScale *
                       clampFloat(linearRight * 20.0f, -24.0f, 24.0f);
  fluidAccelerationY = FluidSpeedScale *
                       clampFloat(gravityPlane * 10.0f + linearDown * 20.0f, -24.0f, 24.0f);
  const float angularForce = FluidSpeedScale * clampFloat(angularAccelerationZ * 2.0f, -12.0f, 12.0f);
  const float centerColumn = FluidWidth * 0.5f;
  const float centerRow = FluidHeight * 0.5f;
  for (uint8_t row = 0; row < FluidHeight; ++row) {
    for (uint8_t column = 1; column < FluidWidth; ++column) {
      const uint16_t leftCell = row * FluidWidth + column - 1U;
      const uint16_t rightCell = leftCell + 1U;
      if (cellParticleCount[leftCell] || cellParticleCount[rightCell]) {
        const float faceRow = row + 0.5f;
        const float rotationalAcceleration = -angularForce * (faceRow - centerRow);
        uGrid[uIndex(column, row)] +=
            clampFloat(fluidAccelerationX + rotationalAcceleration, -24.0f * FluidSpeedScale,
                 24.0f * FluidSpeedScale) * deltaSeconds;
      } else {
        uGrid[uIndex(column, row)] = 0.0f;
      }
    }
  }
  for (uint8_t row = 1; row < FluidHeight; ++row) {
    for (uint8_t column = 0; column < FluidWidth; ++column) {
      const uint16_t upperCell = (row - 1U) * FluidWidth + column;
      const uint16_t lowerCell = upperCell + FluidWidth;
      if (cellParticleCount[upperCell] || cellParticleCount[lowerCell]) {
        const float faceColumn = column + 0.5f;
        const float rotationalAcceleration = angularForce * (faceColumn - centerColumn);
        vGrid[vIndex(column, row)] +=
            clampFloat(fluidAccelerationY + rotationalAcceleration, -24.0f * FluidSpeedScale,
                 24.0f * FluidSpeedScale) * deltaSeconds;
      } else {
        vGrid[vIndex(column, row)] = 0.0f;
      }
    }
  }

  enforceGridBoundaries();
  projectGridVelocity();

  const float maximumColumn = FluidWidth - ParticleRadius;
  const float maximumRow = FluidHeight - ParticleRadius;
  for (FluidParticle& particle : particles) {
    const float uValue = sampleGrid(uGrid, UGridWidth, UGridHeight, particle.x, particle.y - 0.5f);
    const float uBefore = sampleGrid(uGridBefore, UGridWidth, UGridHeight, particle.x, particle.y - 0.5f);
    const float vValue = sampleGrid(vGrid, VGridWidth, VGridHeight, particle.x - 0.5f, particle.y);
    const float vBefore = sampleGrid(vGridBefore, VGridWidth, VGridHeight, particle.x - 0.5f, particle.y);
    const float flipVelocityX = particle.velocityX + uValue - uBefore;
    const float flipVelocityY = particle.velocityY + vValue - vBefore;
    particle.velocityX = clampFloat(flipVelocityX * (1.0f - PicBlend) + uValue * PicBlend,
                    -8.0f * FluidSpeedScale, 8.0f * FluidSpeedScale);
    particle.velocityY = clampFloat(flipVelocityY * (1.0f - PicBlend) + vValue * PicBlend,
                    -8.0f * FluidSpeedScale, 8.0f * FluidSpeedScale);
    particle.x += particle.velocityX * deltaSeconds;
    particle.y += particle.velocityY * deltaSeconds;

    if (particle.x < ParticleRadius) {
      particle.x = ParticleRadius;
      particle.velocityX = fabsf(particle.velocityX) * SideWallRestitution;
    } else if (particle.x > maximumColumn) {
      particle.x = maximumColumn;
      particle.velocityX = -fabsf(particle.velocityX) * SideWallRestitution;
    }
    if (particle.y < ParticleRadius) {
      particle.y = ParticleRadius;
      particle.velocityY = fabsf(particle.velocityY) * TopBottomRestitution;
    } else if (particle.y > maximumRow) {
      particle.y = maximumRow;
      particle.velocityY = -fabsf(particle.velocityY) * TopBottomRestitution;
    }
  }

  separateParticles();
  rasterizeParticles();
}

uint32_t colorForFluid(uint16_t index) {
  const float density = cellDensity[index];
  if (density <= 0.01f) {
    return 0;
  }
  const uint8_t green = static_cast<uint8_t>(clampFloat(70.0f + density * 185.0f, 0.0f, 255.0f));
  return pixels.Color(0, green, 0);
}

void renderFluid() {
  pixels.clear();
  for (uint8_t row = 0; row < FluidHeight; ++row) {
    for (uint8_t column = 0; column < FluidWidth; ++column) {
      const uint16_t index = row * FluidWidth + column;
      pixels.setPixelColor(pixelIndex(column, row), colorForFluid(index));
    }
  }
  const uint32_t showStartedAt = micros();
  pixels.show();
  lastLedShowTimeUs = micros() - showStartedAt;
  screenIsBlank = false;
}

void blankMatrix() {
  if (!screenIsBlank) {
    pixels.clear();
    pixels.show();
    screenIsBlank = true;
  }
}

void setDeviceOn(bool enabled, uint32_t now) {
  if (deviceOn == enabled) {
    return;
  }

  deviceOn = enabled;
  gyroInitialized = false;
  angularAccelerationZ = 0.0f;
  lastGyroSampleAt = now;
  if (deviceOn) {
    seedFluid();
    lastFrameAt = now;
    setImuRate(false);
  } else {
    setImuRate(true);
    blankMatrix();
  }
}

void updateMotion(uint32_t now) {
  float sampleX = 0.0f;
  float sampleY = 0.0f;
  float sampleZ = 1.0f;
  if (!imuAvailable) {
    if (Serial && now - lastDebugPrintAt >= 1000U) {
      Serial.println("IMU not detected or failed to initialize");
      lastDebugPrintAt = now;
    }
    motionScore = 0.0f;
    return;
  }
  if (!readAcceleration(sampleX, sampleY, sampleZ)) {
    if (Serial && now - lastDebugPrintAt >= 1000U) {
      Serial.println("IMU read failed");
      lastDebugPrintAt = now;
    }
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

  const uint16_t gyroErrorsBefore = imu.allOnesCounter + imu.nonSuccessCounter;
  gyroXDegreesPerSecond = imu.readFloatGyroX();
  gyroYDegreesPerSecond = imu.readFloatGyroY();
  gyroZDegreesPerSecond = imu.readFloatGyroZ();
  const uint16_t gyroErrorsAfter = imu.allOnesCounter + imu.nonSuccessCounter;
  if (gyroErrorsAfter != gyroErrorsBefore) {
    gyroInitialized = false;
    angularAccelerationZ = 0.0f;
  } else {
    const float gyroZRadiansPerSecond = gyroZDegreesPerSecond * (Pi / 180.0f);
    if (gyroInitialized) {
      const float sampleSeconds = clampFloat((now - lastGyroSampleAt) * 0.001f, 0.01f, 0.08f);
      float measuredAngularAcceleration =
          clampFloat((gyroZRadiansPerSecond - previousGyroZRadiansPerSecond) / sampleSeconds, -50.0f, 50.0f);
      if (fabsf(measuredAngularAcceleration) < 0.15f) {
        measuredAngularAcceleration = 0.0f;
      }
      angularAccelerationZ += (measuredAngularAcceleration - angularAccelerationZ) * 0.3f;
    } else {
      gyroInitialized = true;
      angularAccelerationZ = 0.0f;
    }
    previousGyroZRadiansPerSecond = gyroZRadiansPerSecond;
  }
  lastGyroSampleAt = now;

  if (Serial && now - lastDebugPrintAt >= DebugPrintIntervalMs) {
    lastDebugPrintAt = now;
    float maximumSpeedSquared = 0.0f;
    if (deviceOn) {
      for (const FluidParticle& particle : particles) {
        const float speedSquared = particle.velocityX * particle.velocityX +
                                   particle.velocityY * particle.velocityY;
        maximumSpeedSquared = speedSquared > maximumSpeedSquared ? speedSquared : maximumSpeedSquared;
      }
    }
    Serial.print("accel_g x=");
    Serial.print(sampleX, 3);
    Serial.print(" y=");
    Serial.print(sampleY, 3);
    Serial.print(" z=");
    Serial.print(sampleZ, 3);
    Serial.print(" linear_g x=");
    Serial.print(linearX, 3);
    Serial.print(" y=");
    Serial.print(linearY, 3);
    Serial.print(" z=");
    Serial.print(linearZ, 3);
    Serial.print(" fluid_accel x=");
    Serial.print(fluidAccelerationX, 2);
    Serial.print(" y=");
    Serial.print(fluidAccelerationY, 2);
    Serial.print(" motion_g=");
    Serial.print(motionScore, 3);
    Serial.print(" gyro_dps x=");
    Serial.print(gyroXDegreesPerSecond, 1);
    Serial.print(" y=");
    Serial.print(gyroYDegreesPerSecond, 1);
    Serial.print(" z=");
    Serial.print(gyroZDegreesPerSecond, 1);
    Serial.print(" alpha_z=");
    Serial.print(angularAccelerationZ, 2);
    Serial.print(" fluid_speed_cells_s=");
    Serial.print(sqrtf(maximumSpeedSquared), 2);
    Serial.print(" fps=");
    Serial.print(measuredFps);
    Serial.print(" frame_ms=");
    Serial.print(lastFrameIntervalMs);
    Serial.print(" sim_us=");
    Serial.print(lastSolverTimeUs);
    Serial.print(" leds_us=");
    Serial.print(lastLedShowTimeUs);
    Serial.print(" shake=");
    Serial.print(shakeArmed ? "armed" : "cooldown");
    Serial.print(" display=");
    Serial.println(deviceOn ? "on" : "off");
  }

  const float gravityAlpha = deviceOn ? 0.025f : 0.08f;
  gravityX += (sampleX - gravityX) * gravityAlpha;
  gravityY += (sampleY - gravityY) * gravityAlpha;
  gravityZ += (sampleZ - gravityZ) * gravityAlpha;

  if (!shakeArmed && now - lastShakeAt >= ShakeCooldownMs && motionScore < ShakeReleaseThresholdG) {
    shakeArmed = true;
  }

  if (shakeArmed && now - lastShakeAt >= ShakeCooldownMs && motionScore >= HardShakeThresholdG) {
    shakeArmed = false;
    lastShakeAt = now;
    setDeviceOn(!deviceOn, now);
    if (Serial) {
      Serial.print("shake toggle -> ");
      Serial.println(deviceOn ? "on" : "off");
    }
    return;
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
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(LedBrightness);
  pixels.clear();
  pixels.show();
  beginImu();

  const uint32_t now = millis();
  lastShakeAt = now;
  shakeArmed = false;
  lastFrameAt = now;
  lastImuSampleAt = now;
  fpsWindowStartedAt = now;
  seedFluid();
}

void loop() {
  const uint32_t now = millis();
  if (now - fpsWindowStartedAt >= FpsWindowMs) {
    const uint32_t windowMs = now - fpsWindowStartedAt;
    measuredFps = static_cast<uint16_t>(framesInFpsWindow * 1000U / windowMs);
    framesInFpsWindow = 0;
    fpsWindowStartedAt = now;
  }

  const uint32_t sampleInterval = deviceOn ? ActiveSampleIntervalMs : IdleSampleIntervalMs;
  if (now - lastImuSampleAt >= sampleInterval) {
    lastImuSampleAt = now;
    updateMotion(now);
  }

  if (!deviceOn) {
    blankMatrix();
    waitForInterrupt();
    return;
  }

  if (now - lastFrameAt >= FrameIntervalMs) {
    const uint32_t frameElapsedMs = now - lastFrameAt;
    lastFrameIntervalMs = frameElapsedMs;
    const float deltaSeconds = clampFloat(frameElapsedMs * 0.001f, 0.01f, 0.08f);
    lastFrameAt = now;
    const uint32_t simulationStartedAt = micros();
    stepFluid(deltaSeconds);
    lastSolverTimeUs = micros() - simulationStartedAt;
    renderFluid();
    ++framesInFpsWindow;
  }

  

  waitForInterrupt();
}