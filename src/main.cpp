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
constexpr float InitialFluidStartColumn = (FluidWidth - InitialFluidWidth) * 0.5f;
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
constexpr float PicBlend = 0.10f;
constexpr float Pi = 3.14159265f;
constexpr float ImuForceScale = 1.0f;
constexpr float SimulationHeightMeters = 0.35f;
constexpr float AccelerationPerG = 9.81f * FluidHeight / SimulationHeightMeters;
constexpr float PhysicalCellMeters = 0.002032f;
constexpr float RotationAccelerationScale = PhysicalCellMeters * FluidHeight / SimulationHeightMeters;
constexpr float MaximumAcceleration = AccelerationPerG * 4.0f;
constexpr float MaximumParticleSpeed = 40.0f;
constexpr float MaximumAdvectiveTravel = 0.75f;
constexpr uint8_t MaximumPhysicsSubsteps = 4;
constexpr float MaximumSubstepSeconds = MaximumAdvectiveTravel / MaximumParticleSpeed;
constexpr float ParticleRadius = 0.26f;
constexpr float MinimumParticleDistance = 2.0f * ParticleRadius;
constexpr uint8_t ImuControl1Register = 0x10;
constexpr uint8_t ImuControl2Register = 0x11;
constexpr uint8_t ImuAddress = 0x6A;
constexpr uint32_t FrameIntervalMs = 12;
constexpr uint32_t ActiveSampleIntervalMs = 20;
constexpr uint32_t IdleSampleIntervalMs = 40;
constexpr uint32_t DebugPrintIntervalMs = 250;
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

struct GridVelocitySample {
  float velocity;
  float correction;
};

struct BodyAcceleration {
  float x;
  float y;
};

struct BodyVelocity {
  float x;
  float y;
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
float angularVelocityZ = 0.0f;
float angularAccelerationZ = 0.0f;
uint32_t lastGyroSampleAt = 0;
uint32_t lastAccelerationSampleAt = 0;
uint8_t lastPhysicsSubsteps = 0;
float lastAdvectiveTravelSquared = 0.0f;
float lastDiscardedPhysicsSeconds = 0.0f;

float clampFloat(float value, float minimum, float maximum) {
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

uint16_t pixelIndex(uint8_t column, uint8_t row) {
  const uint8_t physicalRow = MatrixHeight - 1U - row;
  const uint8_t physicalColumn = MatrixWidth - 1U - column;
#if defined(MATRIX_SERPENTINE) && MATRIX_SERPENTINE
  const uint8_t mappedColumn = (physicalRow & 1U) ? (MatrixWidth - 1U - physicalColumn) : physicalColumn;
  return static_cast<uint16_t>(physicalRow) * MatrixWidth + mappedColumn;
#else
  return static_cast<uint16_t>(physicalRow) * MatrixWidth + physicalColumn;
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

void resetDynamicMotion() {
  linearX = 0.0f;
  linearY = 0.0f;
  angularVelocityZ = 0.0f;
  angularAccelerationZ = 0.0f;
  gyroInitialized = false;
  lastGyroSampleAt = 0;
}

float updateAccelerationEstimate(float sampleX, float sampleY, float sampleZ, float sampleSeconds) {
  if (!gravityInitialized) {
    gravityX = sampleX;
    gravityY = sampleY;
    gravityZ = sampleZ;
    gravityInitialized = true;
  }
  const float filterAlpha = sampleSeconds / (0.25f + sampleSeconds);
  gravityX += (sampleX - gravityX) * filterAlpha;
  gravityY += (sampleY - gravityY) * filterAlpha;
  gravityZ += (sampleZ - gravityZ) * filterAlpha;
  linearX = sampleX - gravityX;
  linearY = sampleY - gravityY;
  const float linearZ = sampleZ - gravityZ;
  motionScore = sqrtf(linearX * linearX + linearY * linearY + linearZ * linearZ);
  return linearZ;
}

void updateGyroscopeEstimate(float degreesPerSecond, float sampleSeconds) {
  const float measuredRate = fabsf(degreesPerSecond) < 1.0f ? 0.0f : degreesPerSecond * (Pi / 180.0f);
  if (!gyroInitialized) {
    angularVelocityZ = measuredRate;
    angularAccelerationZ = 0.0f;
    gyroInitialized = true;
    return;
  }
  const float rateAlpha = sampleSeconds / (0.035f + sampleSeconds);
  const float filteredRate = angularVelocityZ + (measuredRate - angularVelocityZ) * rateAlpha;
  const float measuredAcceleration = clampFloat((filteredRate - angularVelocityZ) / sampleSeconds, -100.0f, 100.0f);
  const float accelerationAlpha = sampleSeconds / (0.04f + sampleSeconds);
  angularAccelerationZ += (measuredAcceleration - angularAccelerationZ) * accelerationAlpha;
  angularVelocityZ = filteredRate;
}

bool beginImu() {
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 4;
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
  if (!writeImuRegister(ImuControl1Register, rateBits | 0x08) ||
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
    cellDensity[index] = 0.0f;
  }

  for (const FluidParticle& particle : particles) {
    const uint8_t column = static_cast<uint8_t>(clampFloat(particle.x, 0.0f, FluidWidth - 1.0f));
    const uint8_t row = static_cast<uint8_t>(clampFloat(particle.y, 0.0f, FluidHeight - 1.0f));
    const uint16_t index = row * FluidWidth + column;
    ++cellParticleCount[index];
    const float densityColumn = clampFloat(particle.x - 0.5f, 0.0f, FluidWidth - 1.001f);
    const float densityRow = clampFloat(particle.y - 0.5f, 0.0f, FluidHeight - 1.001f);
    const uint8_t column0 = static_cast<uint8_t>(densityColumn);
    const uint8_t row0 = static_cast<uint8_t>(densityRow);
    const float fractionX = densityColumn - column0;
    const float fractionY = densityRow - row0;
    const float particleDensity = 1.0f / ParticlesPerCell;
    cellDensity[row0 * FluidWidth + column0] += (1.0f - fractionX) * (1.0f - fractionY) * particleDensity;
    cellDensity[row0 * FluidWidth + column0 + 1U] += fractionX * (1.0f - fractionY) * particleDensity;
    cellDensity[(row0 + 1U) * FluidWidth + column0] += (1.0f - fractionX) * fractionY * particleDensity;
    cellDensity[(row0 + 1U) * FluidWidth + column0 + 1U] += fractionX * fractionY * particleDensity;
  }
}

void seedFluid() {
  for (uint16_t index = 0; index < UGridCount; ++index) {
    uGrid[index] = 0.0f;
  }
  for (uint16_t index = 0; index < VGridCount; ++index) {
    vGrid[index] = 0.0f;
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
          particle.velocityX = 0.0f;
          particle.velocityY = 0.0f;
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
  static_assert(ParticleCount < 255U && FluidCellCount < 255U, "Particle bins require byte-sized indices");
  constexpr uint8_t noParticle = 255U;
  uint8_t cellHead[FluidCellCount] = {};
  uint8_t nextParticle[ParticleCount] = {};
  uint8_t particleCell[ParticleCount] = {};
  const float minimumDistanceSquared = MinimumParticleDistance * MinimumParticleDistance;
  const float minimumColumn = ParticleRadius;
  const float maximumColumn = FluidWidth - ParticleRadius;
  const float minimumRow = ParticleRadius;
  const float maximumRow = FluidHeight - ParticleRadius;

  for (uint8_t iteration = 0; iteration < ParticleSeparationIterations; ++iteration) {
    for (uint16_t cell = 0; cell < FluidCellCount; ++cell) {
      cellHead[cell] = noParticle;
    }
    for (uint16_t index = 0; index < ParticleCount; ++index) {
      const uint8_t column = static_cast<uint8_t>(clampFloat(particles[index].x, 0.0f, FluidWidth - 1.0f));
      const uint8_t row = static_cast<uint8_t>(clampFloat(particles[index].y, 0.0f, FluidHeight - 1.0f));
      const uint8_t cell = row * FluidWidth + column;
      particleCell[index] = cell;
      nextParticle[index] = cellHead[cell];
      cellHead[cell] = static_cast<uint8_t>(index);
    }
    for (uint16_t first = 0; first < ParticleCount; ++first) {
      const uint8_t centerColumn = particleCell[first] % FluidWidth;
      const uint8_t centerRow = particleCell[first] / FluidWidth;
      const uint8_t firstColumn = centerColumn > 0 ? centerColumn - 1U : 0U;
      const uint8_t lastColumn = centerColumn + 1U < FluidWidth ? centerColumn + 1U : FluidWidth - 1U;
      const uint8_t firstRow = centerRow > 0 ? centerRow - 1U : 0U;
      const uint8_t lastRow = centerRow + 1U < FluidHeight ? centerRow + 1U : FluidHeight - 1U;
      for (uint8_t neighborRow = firstRow; neighborRow <= lastRow; ++neighborRow) {
        for (uint8_t neighborColumn = firstColumn; neighborColumn <= lastColumn; ++neighborColumn) {
          for (uint8_t second = cellHead[neighborRow * FluidWidth + neighborColumn];
               second != noParticle; second = nextParticle[second]) {
            if (second <= first) {
              continue;
            }
            FluidParticle& firstParticle = particles[first];
            FluidParticle& secondParticle = particles[second];
            float deltaX = secondParticle.x - firstParticle.x;
            float deltaY = secondParticle.y - firstParticle.y;
            float distanceSquared = deltaX * deltaX + deltaY * deltaY;
            if (distanceSquared >= minimumDistanceSquared) {
              continue;
            }
            if (distanceSquared < 0.000001f) {
              const uint32_t direction = ((first + 1U) * 73856093U ^ (second + 1U) * 19349663U) & 3U;
              deltaX = direction < 2U ? (direction == 0U ? 0.001f : -0.001f) : 0.0f;
              deltaY = direction >= 2U ? (direction == 2U ? 0.001f : -0.001f) : 0.0f;
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
      }
    }

    for (FluidParticle& particle : particles) {
      particle.x = clampFloat(particle.x, minimumColumn, maximumColumn);
      particle.y = clampFloat(particle.y, minimumRow, maximumRow);
    }
  }
}

GridVelocitySample sampleParticleVelocity(bool horizontal, const FluidParticle& particle) {
  const uint8_t gridWidth = horizontal ? UGridWidth : VGridWidth;
  const uint8_t gridHeight = horizontal ? UGridHeight : VGridHeight;
  const float column = clampFloat(particle.x - (horizontal ? 0.0f : 0.5f), 0.0f, gridWidth - 1.001f);
  const float row = clampFloat(particle.y - (horizontal ? 0.5f : 0.0f), 0.0f, gridHeight - 1.001f);
  const uint8_t columns[2] = {static_cast<uint8_t>(column), static_cast<uint8_t>(static_cast<uint8_t>(column) + 1U)};
  const uint8_t rows[2] = {static_cast<uint8_t>(row), static_cast<uint8_t>(static_cast<uint8_t>(row) + 1U)};
  const float columnWeights[2] = {1.0f - (column - columns[0]), column - columns[0]};
  const float rowWeights[2] = {1.0f - (row - rows[0]), row - rows[0]};
  const float* grid = horizontal ? uGrid : vGrid;
  const float* before = horizontal ? uGridBefore : vGridBefore;
  float totalWeight = 0.0f;
  float velocity = 0.0f;
  float correction = 0.0f;
  for (uint8_t rowCorner = 0; rowCorner < 2; ++rowCorner) {
    for (uint8_t columnCorner = 0; columnCorner < 2; ++columnCorner) {
      const uint8_t faceColumn = columns[columnCorner];
      const uint8_t faceRow = rows[rowCorner];
      const bool valid = horizontal
          ? (faceColumn == 0 || faceColumn == FluidWidth ||
             cellParticleCount[faceRow * FluidWidth + faceColumn - 1U] != 0 ||
             cellParticleCount[faceRow * FluidWidth + faceColumn] != 0)
          : (faceRow == 0 || faceRow == FluidHeight ||
             cellParticleCount[(faceRow - 1U) * FluidWidth + faceColumn] != 0 ||
             cellParticleCount[faceRow * FluidWidth + faceColumn] != 0);
      if (!valid) {
        continue;
      }
      const float weight = columnWeights[columnCorner] * rowWeights[rowCorner];
      const uint16_t index = faceRow * gridWidth + faceColumn;
      velocity += weight * grid[index];
      correction += weight * (grid[index] - before[index]);
      totalWeight += weight;
    }
  }
  if (totalWeight <= 0.000001f) {
    return {horizontal ? particle.velocityX : particle.velocityY, 0.0f};
  }
  return {velocity / totalWeight, correction / totalWeight};
}

void resolveParticleWalls(FluidParticle& particle) {
  const float maximumColumn = FluidWidth - ParticleRadius;
  const float maximumRow = FluidHeight - ParticleRadius;

  if (particle.x <= ParticleRadius) {
    particle.x = ParticleRadius;
    particle.velocityX = particle.velocityX > 0.0f ? particle.velocityX : 0.0f;
  } else if (particle.x >= maximumColumn) {
    particle.x = maximumColumn;
    particle.velocityX = particle.velocityX < 0.0f ? particle.velocityX : 0.0f;
  }
  if (particle.y <= ParticleRadius) {
    particle.y = ParticleRadius;
    particle.velocityY = particle.velocityY > 0.0f ? particle.velocityY : 0.0f;
  } else if (particle.y >= maximumRow) {
    particle.y = maximumRow;
    particle.velocityY = particle.velocityY < 0.0f ? particle.velocityY : 0.0f;
  }
}

BodyAcceleration bodyAccelerationAt(float column, float row, float velocityX, float velocityY) {
  const float relativeX = column - FluidWidth * 0.5f;
  const float relativeY = row - FluidHeight * 0.5f;
  const float spinSquared = angularVelocityZ * angularVelocityZ;
  const float rotationX = angularAccelerationZ * relativeY + spinSquared * relativeX +
                          2.0f * angularVelocityZ * velocityY;
  const float rotationY = -angularAccelerationZ * relativeX + spinSquared * relativeY -
                          2.0f * angularVelocityZ * velocityX;
  return {clampFloat(fluidAccelerationX + rotationX * RotationAccelerationScale, -MaximumAcceleration, MaximumAcceleration),
          clampFloat(fluidAccelerationY + rotationY * RotationAccelerationScale, -MaximumAcceleration, MaximumAcceleration)};
}

BodyVelocity integrateBodyVelocityAt(float column, float row, float velocityX, float velocityY, float deltaSeconds) {
  const BodyAcceleration nonCoriolisAcceleration = bodyAccelerationAt(column, row, 0.0f, 0.0f);
  const float halfRotation = angularVelocityZ * RotationAccelerationScale * deltaSeconds;
  const float reciprocal = 1.0f / (1.0f + halfRotation * halfRotation);
  const float rightHandX = velocityX + halfRotation * velocityY + nonCoriolisAcceleration.x * deltaSeconds;
  const float rightHandY = velocityY - halfRotation * velocityX + nonCoriolisAcceleration.y * deltaSeconds;
  return {(rightHandX + halfRotation * rightHandY) * reciprocal,
          (rightHandY - halfRotation * rightHandX) * reciprocal};
}

void stepFluidSubstep(float deltaSeconds) {
  transferParticlesToGrid();

  const float picBlend = 1.0f - powf(1.0f - PicBlend, deltaSeconds / (FrameIntervalMs * 0.001f));
  for (uint8_t row = 0; row < FluidHeight; ++row) {
    for (uint8_t column = 1; column < FluidWidth; ++column) {
      const uint16_t leftCell = row * FluidWidth + column - 1U;
      const uint16_t rightCell = leftCell + 1U;
      if (cellParticleCount[leftCell] || cellParticleCount[rightCell]) {
        const float velocityY = angularVelocityZ != 0.0f
            ? sampleGrid(vGridBefore, VGridWidth, VGridHeight, column - 0.5f, row + 0.5f) : 0.0f;
        const BodyVelocity velocity = integrateBodyVelocityAt(column, row + 0.5f,
                           uGridBefore[uIndex(column, row)], velocityY, deltaSeconds);
        uGrid[uIndex(column, row)] = velocity.x;
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
        const float velocityX = angularVelocityZ != 0.0f
            ? sampleGrid(uGridBefore, UGridWidth, UGridHeight, column + 0.5f, row - 0.5f) : 0.0f;
        const BodyVelocity velocity = integrateBodyVelocityAt(column + 0.5f, row,
                           velocityX, vGridBefore[vIndex(column, row)], deltaSeconds);
        vGrid[vIndex(column, row)] = velocity.y;
      } else {
        vGrid[vIndex(column, row)] = 0.0f;
      }
    }
  }

  enforceGridBoundaries();
  projectGridVelocity();

  for (FluidParticle& particle : particles) {
    const GridVelocitySample horizontalSample = sampleParticleVelocity(true, particle);
    const GridVelocitySample verticalSample = sampleParticleVelocity(false, particle);
    const float flipVelocityX = particle.velocityX + horizontalSample.correction;
    const float flipVelocityY = particle.velocityY + verticalSample.correction;
    particle.velocityX = flipVelocityX * (1.0f - picBlend) + horizontalSample.velocity * picBlend;
    particle.velocityY = flipVelocityY * (1.0f - picBlend) + verticalSample.velocity * picBlend;
    const float speedSquared = particle.velocityX * particle.velocityX + particle.velocityY * particle.velocityY;
    if (speedSquared > MaximumParticleSpeed * MaximumParticleSpeed) {
      const float speedScale = MaximumParticleSpeed / sqrtf(speedSquared);
      particle.velocityX *= speedScale;
      particle.velocityY *= speedScale;
    }
    const float travelSquared = (particle.velocityX * particle.velocityX +
                                  particle.velocityY * particle.velocityY) * deltaSeconds * deltaSeconds;
    lastAdvectiveTravelSquared = travelSquared > lastAdvectiveTravelSquared
        ? travelSquared : lastAdvectiveTravelSquared;
    particle.x += particle.velocityX * deltaSeconds;
    particle.y += particle.velocityY * deltaSeconds;
    resolveParticleWalls(particle);
  }

  separateParticles();
  for (FluidParticle& particle : particles) {
    resolveParticleWalls(particle);
  }
  rasterizeParticles();
}

void stepFluid(float deltaSeconds) {
  fluidAccelerationX = -ImuForceScale * AccelerationPerG * clampFloat(gravityY + linearY, -4.0f, 4.0f);
  fluidAccelerationY = ImuForceScale * AccelerationPerG * clampFloat(gravityX + linearX, -4.0f, 4.0f);
  lastPhysicsSubsteps = 0;
  lastAdvectiveTravelSquared = 0.0f;
  lastDiscardedPhysicsSeconds = 0.0f;
  if (!isfinite(deltaSeconds) || deltaSeconds <= 0.0f) {
    return;
  }
  const float boundedSeconds = clampFloat(deltaSeconds, 0.0f, MaximumSubstepSeconds * MaximumPhysicsSubsteps);
  lastDiscardedPhysicsSeconds = deltaSeconds - boundedSeconds;
  uint8_t substeps = static_cast<uint8_t>(ceilf(boundedSeconds / MaximumSubstepSeconds));
  substeps = substeps > MaximumPhysicsSubsteps ? MaximumPhysicsSubsteps : substeps;
  substeps = substeps == 0 ? 1U : substeps;
  const float substepSeconds = boundedSeconds / substeps;
  lastPhysicsSubsteps = substeps;
  for (uint8_t substep = 0; substep < substeps; ++substep) {
    stepFluidSubstep(substepSeconds);
  }
}

uint32_t colorForFluid(uint16_t index) {
  const float density = cellDensity[index];
  if (density <= 0.01f) {
    return 0;
  }
  const uint8_t green = static_cast<uint8_t>(clampFloat(density * 255.0f, 0.0f, 255.0f));
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
  resetDynamicMotion();
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
    resetDynamicMotion();
    gravityInitialized = false;
    lastAccelerationSampleAt = 0;
    motionScore = 0.0f;
    return;
  }
  if (!readAcceleration(sampleX, sampleY, sampleZ) ||
      !isfinite(sampleX) || !isfinite(sampleY) || !isfinite(sampleZ)) {
    if (Serial && now - lastDebugPrintAt >= 1000U) {
      Serial.println("IMU read failed");
      lastDebugPrintAt = now;
    }
    resetDynamicMotion();
    gravityInitialized = false;
    lastAccelerationSampleAt = 0;
    motionScore = 0.0f;
    return;
  }

  const float accelerationSampleSeconds = lastAccelerationSampleAt == 0 ? 0.02f
      : clampFloat((now - lastAccelerationSampleAt) * 0.001f, 0.001f, 0.2f);
  lastAccelerationSampleAt = now;
  const float linearZ = updateAccelerationEstimate(sampleX, sampleY, sampleZ, accelerationSampleSeconds);

  const uint16_t gyroErrorsBefore = imu.allOnesCounter + imu.nonSuccessCounter;
  gyroXDegreesPerSecond = imu.readFloatGyroX();
  gyroYDegreesPerSecond = imu.readFloatGyroY();
  gyroZDegreesPerSecond = imu.readFloatGyroZ();
  const uint16_t gyroErrorsAfter = imu.allOnesCounter + imu.nonSuccessCounter;
  if (gyroErrorsAfter != gyroErrorsBefore || !isfinite(gyroZDegreesPerSecond)) {
    gyroInitialized = false;
    angularVelocityZ = 0.0f;
    angularAccelerationZ = 0.0f;
  } else {
    if (lastGyroSampleAt != 0 && now - lastGyroSampleAt > 100U) {
      gyroInitialized = false;
    }
    const float gyroSampleSeconds = lastGyroSampleAt == 0 ? 0.02f
        : clampFloat((now - lastGyroSampleAt) * 0.001f, 0.001f, 0.1f);
    updateGyroscopeEstimate(gyroZDegreesPerSecond, gyroSampleSeconds);
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
    Serial.print(" omega_z=");
    Serial.print(angularVelocityZ, 3);
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
    Serial.print(" physics_steps=");
    Serial.print(lastPhysicsSubsteps);
    Serial.print(" advect_cells=");
    Serial.print(sqrtf(lastAdvectiveTravelSquared), 3);
    Serial.print(" shake=");
    Serial.print(shakeArmed ? "armed" : "cooldown");
    Serial.print(" display=");
    Serial.println(deviceOn ? "on" : "off");
  }

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

#if defined(FLUID_SIM_SELF_TEST)
bool reportSelfTest(const char* name, bool passed) {
  Serial.print(passed ? "PASS " : "FAIL ");
  Serial.println(name);
  return passed;
}

float meanParticleX() {
  float sum = 0.0f;
  for (const FluidParticle& particle : particles) {
    sum += particle.x;
  }
  return sum / ParticleCount;
}

float meanParticleY() {
  float sum = 0.0f;
  for (const FluidParticle& particle : particles) {
    sum += particle.y;
  }
  return sum / ParticleCount;
}

float maximumParticleSpeed() {
  float maximumSpeedSquared = 0.0f;
  for (const FluidParticle& particle : particles) {
    const float speedSquared = particle.velocityX * particle.velocityX +
                               particle.velocityY * particle.velocityY;
    maximumSpeedSquared = speedSquared > maximumSpeedSquared ? speedSquared : maximumSpeedSquared;
  }
  return sqrtf(maximumSpeedSquared);
}

bool particlesWithinBounds() {
  const float maximumColumn = FluidWidth - ParticleRadius;
  const float maximumRow = FluidHeight - ParticleRadius;
  for (const FluidParticle& particle : particles) {
    if (!(particle.x >= ParticleRadius && particle.x <= maximumColumn &&
          particle.y >= ParticleRadius && particle.y <= maximumRow &&
          isfinite(particle.velocityX) && isfinite(particle.velocityY))) {
      return false;
    }
  }
  return true;
}

uint16_t connectedParticleCount(uint16_t startingParticle) {
  bool visited[ParticleCount] = {};
  uint16_t pending[ParticleCount] = {};
  uint16_t readPosition = 0;
  uint16_t writePosition = 1;
  pending[0] = startingParticle;
  visited[startingParticle] = true;
  const float connectionDistance = MinimumParticleDistance * 1.6f;
  while (readPosition < writePosition) {
    const FluidParticle& current = particles[pending[readPosition++]];
    for (uint16_t candidate = 0; candidate < ParticleCount; ++candidate) {
      if (visited[candidate]) {
        continue;
      }
      const float deltaX = particles[candidate].x - current.x;
      const float deltaY = particles[candidate].y - current.y;
      if (deltaX * deltaX + deltaY * deltaY <= connectionDistance * connectionDistance) {
        visited[candidate] = true;
        pending[writePosition++] = candidate;
      }
    }
  }
  return writePosition;
}

void prepareSelfTestPool(float sensorGravityX, float sensorGravityY, bool startAtTop = true) {
  seedFluid();
  for (FluidParticle& particle : particles) {
    if (startAtTop) {
      particle.y = clampFloat(particle.y - InitialFluidStartRow, ParticleRadius,
                              FluidHeight - ParticleRadius);
    }
    particle.velocityX = 0.0f;
    particle.velocityY = 0.0f;
  }
  gravityX = sensorGravityX;
  gravityY = sensorGravityY;
  gravityZ = 0.0f;
  linearX = 0.0f;
  linearY = 0.0f;
  angularVelocityZ = 0.0f;
  angularAccelerationZ = 0.0f;
  rasterizeParticles();
}

void waitForMappingAcknowledgement() {
  while (true) {
    if (Serial.available() > 0 && Serial.read() == 'N') {
      return;
    }
    delay(1);
  }
}

void runDisplayMappingChecks() {
  pixels.clear();
  for (uint8_t column = 0; column < MatrixWidth; ++column) {
    pixels.setPixelColor(pixelIndex(column, MatrixHeight - 1U), pixels.Color(0, 160, 0));
  }
  pixels.show();
  Serial.println("MAP_EXPECT_BOTTOM");
  waitForMappingAcknowledgement();

  pixels.clear();
  for (uint8_t row = 0; row < MatrixHeight; ++row) {
    pixels.setPixelColor(pixelIndex(0, row), pixels.Color(0, 160, 0));
  }
  pixels.show();
  Serial.println("MAP_EXPECT_LEFT");
  waitForMappingAcknowledgement();
  pixels.clear();
  pixels.show();
  Serial.println("MAP_DONE");
}

bool runFluidSimulationSelfTests() {
  bool allPassed = true;
  Serial.println("SIMTEST_BEGIN");

  const bool rowMappingCorrect = pixelIndex(0, 0) == LedCount - 1U &&
                                 pixelIndex(0, MatrixHeight - 1U) == MatrixWidth - 1U &&
                                 pixelIndex(MatrixWidth - 1U, MatrixHeight - 1U) == 0;
  allPassed = reportSelfTest("display_axes_match_verified_edges", rowMappingCorrect) && allPassed;

  FluidParticle sideCollision = {ParticleRadius - 0.1f, 3.0f, -4.0f, 2.5f};
  resolveParticleWalls(sideCollision);
  const bool sideWallSlides = fabsf(sideCollision.x - ParticleRadius) < 0.001f &&
                              fabsf(sideCollision.velocityX) < 0.001f &&
                              fabsf(sideCollision.velocityY - 2.5f) < 0.001f;
  allPassed = reportSelfTest("side_wall_blocks_normal_without_bounce_or_friction", sideWallSlides) && allPassed;

  FluidParticle inwardCollision = {ParticleRadius - 0.1f, 3.0f, 2.0f, 2.5f};
  resolveParticleWalls(inwardCollision);
  allPassed = reportSelfTest("wall_preserves_velocity_away_from_surface",
                             fabsf(inwardCollision.velocityX - 2.0f) < 0.001f) && allPassed;

  prepareSelfTestPool(0.8f, -0.6f);
  stepFluid(0.0f);
  const bool tiltedGravityProjectsToBothAxes = fabsf(fluidAccelerationX / AccelerationPerG - 0.6f) < 0.001f &&
                                               fabsf(fluidAccelerationY / AccelerationPerG - 0.8f) < 0.001f;
  allPassed = reportSelfTest("tilted_gravity_projects_to_both_axes",
                             tiltedGravityProjectsToBothAxes) && allPassed;

  prepareSelfTestPool(1.0f, 0.0f);
  stepFluid(0.0f);
  const bool accelerometerForceIsBaseline = fabsf(fluidAccelerationY - AccelerationPerG) < 0.001f &&
                                            fabsf(fluidAccelerationX) < 0.001f;
  allPassed = reportSelfTest("one_g_uses_explicit_simulation_length_units",
                             accelerometerForceIsBaseline) && allPassed;

  prepareSelfTestPool(0.0f, 0.0f);
  linearX = 0.2f;
  linearY = -0.1f;
  stepFluid(0.0f);
  const bool lateralMotionProjectsCorrectly = fabsf(fluidAccelerationX / AccelerationPerG - 0.1f) < 0.001f &&
                                              fabsf(fluidAccelerationY / AccelerationPerG - 0.2f) < 0.001f;
  allPassed = reportSelfTest("linear_acceleration_projects_to_both_axes",
                             lateralMotionProjectsCorrectly) && allPassed;

  prepareSelfTestPool(1.0f, 0.0f);
  stepFluid(0.0f);
  const float initialMeanX = meanParticleX();
  const float initialMeanY = meanParticleY();
  uint32_t maximumStepTimeUs = 0;
  uint32_t maximumFrameTimeUs = 0;
  uint32_t totalFrameTimeUs = 0;
  for (uint16_t frame = 0; frame < 100; ++frame) {
    const uint32_t frameStartedAt = micros();
    const uint32_t stepStartedAt = frameStartedAt;
    stepFluid(FrameIntervalMs * 0.001f);
    const uint32_t stepTimeUs = micros() - stepStartedAt;
    maximumStepTimeUs = stepTimeUs > maximumStepTimeUs ? stepTimeUs : maximumStepTimeUs;
    renderFluid();
    const uint32_t frameTimeUs = micros() - frameStartedAt;
    maximumFrameTimeUs = frameTimeUs > maximumFrameTimeUs ? frameTimeUs : maximumFrameTimeUs;
    totalFrameTimeUs += frameTimeUs;
  }
  const float horizontalDrift = fabsf(meanParticleX() - initialMeanX);
  const float downwardTravel = meanParticleY() - initialMeanY;
  const uint32_t averageFrameTimeUs = totalFrameTimeUs / 100U;
  const bool gravityMovesDown = downwardTravel > 0.5f;
  const bool gravityDoesNotPushSideways = horizontalDrift < 0.25f;
  allPassed = reportSelfTest("gravity_moves_pool_down", gravityMovesDown) && allPassed;
  allPassed = reportSelfTest("downward_gravity_has_low_side_drift", gravityDoesNotPushSideways) && allPassed;
  allPassed = reportSelfTest("particles_remain_inside_walls", particlesWithinBounds()) && allPassed;

  seedFluid();
  gravityX = 0.0f;
  gravityY = 0.0f;
  linearX = 0.0f;
  linearY = 0.0f;
  angularAccelerationZ = 0.0f;
  const float initialFlowSpeed = maximumParticleSpeed();
  allPassed = reportSelfTest("startup_does_not_inject_artificial_swirl", initialFlowSpeed < 0.001f) && allPassed;
  for (uint8_t frame = 0; frame < 40; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
  }
  const float retainedFlowSpeed = maximumParticleSpeed();
  const bool flowRemainsQuiet = retainedFlowSpeed < 0.001f;
  allPassed = reportSelfTest("no_idle_force_creates_motion", flowRemainsQuiet) && allPassed;

  seedFluid();
  for (FluidParticle& particle : particles) {
    particle.velocityX = 0.0f;
    particle.velocityY = 0.0f;
  }
  gravityX = -1.0f;
  gravityY = 0.0f;
  linearX = 0.0f;
  linearY = 0.0f;
  angularAccelerationZ = 0.0f;
  const uint16_t responseFrames = (1000U + FrameIntervalMs - 1U) / FrameIntervalMs;
  float inversionResponseSeconds = -1.0f;
  for (uint16_t frame = 0; frame < responseFrames; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
    uint16_t arrived = 0;
    for (const FluidParticle& particle : particles) {
      arrived += particle.y < 3.5f ? 1U : 0U;
    }
    if (inversionResponseSeconds < 0.0f && arrived * 5U >= ParticleCount * 4U) {
      inversionResponseSeconds = (frame + 1U) * FrameIntervalMs * 0.001f;
    }
  }
  uint16_t reversedParticleCount = 0;
  for (const FluidParticle& particle : particles) {
    reversedParticleCount += particle.y < 3.5f ? 1U : 0U;
  }
  allPassed = reportSelfTest("inversion_moves_80_percent_to_new_bottom_in_one_second",
                             reversedParticleCount * 5U >= ParticleCount * 4U) && allPassed;
  allPassed = reportSelfTest("inversion_80_percent_response_is_under_600ms",
                             inversionResponseSeconds >= 0.0f && inversionResponseSeconds <= 0.6f) && allPassed;

  for (uint16_t frame = 0; frame < responseFrames; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
  }
  uint16_t settledArrivalCount = 0;
  float settledSpeedSquared = 0.0f;
  for (const FluidParticle& particle : particles) {
    settledArrivalCount += particle.y < 4.0f ? 1U : 0U;
    settledSpeedSquared += particle.velocityX * particle.velocityX + particle.velocityY * particle.velocityY;
  }
  const float settledRmsSpeed = sqrtf(settledSpeedSquared / ParticleCount);
  allPassed = reportSelfTest("inversion_leaves_no_stranded_tail_after_two_seconds",
                             settledArrivalCount >= ParticleCount - 1U) && allPassed;
  allPassed = reportSelfTest("stationary_pool_settles_without_velocity_jitter",
                             settledRmsSpeed < 1.0f) && allPassed;

  seedFluid();
  for (FluidParticle& particle : particles) {
    particle.velocityX = 0.0f;
    particle.velocityY = 0.0f;
  }
  particles[0].x = ParticleRadius + 0.001f;
  particles[0].y = 1.0f;
  gravityX = 1.0f;
  gravityY = 0.0f;
  for (uint16_t frame = 0; frame < responseFrames; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
  }
  allPassed = reportSelfTest("wall_droplet_rejoins_pool_in_one_second",
                             particles[0].y > FluidHeight - 3.5f) && allPassed;
  Serial.print("SIMTEST inversion_arrivals=");
  Serial.print(reversedParticleCount);
  Serial.print(" response_seconds=");
  Serial.print(inversionResponseSeconds, 3);
  Serial.print(" wall_droplet_y=");
  Serial.println(particles[0].y, 2);

  prepareSelfTestPool(0.0f, 0.0f, false);
  particles[0] = {2.9f, 1.4f, 0.0f, 2.0f};
  stepFluid(0.001f);
  const float freeSurfaceRetainedVelocity = particles[0].velocityY;
  allPassed = reportSelfTest("isolated_droplet_has_no_air_interpolation_drag",
                             fabsf(freeSurfaceRetainedVelocity - 2.0f) < 0.05f) && allPassed;

  prepareSelfTestPool(1.0f, 0.0f, false);
  particles[0] = {2.9f, 1.4f, 0.0f, 0.0f};
  stepFluid(FrameIntervalMs * 0.001f);
  const float freeSurfaceGravityVelocity = particles[0].velocityY;
  const float expectedGravityVelocity = AccelerationPerG * FrameIntervalMs * 0.001f;
  allPassed = reportSelfTest("isolated_droplet_receives_full_gravity",
                             fabsf(freeSurfaceGravityVelocity - expectedGravityVelocity) < expectedGravityVelocity * 0.05f) && allPassed;

  prepareSelfTestPool(0.7071068f, 0.7071068f, false);
  particles[0] = {ParticleRadius + 0.001f, 1.0f, 0.0f, 0.0f};
  for (uint16_t frame = 0; frame < responseFrames; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
  }
  const float leftWallDrainY = particles[0].y;
  const uint16_t leftWallConnectedCount = connectedParticleCount(0);
  allPassed = reportSelfTest("left_wall_droplet_drains_under_diagonal_gravity",
                             leftWallConnectedCount * 5U >= ParticleCount * 4U) && allPassed;

  prepareSelfTestPool(0.7071068f, -0.7071068f, false);
  particles[0] = {FluidWidth - ParticleRadius - 0.001f, 1.0f, 0.0f, 0.0f};
  for (uint16_t frame = 0; frame < responseFrames; ++frame) {
    stepFluid(FrameIntervalMs * 0.001f);
  }
  const float rightWallDrainY = particles[0].y;
  const uint16_t rightWallConnectedCount = connectedParticleCount(0);
  allPassed = reportSelfTest("right_wall_droplet_drains_under_diagonal_gravity",
                             rightWallConnectedCount * 5U >= ParticleCount * 4U) && allPassed;
  Serial.print("SIMTEST settling_arrivals=");
  Serial.print(settledArrivalCount);
  Serial.print(" settling_rms_speed=");
  Serial.print(settledRmsSpeed, 3);
  Serial.print(" isolated_velocity=");
  Serial.print(freeSurfaceRetainedVelocity, 3);
  Serial.print(" gravity_velocity=");
  Serial.print(freeSurfaceGravityVelocity, 3);
  Serial.print(" expected_gravity_velocity=");
  Serial.print(expectedGravityVelocity, 3);
  Serial.print(" left_wall_y=");
  Serial.print(leftWallDrainY, 2);
  Serial.print(" right_wall_y=");
  Serial.println(rightWallDrainY, 2);
  Serial.print("SIMTEST left_wall_connected=");
  Serial.print(leftWallConnectedCount);
  Serial.print(" right_wall_connected=");
  Serial.println(rightWallConnectedCount);

  prepareSelfTestPool(0.0f, 0.0f, false);
  particles[0] = {2.99f, 1.5f, 0.0f, 0.0f};
  rasterizeParticles();
  float previousDensity[FluidCellCount] = {};
  for (uint16_t index = 0; index < FluidCellCount; ++index) {
    previousDensity[index] = cellDensity[index];
  }
  particles[0].x = 3.01f;
  rasterizeParticles();
  float densityChange = 0.0f;
  float renderedMass = 0.0f;
  for (uint16_t index = 0; index < FluidCellCount; ++index) {
    densityChange += fabsf(cellDensity[index] - previousDensity[index]);
    renderedMass += cellDensity[index];
  }
  allPassed = reportSelfTest("density_is_continuous_across_cell_boundaries", densityChange < 0.05f) && allPassed;
  allPassed = reportSelfTest("density_splat_preserves_particle_mass",
                             fabsf(renderedMass - static_cast<float>(ParticleCount) / ParticlesPerCell) < 0.001f) && allPassed;

  prepareSelfTestPool(0.0f, 0.0f, false);
  particles[0] = {3.0f, 1.5f, 0.0f, 0.0f};
  particles[1] = particles[0];
  separateParticles();
  const float coincidentDeltaX = particles[1].x - particles[0].x;
  const float coincidentDeltaY = particles[1].y - particles[0].y;
  allPassed = reportSelfTest("coincident_particles_separate_without_nan",
                             coincidentDeltaX * coincidentDeltaX + coincidentDeltaY * coincidentDeltaY >
                             MinimumParticleDistance * MinimumParticleDistance * 0.9f && particlesWithinBounds()) && allPassed;

  gravityInitialized = false;
  updateAccelerationEstimate(1.0f, 0.0f, 0.0f, 0.02f);
  updateAccelerationEstimate(0.6f, 0.8f, 0.0f, 0.02f);
  allPassed = reportSelfTest("filtered_gravity_and_linear_reconstruct_latest_sample",
                             fabsf(gravityX + linearX - 0.6f) < 0.00001f &&
                             fabsf(gravityY + linearY - 0.8f) < 0.00001f) && allPassed;

  resetDynamicMotion();
  updateGyroscopeEstimate(0.6f, 0.02f);
  updateGyroscopeEstimate(-0.5f, 0.02f);
  allPassed = reportSelfTest("stationary_gyro_noise_does_not_stir_fluid",
                             angularVelocityZ == 0.0f && angularAccelerationZ == 0.0f) && allPassed;

  fluidAccelerationX = 0.0f;
  fluidAccelerationY = 0.0f;
  angularVelocityZ = 0.0f;
  angularAccelerationZ = 3.0f;
  const BodyAcceleration eulerForce = bodyAccelerationAt(FluidWidth * 0.5f + 1.0f,
                                                         FluidHeight * 0.5f, 0.0f, 0.0f);
  allPassed = reportSelfTest("rotation_acceleration_opposes_container_spin",
                             fabsf(eulerForce.x) < 0.001f && fabsf(eulerForce.y + 3.0f * RotationAccelerationScale) < 0.001f) && allPassed;
  angularAccelerationZ = 0.0f;
  angularVelocityZ = 2.0f;
  const BodyAcceleration centrifugalForce = bodyAccelerationAt(FluidWidth * 0.5f + 1.0f,
                                                               FluidHeight * 0.5f, 0.0f, 0.0f);
  allPassed = reportSelfTest("steady_spin_has_centrifugal_response",
                             fabsf(centrifugalForce.x - 4.0f * RotationAccelerationScale) < 0.001f && fabsf(centrifugalForce.y) < 0.001f) && allPassed;
  const BodyAcceleration coriolisForce = bodyAccelerationAt(FluidWidth * 0.5f,
                                                            FluidHeight * 0.5f, 1.0f, 0.0f);
  allPassed = reportSelfTest("moving_fluid_has_coriolis_response",
                             fabsf(coriolisForce.x) < 0.001f && fabsf(coriolisForce.y + 4.0f * RotationAccelerationScale) < 0.001f) && allPassed;
  angularVelocityZ = 8.0f;
  BodyVelocity energyCheck = {3.0f, 4.0f};
  for (uint8_t frame = 0; frame < 100; ++frame) {
    energyCheck = integrateBodyVelocityAt(FluidWidth * 0.5f, FluidHeight * 0.5f,
                                          energyCheck.x, energyCheck.y, FrameIntervalMs * 0.001f);
  }
  const float coriolisEnergyError = fabsf(energyCheck.x * energyCheck.x + energyCheck.y * energyCheck.y - 25.0f);
  allPassed = reportSelfTest("coriolis_integration_does_not_create_kinetic_energy",
                             coriolisEnergyError < 0.01f) && allPassed;
  linearX = 2.0f;
  linearY = -2.0f;
  resetDynamicMotion();
  allPassed = reportSelfTest("sensor_reset_clears_stale_motion_impulses",
                             linearX == 0.0f && linearY == 0.0f && angularVelocityZ == 0.0f &&
                             angularAccelerationZ == 0.0f && !gyroInitialized) && allPassed;

  prepareSelfTestPool(1.0f, 0.0f, false);
  const FluidParticle beforeZeroStep = particles[0];
  stepFluid(0.0f);
  stepFluid(-0.1f);
  allPassed = reportSelfTest("nonpositive_timestep_does_not_move_particles",
                             particles[0].x == beforeZeroStep.x && particles[0].y == beforeZeroStep.y &&
                             particles[0].velocityX == beforeZeroStep.velocityX &&
                             particles[0].velocityY == beforeZeroStep.velocityY) && allPassed;
  for (FluidParticle& particle : particles) {
    particle.velocityX = MaximumParticleSpeed;
    particle.velocityY = MaximumParticleSpeed;
  }
  stepFluid(0.05f);
  allPassed = reportSelfTest("delayed_frame_respects_particle_travel_limit",
                             lastPhysicsSubsteps > 1U &&
                             lastAdvectiveTravelSquared <= MaximumAdvectiveTravel * MaximumAdvectiveTravel + 0.0001f &&
                             particlesWithinBounds()) && allPassed;
  stepFluid(0.5f);
  allPassed = reportSelfTest("long_stall_has_bounded_catchup_work",
                             lastPhysicsSubsteps <= MaximumPhysicsSubsteps &&
                             lastDiscardedPhysicsSeconds > 0.0f && particlesWithinBounds()) && allPassed;

  prepareSelfTestPool(1.0f, 0.0f);
  angularVelocityZ = 8.0f;
  angularAccelerationZ = 40.0f;
  uint32_t maximumRotatingFrameUs = 0;
  float maximumRotatingJumpSquared = 0.0f;
  float maximumRotatingMassError = 0.0f;
  bool rotatingParticleCountStable = true;
  float previousParticleX[ParticleCount] = {};
  float previousParticleY[ParticleCount] = {};
  for (uint8_t frame = 0; frame < 40; ++frame) {
    for (uint16_t index = 0; index < ParticleCount; ++index) {
      previousParticleX[index] = particles[index].x;
      previousParticleY[index] = particles[index].y;
    }
    const uint32_t frameStartedAt = micros();
    stepFluid(FrameIntervalMs * 0.001f);
    renderFluid();
    const uint32_t frameTimeUs = micros() - frameStartedAt;
    maximumRotatingFrameUs = frameTimeUs > maximumRotatingFrameUs ? frameTimeUs : maximumRotatingFrameUs;
    float rotatingMass = 0.0f;
    uint16_t rotatingParticleCount = 0;
    for (uint16_t cell = 0; cell < FluidCellCount; ++cell) {
      rotatingMass += cellDensity[cell];
      rotatingParticleCount += cellParticleCount[cell];
    }
    const float massError = fabsf(rotatingMass - static_cast<float>(ParticleCount) / ParticlesPerCell);
    maximumRotatingMassError = massError > maximumRotatingMassError ? massError : maximumRotatingMassError;
    rotatingParticleCountStable = rotatingParticleCountStable && rotatingParticleCount == ParticleCount;
    for (uint16_t index = 0; index < ParticleCount; ++index) {
      const float deltaX = particles[index].x - previousParticleX[index];
      const float deltaY = particles[index].y - previousParticleY[index];
      const float jumpSquared = deltaX * deltaX + deltaY * deltaY;
      maximumRotatingJumpSquared = jumpSquared > maximumRotatingJumpSquared ? jumpSquared : maximumRotatingJumpSquared;
    }
  }
  allPassed = reportSelfTest("rotating_rendered_frame_meets_interval",
                             maximumRotatingFrameUs < FrameIntervalMs * 1000U && particlesWithinBounds()) && allPassed;
  allPassed = reportSelfTest("fast_spin_cannot_create_particles_or_rendered_mass",
                             rotatingParticleCountStable && maximumRotatingMassError < 0.001f) && allPassed;
  allPassed = reportSelfTest("fast_spin_has_no_particle_teleport_over_one_cell",
                             maximumRotatingJumpSquared < 1.0f) && allPassed;
  Serial.print("SIMTEST max_rotating_frame_us=");
  Serial.println(maximumRotatingFrameUs);
  Serial.print("SIMTEST spin_max_jump_cells=");
  Serial.print(sqrtf(maximumRotatingJumpSquared), 3);
  Serial.print(" spin_mass_error=");
  Serial.print(maximumRotatingMassError, 6);
  Serial.print(" coriolis_energy_error=");
  Serial.println(coriolisEnergyError, 6);

  Serial.print("SIMTEST downward_cells=");
  Serial.print(downwardTravel, 2);
  Serial.print(" side_drift_cells=");
  Serial.print(horizontalDrift, 2);
  Serial.print(" max_step_us=");
  Serial.println(maximumStepTimeUs);
  Serial.print(" avg_frame_us=");
  Serial.print(averageFrameTimeUs);
  Serial.print(" max_frame_us=");
  Serial.println(maximumFrameTimeUs);
  Serial.print(" initial_flow_speed=");
  Serial.print(initialFlowSpeed, 2);
  Serial.print(" retained_flow_speed=");
  Serial.println(retainedFlowSpeed, 2);
  allPassed = reportSelfTest("solver_step_fits_frame_budget",
                             maximumStepTimeUs < FrameIntervalMs * 1000U) && allPassed;
  allPassed = reportSelfTest("rendered_frame_meets_interval",
                             averageFrameTimeUs < FrameIntervalMs * 1000U &&
                             maximumFrameTimeUs < FrameIntervalMs * 1000U) && allPassed;
  return allPassed;
}
#endif

}  // namespace

void setup() {
  Serial.begin(115200);
#if defined(FLUID_SIM_SELF_TEST)
  while (!Serial) {
    delay(10);
  }
  pixels.begin();
  pixels.setBrightness(LedBrightness);
  pixels.clear();
  pixels.show();
  Serial.println("SIMTEST_READY");
  while (Serial.available() == 0) {
    delay(1);
  }
  if (Serial.read() == 'M') {
    runDisplayMappingChecks();
  } else {
    const bool passed = runFluidSimulationSelfTests();
    Serial.println(passed ? "SIMTEST_PASS" : "SIMTEST_FAIL");
  }
  while (true) {
    delay(1000);
  }
#else
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
#endif
}

void loop() {
#if defined(FLUID_SIM_SELF_TEST)
  delay(1000);
#else
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
    const float deltaSeconds = frameElapsedMs * 0.001f;
    lastFrameAt = now;
    const uint32_t simulationStartedAt = micros();
    stepFluid(deltaSeconds);
    lastSolverTimeUs = micros() - simulationStartedAt;
    renderFluid();
    ++framesInFpsWindow;
  }

  

  waitForInterrupt();
#endif
}