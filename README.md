# Fluid Simulation Pendant

Firmware for a Seeed Studio XIAO nRF52840 Sense Plus and the Seeed 6x10 WS2812 RGB Matrix. The LSM6DS3TR-C drives a FLIP simulation over the full 6x10 grid at a 100 FPS target. It starts with a 3x6 fluid pool at the bottom, rendered in one green color.

The solver transfers particles to a staggered velocity grid, applies gravity/accelerometer and gyro-derived rotational forces at an 8x flow-speed scale, projects the grid to a divergence-free field with free-surface Jacobi pressure iterations, transfers velocity back with a 95% FLIP / 5% PIC blend, and separates overlapping particles twice per frame. Particle centers stay one particle radius from the walls to reduce corner clumping.

## Build and upload

1. Install PlatformIO in VS Code.
2. Open this folder as the PlatformIO project.
3. Build and upload the `seeed_xiao_nrf52840` environment.

The firmware uses the Arduino framework, Adafruit NeoPixel, and Seeed's LSM6DS3 library. The Seeed driver handles the Sense Plus IMU's internal I2C bus and sensor power pin.

Open the PlatformIO Serial Monitor at 115200 baud to see accelerometer and gyro values, resolved screen-space `fluid_accel` values, angular acceleration, fluid speed, measured FPS, frame interval, solver time, and LED transfer time. The in-plane filtered gravity vector continuously defines screen-down, and linear acceleration is projected onto that direction and its perpendicular. No startup pose calibration or fixed sensor-axis assignment is required. `fps` near 100 with low `sim_us` indicates the solver is keeping up; low `fluid_speed_cells_s` points to slow fluid motion instead.

## Hardware configuration

- The matrix DIN is connected to XIAO `D0`.
- Button handling has been removed. A hard shake toggles the display.
- The physical chain advances across six LEDs per row in the official board CAD. `pixelIndex()` uses row-major order with physical rows reversed to match the mounted matrix orientation.
- If the IMU is not detected, check the board revision, I2C wiring, and that the Sense Plus board is selected.

## Controls

Shake the pendant firmly to turn the display off or on. Shake detection arms after 1.2 seconds of rest at startup. The trigger threshold defaults to 2.0 g and is configurable as `HardShakeThresholdG` in `src/main.cpp`.

The FLIP particles start from a discrete streamfunction on the staggered grid, so the initial flow is divergence-free on the solver grid. Serial diagnostics report maximum particle speed in cells per second. When toggled off, the LEDs are blanked and the accelerometer runs at 26 Hz so a hard shake can turn the display back on. There is no inactivity timeout.