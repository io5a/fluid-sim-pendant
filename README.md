# Fluid Simulation Pendant

Firmware for a Seeed Studio XIAO nRF52840 Sense Plus and the Seeed 6x10 WS2812 RGB Matrix. The LSM6DS3TR-C drives a FLIP simulation over the full 6x10 grid at an 83 FPS target. It starts with a 3x6 fluid pool at the bottom, rendered in one green color.

The 72-particle solver uses a staggered velocity grid, a free-surface pressure projection, valid-face PIC/FLIP interpolation, and four neighboring-cell separation passes. It starts without injected velocity or idle swirl. Walls remove only velocity directed into the surface; tangential motion and velocity away from the wall are preserved. The reference blend is 90% FLIP / 10% PIC at 12 ms and is adjusted for substep duration. Bilinear density splats give smooth, mass-preserving green coverage instead of abrupt cell-occupancy jumps.

## Build and upload

1. Install PlatformIO in VS Code.
2. Open this folder as the PlatformIO project.
3. Build and upload the `seeed_xiao_nrf52840` environment.

From the project root, run `pwsh -File tools/run-sim-tests.ps1 -Port COM8` for deterministic checks on the actual MCU using the production solver. The script temporarily uploads a test build, captures PASS/FAIL output, and restores the regular firmware in `finally`, including after a failed assertion. Close other serial monitors first. USB serial re-enumeration and DTR are handled by the runner.

Run `pwsh -File tools/run-sim-tests.ps1 -Port COM8 -CheckMapping` for the interactive logical-bottom and logical-left LED-edge check. Answer which physical edge lights up. This diagnoses mounting; it is not a required startup calibration.

The suite covers gravity and translation axes, rapid inversion, isolated-droplet momentum, drainage along vertical and tilted walls, settling, collision slip, density continuity and mass, gyro noise and rotating-frame forces, Coriolis energy conservation, fast-spin particle continuity, zero/negative timesteps, delayed frames, bounded stall recovery, and solver-plus-LED timing. It cannot replace a physical visual check of the assembled pendant.

The firmware uses the Arduino framework, Adafruit NeoPixel, and Seeed's LSM6DS3 library. The Seeed driver handles the Sense Plus IMU's internal I2C bus and sensor power pin.

Open the PlatformIO Serial Monitor at 115200 baud to see sensor values, `fluid_accel` in cells/s^2, particle speed in cells/s, measured FPS, solver and LED time, physics substeps, and maximum advective travel per substep. The 12 ms target is about 83 FPS; live operation with diagnostics has been around 80 FPS.

## Motion and units

The mounting was checked with the LED face toward the observer and the XIAO component side facing away, USB downward. Upright samples were approximately `(0.978, -0.045, -0.098) g`; leaning the top left produced positive sensor Y, and leaning right produced negative sensor Y. The logical force transform is therefore `screen_x = -sensor_y`, `screen_y = sensor_x`. A separate physical edge test confirmed bottom and left after reversing both electrical row and column order. This transform is mounting-specific, not inferred from a single arbitrary startup pose.

Accelerometer readings are specific force. Filtered gravity plus the linear remainder reconstruct the latest raw sample exactly, so tilt response is not delayed by the gravity filter or counted twice. The virtual tank height is 0.35 m: one g converts to about 280 cells/s^2. This virtual size sets visual pace, not the actual board dimensions, and replaces the previous arbitrary speed/force multipliers.

The official Eagle CAD has 2.032 mm LED pitch. Gyro lever arms use that physical pitch before conversion to simulation units, avoiding oversized fast-spin forces. Euler, centrifugal, and Coriolis terms are included; an implicit-midpoint Coriolis update avoids adding kinetic energy. A small gyro deadband suppresses stationary noise. Invalid reads clear transient motion rather than retaining an old impulse. The accelerometer uses a +/-4 g range and the gyro a +/-500 degrees/s range.

Particle speed is limited to 40 cells/s. Ordinary frames use one physics step; delayed frames are divided so advection stays below 0.75 cell per substep. Catch-up is limited to four substeps (75 ms of simulation); excess time after a longer stall is discarded to avoid tunneling and unbounded catch-up work. This is a planar liquid approximation, not a full 3D free-surface model.

## Hardware configuration

- The matrix DIN is connected to XIAO `D0`.
- Button handling has been removed. A hard shake toggles the display.
- The physical chain advances across six LEDs per row in the official board CAD. `pixelIndex()` reverses both physical rows and columns for the manually verified mounting.
- If the IMU is not detected, check the board revision, I2C wiring, and that the Sense Plus board is selected.

## Controls

Shake the pendant firmly to turn the display off or on. Shake detection arms after 1.2 seconds of rest at startup. The trigger threshold defaults to 2.0 g and is configurable as `HardShakeThresholdG` in `src/main.cpp`.

There is no startup pose calibration, artificial startup vortex, or perpetual stirring. The initial pool settles under the current IMU force. When toggled off, the LEDs are blanked and both IMU output rates drop to 26 Hz so a hard shake can turn the display back on. There is no inactivity timeout.

## References

- [Ten Minute Physics FLIP tutorial and source](https://matthias-research.github.io/pages/tenMinutePhysics/18-flip.html)
- [Mitxela fluid pendant](https://mitxela.com/projects/fluid-pendant)
- [Seeed matrix hardware and Eagle CAD](https://wiki.seeedstudio.com/rgb_matrix_for_xiao/)