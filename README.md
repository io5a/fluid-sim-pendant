# Fluid Simulation Pendant

Firmware for a Seeed Studio XIAO nRF52840 Sense Plus and the Seeed 6x10 WS2812 RGB Matrix. The LSM6DS3TR-C accelerometer drives a small semi-Lagrangian advection/diffusion simulation; the fixed 60-pixel field avoids dynamic allocation and is updated at a 30 FPS target.

## Build and upload

1. Install PlatformIO in VS Code.
2. Open this folder as the PlatformIO project.
3. Build and upload the `seeed_xiao_nrf52840` environment.

The firmware uses the Arduino framework and the Adafruit NeoPixel library. It reads the IMU directly over I2C, so no separate IMU library is needed.

## Hardware configuration

- The matrix DIN is connected to XIAO `D0`.
- `USER_BUTTON_PIN` defaults to `D1` in `src/main.cpp`. Seeed's public material confirms the Sense Plus has a user button, but the public schematic available for the Plus revision does not document that switch's GPIO. Confirm the pin for your board revision and change this one definition if needed. The button input is configured active-low with an internal pull-up.
- The matrix mapping defaults to serpentine rows. If the image is mirrored or the rows run the other way on your board, adjust `pixelIndex()` or set `MATRIX_SERPENTINE` to `0` in the build flags.
- If the IMU is not detected, check the board revision, I2C wiring, and that the Sense Plus board is selected.

## Controls

- Click: cycle the color palette.
- Double-click: cycle brightness.
- Hold for 800 ms: turn the display simulation off or back on.

The three brightness levels are deliberately conservative for a wearable battery-powered build. Raise `BrightnessLevels` only after checking battery, LED, and regulator current limits.

After 12 seconds without detected movement, the matrix is blanked and the accelerometer is switched to 12.5 Hz. The nRF52840 waits between checks and continues polling the IMU and button so movement or a button hold can wake the animation. This is a low-power idle mode, not a full system-off state.