# eMariete Programmable Constant Current Electronic Load

[![MIT License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A PlatformIO-friendly Arduino library that turns an ESP32, an INA3221 triple-channel current monitor, and an MCP4725 DAC into a multi-channel programmable electronic load with per-channel calibration, temperature supervision, and INA3221-driven telemetry.

## Features
- **Triple-channel control**  independently enable/disable channels, set their shunt resistors, and enforce per-channel current limits.
- **Closed-loop regulation**  generates DAC setpoints through an MCP4725 and continuously checks INA3221 telemetry to keep the requested current in range.
- **Guided calibration**  run `autoCalibrate()` to sweep two DAC points, average INA3221 readings, and derive slope/offset values.
- **Safety hooks**  optional temperature callback, hard over-current cutoffs, and automatic shutdown with console diagnostics.
- **Turnkey example**  [examples/BasicUsage/src/main.cpp](examples/BasicUsage/src/main.cpp) demonstrates initialization, calibration, and live telemetry streaming over Serial.

## Hardware Assumptions
| Part | Notes |
| --- | --- |
| ESP32 DevKit (or compatible) | Example uses pins 21/22 for I2C; adjust `Wire.begin()` for your board. |
| INA3221 breakout | Default address `0x40`; per-channel shunt value is required. |
| MCP4725 DAC | Default address `0x60`; drives the MOSFET gate driver / op-amp stage. |
| Power MOSFET + shunts | One shunt per channel; connect to INA3221 inputs and the analog stage that sinks current. |

## Getting Started
1. Wire ESP32, INA3221 and MCP4725 on the same I2C bus plus your load hardware.
2. Install dependencies for the example once:

```
pio pkg install --library "adafruit/Adafruit INA3221@^1.0.0" --library "adafruit/Adafruit BusIO@^1.14.5" -d examples/BasicUsage
```

3. Build the reference sketch:

```
pio run -d examples/BasicUsage
```

4. Flash your board and open the monitor at 115200:

```
pio run -t upload -d examples/BasicUsage
pio device monitor -p COM24 -b 115200
```

5. Tune shunt values, maximum current, and calibration ranges inside the example or via your own sketch.

The example project under [examples/BasicUsage](examples/BasicUsage) has its own `platformio.ini` so it can be built independently while still consuming the local library source in [src](src).

## Library Overview
Key entry points exposed by [src/ElectronicLoad.h](src/ElectronicLoad.h):
- `enableChannel()` / `disableChannel()` / `setActiveChannel()` to manage which sink is currently active.
- `setTargetCurrent()` and `setMaxCurrentLimit()` per channel plus `turnOn()` / `turnOff()` for runtime control.
- `setShuntResistance()` to keep INA3221 math correct, matching the real shunt resistors in your design.
- `autoCalibrate()` or manual `calibrate()` when you have DAC/current reference points.
- `getVoltage()`, `getCurrent()`, and `getPower()` for telemetry streaming.
- `setTemperatureReader()` and `setTemperatureLimit()` to add a safety thermostat.

See [src/ElectronicLoad.cpp](src/ElectronicLoad.cpp) for the regulation loop, DAC writes, and protective checks.

## Calibration Workflow
1. Start with conservative current targets (for example 0.2 A).
2. Call `autoCalibrate(channel, dacLow, dacHigh, samples, delayMs)` with two DAC codes spanning the expected operating range.
3. Verify the averaged INA3221 currents printed on Serial; repeat with refined ranges if necessary.
4. Persist the derived slope/offset (e.g., store in NVS, LittleFS, or hardcode) for deterministic startups.

## Extending the Project
- Replace the `mockTemperatureReader()` in the example with your preferred sensor (NTC on ADC, digital temperature sensor, etc.).
- Integrate PID-style regulation by observing `getCurrent()` and adjusting `setTargetCurrent()` dynamically.
- Add more automation by querying the INA3221 warning/critical registers (already initialized by Adafruit's driver) for smarter protections.

## Wiring (quick)
- MCP4725 SDA  ESP32 SDA (e.g., GPIO21)
- MCP4725 SCL  ESP32 SCL (e.g., GPIO22)
- INA3221 SDA/SCL  ESP32 I2C bus
- Shunt resistor connected to INA3221 channel 0, 1, or 2

## Example (compact)
See [examples/BasicUsage/main.cpp](examples/BasicUsage/main.cpp):

```cpp
#include <Arduino.h>
#include "ElectronicLoad.h"
#include <Wire.h>

static constexpr int INA_CHANNEL = 0; // 0, 1, or 2
static constexpr uint8_t MCP4725_ADDR = 0x60;
static constexpr float SHUNT_OHMS = 0.1f;
ElectronicLoad load(MCP4725_ADDR);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  load.begin();
  load.autoCalibrate(160, 500);
  load.setMaxCurrentLimit(1.5f);
  load.setTargetCurrent(0.3f);
  load.turnOn();
}

void loop() {
  load.update();
  // ... print telemetry ...
}
```

## License
This project is released under the MIT license. See [LICENSE](LICENSE) for details.

## Author
[eMariete](https://emariete.com)
GitHub: [melkati](https://github.com/melkati)
