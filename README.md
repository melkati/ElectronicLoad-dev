# eMariete Programmable Constant Current Electronic Load

[![MIT License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Library for a programmable electronic load using ESP32, MCP4725 DAC, and INA3221 current/voltage monitor.

---

## Features

- Constant Current (CC) mode
- Safety: Overcurrent and overtemperature protection
- Automatic and manual calibration (with INA3221)
- Flexible shunt resistor configuration
- Example project included

---

## Hardware

- ESP32 (Arduino framework)
- MCP4725 (I2C DAC)
- INA3221 (I2C current/voltage monitor)
- Shunt resistor (configurable, e.g., 0.1Ω)

---

## Wiring

- MCP4725 SDA → ESP32 SDA (e.g., GPIO21)
- MCP4725 SCL → ESP32 SCL (e.g., GPIO22)
- INA3221 SDA/SCL → ESP32 I2C bus
- Shunt resistor connected to INA3221 channel 0, 1, or 2

---

## Quick Start

1. **Install via PlatformIO:**
    ```
    lib_deps =
      https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load.git
    ```
2. **See the example:**  
   Open `examples/BasicUsage/main.cpp` for a complete usage example.
3. **Adjust for your hardware:**  
   Set `INA_CHANNEL` and `SHUNT_OHMS` to match your wiring and shunt value.
4. **Calibrate:**  
   Use `autoCalibrate()` for easy calibration using the INA3221.

---

## Example

See [examples/BasicUsage/main.cpp](examples/BasicUsage/main.cpp):

```cpp
#include <Arduino.h>
#include "ElectronicLoad.h"
#include <Wire.h>

static constexpr int INA_CHANNEL = 0; // 0, 1, or 2
static constexpr uint8_t MCP4725_ADDR = 0x60;
static constexpr float SHUNT_OHMS = 0.1f;
ElectronicLoad load(MCP4725_ADDR, INA_CHANNEL, SHUNT_OHMS);

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

---

## Calibration

The library supports automatic calibration using the INA3221.  
You can also calibrate manually by measuring current with a multimeter and calling `load.calibrate(dac1, i1, dac2, i2);`.

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Author

[eMariete](https://emariete.com)  
GitHub: [melkati](https://github.com/melkati)