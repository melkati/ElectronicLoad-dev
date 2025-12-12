#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "ElectronicLoad.h"

/*
  Basic usage example for the ElectronicLoad class.

  This sketch demonstrates:
  - Initializing I2C and the ElectronicLoad instance
  - Setting per-channel shunt resistances
  - Calibrating the active channel (using INA3221 measurements)
  - Setting target/current limits
  - Periodically calling update() and printing measurements

  The comments are intentionally detailed to enable automatic documentation
  generation and to serve as contextual help for code-completion tools.
*/

// MCP4725 default I2C address (7-bit)
static constexpr uint8_t MCP4725_ADDR = 0x60;

// Example shunt resistor values (ohms) for INA3221 channels.
// Adjust these to match your hardware. Each channel maps to a shunt.
static constexpr float SHUNT_OHMS[ElectronicLoad::ChannelCount] = {0.1f, 0.01f, 0.1f};

// Optional temperature sensor callback placeholder. Return NAN if not available.
// The callback signature: float reader(int sensorId)
float mockTemperatureReader(int /*sensorId*/) {
  return NAN; // Replace with actual sensor read if available
}

// Create a single ElectronicLoad instance bound to the DAC I2C address.
ElectronicLoad load(MCP4725_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("eMariete Programmable Electronic Load - Basic Usage");

  // Initialize Wire with explicit SDA/SCL pins (ESP32 example pins used here).
  // Change pins as required by your platform.
  Wire.begin(21, 22);
  load.begin();

  // Configure shunt resistances and enable all channels.
  for (uint8_t ch = 0; ch < ElectronicLoad::ChannelCount; ++ch) {
    load.setShuntResistance(ch, SHUNT_OHMS[ch]);
    load.enableChannel(ch);
  }

  // Provide a temperature reader (optional) and set an emergency temperature limit (deg C).
  load.setTemperatureReader(mockTemperatureReader, 0);
  load.setTemperatureLimit(70.0f);

  // Calibrate the active channel (channel 0 by default) using the INA3221.
  // Arguments: channel, dacLow, dacHigh, samples (optional), delayMs (optional)
  // The example below requests DAC=160 and DAC=400 sample points to compute slope.
  // Guarded auto-calibration: only run when the input (channel 0) is present.
  // If channel 0 bus voltage is NAN or below 1.0V, skip calibration to avoid
  // performing calibration with no input (which would calibrate in open-circuit).
  {
    float ch0_v = load.getVoltage(0);
    if (!isnan(ch0_v) && ch0_v >= 1.0f) {
      load.autoCalibrate(0, 160, 400);
    } else {
      Serial.print(F("[ElectronicLoad] Skipping autoCalibrate: channel 0 voltage="));
      if (isnan(ch0_v)) Serial.println(F("NAN")); else Serial.println(ch0_v, 3);
    }
  }

  // Set per-channel maximum current limit (safety) and the requested target current.
  load.setMaxCurrentLimit(0, 0.5f);
  load.setTargetCurrent(0, 0.3f);

  // Turn the load on for the active channel.
  load.turnOn(0);
}

void loop() {
  // Keep the ElectronicLoad logic running: safety checks + DAC updates.
  load.update();

  // Print measurements for each channel to the serial console.
  for (uint8_t ch = 0; ch < ElectronicLoad::ChannelCount; ++ch) {
    float voltage = load.getVoltage(ch);
    float current = load.getCurrent(ch);
    if (!isnan(voltage) && !isnan(current)) {
      Serial.print("CH");
      Serial.print(ch);
      Serial.print(" V=");
      Serial.print(voltage, 3);
      Serial.print("V I=");
      Serial.print(current, 3);
      Serial.print("A P=");
      Serial.print(load.getPower(ch), 3);
      Serial.print("W");
      if (ch == load.getActiveChannel()) {
        Serial.print(" *"); // mark the active channel with an asterisk
      }
      Serial.print("  ");
    }
  }
  Serial.println();

  delay(500); // main loop interval; tune as necessary for your application
}
