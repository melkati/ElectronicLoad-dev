/*
  eMariete Programmable Constant Current Electronic Load
  https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load

  Copyright (c) 2025 eMariete (https://emariete.com)
  Licensed under the MIT License. See LICENSE file for details.
*/

#include <Arduino.h>
#include "ElectronicLoad.h"
#include <Wire.h>

// INA3221 channel to use (0, 1, or 2)
static constexpr int INA_CHANNEL = 0; // or 1 or 2 depending on wiring

// MCP4725 default address
static constexpr uint8_t MCP4725_ADDR = 0x60;

/**
 * Example temperature callback for demonstration.
 * Replace with a real sensor read (e.g. DS18B20, LM75, etc.)
 */
float exampleTempReader(int sensorId) {
  static float t = 30.0f;
  t += 0.02f;
  if (t > 70.0f) t = 30.0f;
  return t;
}

// Example for a 0.1 Ohm shunt resistor
static constexpr float SHUNT_OHMS = 0.1f;
ElectronicLoad load(MCP4725_ADDR);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ElectronicLoad example starting...");

  // Optional: specify SDA/SCL pins for ESP32
  Wire.begin(21, 22);

  // Initialize load library (starts I2C, zeroes DAC)
  load.begin();
  load.setShuntResistance(0, 0.1f);
  load.setShuntResistance(1, 0.05f);
  load.setShuntResistance(2, 0.2f);

  // Automatic calibration at two DAC points (per-channel API)
  bool ok = load.autoCalibrate(INA_CHANNEL, 160, 500);
  Serial.print("autoCalibrate OK: "); Serial.println(ok ? "true" : "false");

  // Safety limits
  load.setMaxCurrentLimit(1.5f);     // 1.5 A OCP
  load.setTemperatureReader(exampleTempReader, 0);
  load.setTemperatureLimit(70.0f);   // 70 C OTP

  // Set target and enable (per-channel)
  load.setTargetCurrent(INA_CHANNEL, 0.3f); // 0.3 A
  load.turnOn(INA_CHANNEL);
}

void loop() {
  // Poll safety and apply DAC value
  load.update();

  static unsigned long last = 0;
  if (millis() - last > 500) {
    last = millis();
    float v = load.getVoltage();
    float i = load.getCurrent();
    float p = load.getPower();
    float t = load.getTemperature();
    Serial.print("V="); Serial.print(v,3);
    Serial.print(" V, I="); Serial.print(i,3);
    Serial.print(" A, P="); Serial.print(p,3);
    Serial.print(" W, T=");
    if (isnan(t)) Serial.print("N/A"); else Serial.print(t,1);
    Serial.print(" C, Target="); Serial.print(load.getTargetCurrent(INA_CHANNEL),3);
    Serial.print(" A, On="); Serial.println(load.isOn() ? "Yes" : "No");
  }

  // Per-channel verbose output disabled to reduce serial spam
  /*
  for (uint8_t ch = 0; ch < 3; ++ch) {
      float v = load.getVoltage(ch);
      float i = load.getCurrent(ch);
      Serial.printf("Channel %d: V=%.3fV, I=%.3fA\n", ch, v, i);
  }
  */
}