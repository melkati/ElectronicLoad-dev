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
ElectronicLoad load(MCP4725_ADDR, INA_CHANNEL, SHUNT_OHMS);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ElectronicLoad example starting...");

  // Optional: specify SDA/SCL pins for ESP32
  Wire.begin(21, 22);

  // Initialize load library (starts I2C, zeroes DAC)
  load.begin();

  // Automatic calibration at two DAC points with 160 and 500mA targets
  load.autoCalibrate(160, 500);

  // Safety limits
  load.setMaxCurrentLimit(1.5f);     // 1.5 A OCP
  load.setTemperatureReader(exampleTempReader, 0);
  load.setTemperatureLimit(70.0f);   // 70 C OTP

  // Set target and enable
  load.setTargetCurrent(0.3f); // 0.3 A
  load.turnOn();
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
    Serial.print(" C, Target="); Serial.print(load.getTargetCurrent(),3);
    Serial.print(" A, On="); Serial.println(load.isOn() ? "Yes" : "No");
  }
}

// Example: apply a target and print averaged ADC-based current reading
void calibrateStep(ElectronicLoad &load, int dacValue, int samples = 10, unsigned long delayMs = 200) {
  delay(500); // Wait for system to settle
  float sumI = 0.0f;
  for (int i = 0; i < samples; ++i) {
    delay(delayMs);
    sumI += load.getCurrent();
  }
  float avgI = sumI / float(samples);
  Serial.print("DAC=");
  Serial.print(dacValue);
  Serial.print(" AvgCurrent=");
  Serial.print(avgI, 6);
  Serial.println(" A");
}