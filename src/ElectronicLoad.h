/*
  eMariete Programmable Constant Current Electronic Load
  https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load

  Copyright (c) 2025 eMariete (https://emariete.com)
  Licensed under the MIT License. See LICENSE file for details.
*/

#pragma once

#include <Arduino.h>
#include <Adafruit_INA3221.h>

typedef float (*TemperatureReaderCallback)(int sensorId);

class ElectronicLoad {
public:
  // MCP4725 DAC (12-bit)
  static constexpr uint16_t DAC_MAX = 4095;

  // Constructors
  ElectronicLoad(uint8_t i2cAddress);
  ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms = 0.1f);

  // Initialization
  bool begin();

  // ON/OFF control (affects DAC output)
  void turnOn();
  void turnOn(uint8_t channel);
  void turnOff();
  void update();

  // Manual calibration (legacy single-channel)
  void calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);
  // Per-channel calibration
  void calibrate(uint8_t channel, int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);

  // Automatic calibration using INA3221 readings (per-channel)
  bool autoCalibrate(uint8_t channel, int dacLow, int dacHigh, int samples = 10, unsigned long delayMs = 200);

  // Target current control (per-channel)
  bool setTargetCurrent(uint8_t channel, float targetAmps);
  // legacy convenience: uses default INA channel
  bool setTargetCurrent(float targetAmps) { return setTargetCurrent((uint8_t)_inaChannel, targetAmps); }
  float getTargetCurrent() const { return getTargetCurrent((uint8_t)_inaChannel); }
  float getTargetCurrent(uint8_t channel) const;

  // Overcurrent protection
  void setMaxCurrentLimit(float limitAmps);                // legacy/global (applies to default channel)
  void setMaxCurrentLimit(uint8_t channel, float limitAmps); // per-channel

  // Temperature monitoring
  void setTemperatureReader(TemperatureReaderCallback callback, int sensorId);
  void setTemperatureLimit(float degreesC);
  float getTemperature();
  bool isTemperatureMonitoringEnabled() const;

  // Measurements
  float getVoltage();
  float getCurrent();
  float getPower();

  float getVoltage(uint8_t channel);
  float getCurrent(uint8_t channel);
  float getPower(uint8_t channel);

  // Status
  bool isOn() const;
  float getMaxCurrentLimit() const;          // legacy/global (default INA channel)
  float getMaxCurrentLimit(uint8_t channel) const; // per-channel

  // DAC low-level access (centralized)
  void writeRawDac(uint16_t raw);
  uint16_t getRawDac() const;

  // INA3221 shunt configuration (per channel)
  void setShuntResistance(uint8_t channel, float value);

private:
  struct ChannelData {
    float calibM = 0.0f;   // slope
    float calibB = 0.0f;   // intercept
    float targetAmps = 0.0f;     // last requested target current
    float maxCurrent = 10.0f;     // default per channel
  };

  uint8_t _i2cAddress;
  int _inaChannel; // default INA3221 channel for DAC-driven load (0-based)
  Adafruit_INA3221 _ina3221;

  float _shuntOhms[3];
  ChannelData _channels[3];

  // Last raw DAC value written
  uint16_t _rawDac = 0;
  // Global on/off state
  bool _isOn = false;
  uint8_t _active = 0;

  // Temperature
  TemperatureReaderCallback _tempReader;
  int _tempSensorId;
  float _tempLimitEmergency;
  bool _tempMonitoringEnabled;

  // Low-level DAC write helper (returns success)
  bool writeDacValue(int value);

  // Convert desired current to DAC raw value (applies calibration and clamping)
  int currentToDacValue(uint8_t channel, float amps) const;

  bool isValidChannel(uint8_t channel) const { return channel < 3; }
};