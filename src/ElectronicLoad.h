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
  // Constructors
  ElectronicLoad(uint8_t i2cAddress);
  ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms = 0.1f);

  // Initialization
  void begin();

  // ON/OFF control (affects DAC output)
  void turnOn();
  void turnOff();
  void update();

  // Manual calibration (legacy single-channel)
  void calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);
  // Per-channel calibration
  void calibrate(uint8_t channel, int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);

  // Automatic calibration using INA3221 readings (for default DAC channel)
  void autoCalibrate(int dac1, int dac2, int samples = 10, unsigned long delayMs = 200);

  // Target current control (single target for DAC-driven load)
  bool setTargetCurrent(float targetAmps);
  float getTargetCurrent() const;

  // Overcurrent protection
  void setMaxCurrentLimit(float limitAmps);                // legacy/global
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
  float getMaxCurrentLimit() const;          // legacy/global
  float getMaxCurrentLimit(uint8_t channel); // per-channel

  // DAC low-level access
  void writeRawDac(int value);

  // INA3221 shunt configuration (per channel)
  void setShuntResistance(uint8_t channel, float value);

private:
  uint8_t _i2cAddress;
  int _inaChannel; // default INA3221 channel for DAC-driven load (0-based)
  Adafruit_INA3221 _ina3221;

  float _shuntOhms[3] = {0.1f, 0.1f, 0.1f};
  float _calib_m[3] = {0.0f, 0.0f, 0.0f};
  float _calib_b[3] = {0.0f, 0.0f, 0.0f};
  float _maxCurrentLimitPerChannel[3] = {10.0f, 10.0f, 10.0f};

  bool _isOn;
  float _targetAmps;
  float _maxCurrentLimit; // legacy/global limit for _inaChannel

  TemperatureReaderCallback _tempReader;
  int _tempSensorId;
  float _tempLimitEmergency;
  bool _tempMonitoringEnabled;

  // DAC helpers
  bool writeDacValue(int value);
  int currentToDacValue(float amps, uint8_t channel = 0) const;
};
