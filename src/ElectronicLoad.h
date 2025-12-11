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
  // Single-argument constructor (uses multi-channel INA3221 instance)
  ElectronicLoad(uint8_t i2cAddress);
  // Backwards-compatible constructor: specify a default INA channel and shunt
  ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms = 0.1f);

  // Initialization
  void begin();

  // ON/OFF control (affects DAC output)
  void turnOn();
  void turnOff();
  void update(); // poll safety and apply DAC

  // Manual calibration (backwards-compatible single-channel)
  void calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);
  // Per-channel calibration
  void calibrate(uint8_t channel, int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);

  // Automatic calibration using INA3221 readings (applies to the channel currently used by DAC)
  void autoCalibrate(int dac1, int dac2, int samples = 10, unsigned long delayMs = 200);

  // Target/current control (single target for DAC-driven load)
  bool setTargetCurrent(float targetAmps);
  float getTargetCurrent() const;

  // Per-channel overcurrent protection: overloaded API
  void setMaxCurrentLimit(float limitAmps);                // global / legacy
  void setMaxCurrentLimit(uint8_t channel, float limitAmps); // per-channel

  // Temperature monitoring
  void setTemperatureReader(TemperatureReaderCallback callback, int sensorId);
  void setTemperatureLimit(float degreesC);
  float getTemperature();
  bool isTemperatureMonitoringEnabled() const;

  // Measurements
  // Legacy single-channel getters (useful if using single-channel flow)
  float getVoltage();
  float getCurrent();
  float getPower();

  // Per-channel measurement API
  float getVoltage(uint8_t channel);
  float getCurrent(uint8_t channel);
  float getPower(uint8_t channel);

  // Status
  bool isOn() const;
  float getMaxCurrentLimit() const;           // legacy
  float getMaxCurrentLimit(uint8_t channel);  // per-channel

  // DAC low-level access (for calibration routines)
  void writeRawDac(int value);

  // INA3221 shunt configuraton (per channel)
  void setShuntResistance(uint8_t channel, float value);

private:
  uint8_t _i2cAddress;
  int _inaChannel; // legacy: default channel for single-channel API
  Adafruit_INA3221 _ina3221;

  // Per-channel calibration and shunt storage
  float _shuntOhms[3] = {0.1f, 0.1f, 0.1f};
  float _calib_m[3] = {0.0f, 0.0f, 0.0f};
  float _calib_b[3] = {0.0f, 0.0f, 0.0f};
  float _maxCurrentLimitPerChannel[3] = {10.0f, 10.0f, 10.0f};

  // Legacy / single-instance fields
  bool _isOn;
  float _targetAmps;
  float _maxCurrentLimit; // legacy single-channel limit

  // Temperature monitoring
  TemperatureReaderCallback _tempReader;
  int _tempSensorId;
  float _tempLimitEmergency;
  bool _tempMonitoringEnabled;

  // DAC helper
  bool writeDacValue(int value);
  int currentToDacValue(float amps, uint8_t channel = 0) const;
};

// --- Constructors ---

// Single-argument constructor (multi-channel mode, default INA channel = 0)
ElectronicLoad::ElectronicLoad(uint8_t i2cAddress)
  : _i2cAddress(i2cAddress),
    _inaChannel(0),
    _ina3221(),
    _isOn(false),
    _targetAmps(0.0f),
    _maxCurrentLimit(10.0f),
    _tempReader(nullptr),
    _tempSensorId(0),
    _tempLimitEmergency(85.0f),
    _tempMonitoringEnabled(false)
{
  // initialize per-channel arrays with defaults
  for (int i = 0; i < 3; ++i) {
    _shuntOhms[i] = 0.1f;
    _calib_m[i] = 0.0f;
    _calib_b[i] = 0.0f;
    _maxCurrentLimitPerChannel[i] = 10.0f;
  }
}

// Backwards-compatible constructor: specify default INA channel and a shunt value for that channel
ElectronicLoad::ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms)
  : _i2cAddress(i2cAddress),
    _inaChannel(inaChannel >= 0 && inaChannel < 3 ? inaChannel : 0),
    _ina3221(),
    _isOn(false),
    _targetAmps(0.0f),
    _maxCurrentLimit(10.0f),
    _tempReader(nullptr),
    _tempSensorId(0),
    _tempLimitEmergency(85.0f),
    _tempMonitoringEnabled(false)
{
  for (int i = 0; i < 3; ++i) {
    _shuntOhms[i] = 0.1f;
    _calib_m[i] = 0.0f;
    _calib_b[i] = 0.0f;
    _maxCurrentLimitPerChannel[i] = 10.0f;
  }
  _shuntOhms[_inaChannel] = shuntOhms;
}

 // --- ON/OFF helpers (use legacy/default channel for DAC mapping) ---

void ElectronicLoad::turnOn() {
  _isOn = true;
  int dacValue = currentToDacValue(_targetAmps, (uint8_t)_inaChannel);
  writeDacValue(dacValue);
}

void ElectronicLoad::turnOff() {
  _isOn = false;
  writeDacValue(0);
}

void ElectronicLoad::update() {
  if (!_isOn) return;

  // Over Current Protection (OCP) for legacy channel
  float current = getCurrent((uint8_t)_inaChannel);
  if (!isnan(current) && current >= _maxCurrentLimit) {
    turnOff();
    Serial.println("[CRITICAL] OCP triggered: current exceeded limit. Load turned OFF.");
    return;
  }

  // Over Temperature Protection (OTP)
  if (_tempMonitoringEnabled && _tempReader) {
    float tempC = _tempReader(_tempSensorId);
    if (!isnan(tempC) && tempC >= _tempLimitEmergency) {
      turnOff();
      Serial.println("[CRITICAL] OTP triggered: temperature exceeded limit. Load turned OFF.");
      return;
    }
  }

  // Apply target current to DAC (use legacy/default channel)
  int dacValue = currentToDacValue(_targetAmps, (uint8_t)_inaChannel);
  writeDacValue(dacValue);
}

bool ElectronicLoad::setTargetCurrent(float targetAmps) {
  if (targetAmps < 0.0f) return false;
  _targetAmps = targetAmps;
  if (_isOn) {
    int dacValue = currentToDacValue(_targetAmps, (uint8_t)_inaChannel);
    writeDacValue(dacValue);
  }
  return true;
}

// --- Per-channel helpers and calibration already exist in header ---

// Convert desired current to DAC value using per-channel calibration
int ElectronicLoad::currentToDacValue(float amps, uint8_t channel) const {
  if (channel >= 3) channel = 0;
  float m = _calib_m[channel];
  float b = _calib_b[channel];
  if (m == 0.0f) {
    return 0;
  }
  float dacFloat = (amps - b) / m;
  if (dacFloat < float(DAC_MIN)) dacFloat = float(DAC_MIN);
  if (dacFloat > float(DAC_MAX)) dacFloat = float(DAC_MAX);
  return int(dacFloat + 0.5f);
}