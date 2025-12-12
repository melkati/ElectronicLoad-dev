/*
  eMariete Programmable Constant Current Electronic Load
  https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load

  Copyright (c) 2025 eMariete (https://emariete.com)
  Licensed under the MIT License. See LICENSE file for details.
*/

#include "ElectronicLoad.h"
#include <Wire.h>
#include <Adafruit_INA3221.h>

// DAC constants
static constexpr int DAC_MAX = 4095;
static constexpr int DAC_MIN = 0;

// --- Constructors ---
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
  for (int i = 0; i < 3; ++i) {
    _shuntOhms[i] = 0.1f;
    _calib_m[i] = 0.0f;
    _calib_b[i] = 0.0f;
    _maxCurrentLimitPerChannel[i] = 10.0f;
  }
}

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

// --- Init ---
void ElectronicLoad::begin() {
  Wire.begin();
  _ina3221.begin();
  for (uint8_t ch = 0; ch < 3; ++ch) {
    _ina3221.setShuntResistance(ch, _shuntOhms[ch]);
  }
  writeDacValue(0);
}

// --- Control ---
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

  // OCP for default channel
  float current = getCurrent((uint8_t)_inaChannel);
  if (!isnan(current) && current >= _maxCurrentLimit) {
    turnOff();
    Serial.println("[CRITICAL] OCP triggered: current exceeded limit. Load turned OFF.");
    return;
  }

  // OTP
  if (_tempMonitoringEnabled && _tempReader) {
    float tempC = _tempReader(_tempSensorId);
    if (!isnan(tempC) && tempC >= _tempLimitEmergency) {
      turnOff();
      Serial.println("[CRITICAL] OTP triggered: temperature exceeded limit. Load turned OFF.");
      return;
    }
  }

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

float ElectronicLoad::getTargetCurrent() const { return _targetAmps; }

// --- Calibration ---
void ElectronicLoad::calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax) {
  calibrate((uint8_t)_inaChannel, dacMin, currentMeasMin, dacMax, currentMeasMax);
}

void ElectronicLoad::calibrate(uint8_t channel, int dacMin, float currentMeasMin, int dacMax, float currentMeasMax) {
  if (channel >= 3 || dacMin == dacMax) return;
  float m = (currentMeasMax - currentMeasMin) / float(dacMax - dacMin);
  float b = currentMeasMin - m * float(dacMin);
  _calib_m[channel] = m;
  _calib_b[channel] = b;
}

void ElectronicLoad::autoCalibrate(int dac1, int dac2, int samples, unsigned long delayMs) {
  int dacValues[2] = {dac1, dac2};
  float currents[2] = {0.0f, 0.0f};

  turnOn();
  for (int idx = 0; idx < 2; ++idx) {
    int dac = dacValues[idx];
    writeRawDac(dac);
    delay(500);

    float sumI = 0.0f;
    for (int i = 0; i < samples; ++i) {
      delay(delayMs);
      sumI += getCurrent((uint8_t)_inaChannel);
    }
    currents[idx] = sumI / float(samples);
    Serial.print("DAC="); Serial.print(dac);
    Serial.print(" MeasuredCurrent="); Serial.print(currents[idx], 6);
    Serial.println(" A");
  }

  calibrate((uint8_t)_inaChannel, dac1, currents[0], dac2, currents[1]);
  Serial.println("Calibration complete.");

  turnOff();
}

// --- Limits ---
void ElectronicLoad::setMaxCurrentLimit(float limitAmps) {
  _maxCurrentLimit = limitAmps;
}

void ElectronicLoad::setMaxCurrentLimit(uint8_t channel, float limitAmps) {
  if (channel < 3) _maxCurrentLimitPerChannel[channel] = limitAmps;
}

float ElectronicLoad::getMaxCurrentLimit() const { return _maxCurrentLimit; }

float ElectronicLoad::getMaxCurrentLimit(uint8_t channel) {
  if (channel < 3) return _maxCurrentLimitPerChannel[channel];
  return NAN;
}

// --- Temperature ---
void ElectronicLoad::setTemperatureReader(TemperatureReaderCallback callback, int sensorId) {
  _tempReader = callback;
  _tempSensorId = sensorId;
  _tempMonitoringEnabled = (callback != nullptr);
}

void ElectronicLoad::setTemperatureLimit(float degreesC) {
  _tempLimitEmergency = degreesC;
}

float ElectronicLoad::getTemperature() {
  if (_tempReader) return _tempReader(_tempSensorId);
  return NAN;
}

bool ElectronicLoad::isTemperatureMonitoringEnabled() const { return _tempMonitoringEnabled; }

// --- Measurements ---
float ElectronicLoad::getVoltage(uint8_t channel) {
  if (channel < 3) return _ina3221.getBusVoltage(channel);
  return NAN;
}

float ElectronicLoad::getCurrent(uint8_t channel) {
  if (channel < 3) return _ina3221.getCurrentAmps(channel);
  return NAN;
}

float ElectronicLoad::getPower(uint8_t channel) {
  float v = getVoltage(channel);
  float i = getCurrent(channel);
  if (isnan(v) || isnan(i)) return NAN;
  return v * i;
}

float ElectronicLoad::getVoltage() { return getVoltage((uint8_t)_inaChannel); }
float ElectronicLoad::getCurrent() { return getCurrent((uint8_t)_inaChannel); }
float ElectronicLoad::getPower()   { return getPower((uint8_t)_inaChannel); }

// --- Status ---
bool ElectronicLoad::isOn() const { return _isOn; }

// --- DAC helpers ---
bool ElectronicLoad::writeDacValue(int value) {
  if (value < DAC_MIN) value = DAC_MIN;
  if (value > DAC_MAX) value = DAC_MAX;

  Wire.beginTransmission(_i2cAddress);
  Wire.write(0x40);
  uint8_t high = (value >> 4) & 0xFF;
  uint8_t low  = (value & 0x0F) << 4;
  Wire.write(high);
  Wire.write(low);
  uint8_t result = Wire.endTransmission();
  return (result == 0);
}

int ElectronicLoad::currentToDacValue(float amps, uint8_t channel) const {
  if (channel >= 3) channel = 0;
  float m = _calib_m[channel];
  float b = _calib_b[channel];
  if (m == 0.0f) return 0;
  float dacFloat = (amps - b) / m;
  if (dacFloat < float(DAC_MIN)) dacFloat = float(DAC_MIN);
  if (dacFloat > float(DAC_MAX)) dacFloat = float(DAC_MAX);
  return int(dacFloat + 0.5f);
}

void ElectronicLoad::writeRawDac(int value) { (void)writeDacValue(value); }

// --- Shunt ---
void ElectronicLoad::setShuntResistance(uint8_t channel, float value) {
  if (channel < 3) {
    _shuntOhms[channel] = value;
    _ina3221.setShuntResistance(channel, value);
  }
}
