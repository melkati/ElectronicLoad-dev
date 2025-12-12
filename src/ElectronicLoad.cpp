/*
  eMariete Programmable Constant Current Electronic Load
  https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load

  Copyright (c) 2025 eMariete (https://emariete.com)
  Licensed under the MIT License. See LICENSE file for details.
*/

#include "ElectronicLoad.h"
#include <Wire.h>
#include <Adafruit_INA3221.h>

// Note: DAC_MAX defined in header

// --- Constructors ---
ElectronicLoad::ElectronicLoad(uint8_t i2cAddress)
  : _i2cAddress(i2cAddress),
    _inaChannel(0),
    _ina3221(),
    _rawDac(0),
    _isOn(false),
    _active(0),
    _tempReader(nullptr),
    _tempSensorId(0),
    _tempLimitEmergency(85.0f),
    _tempMonitoringEnabled(false)
{
  for (int i = 0; i < 3; ++i) {
    _shuntOhms[i] = 0.1f;
    _channels[i].calibM = 0.0f;
    _channels[i].calibB = 0.0f;
    _channels[i].targetAmps = 0.0f;
    _channels[i].maxCurrent = 10.0f;
  }
}

ElectronicLoad::ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms)
  : _i2cAddress(i2cAddress),
    _inaChannel(inaChannel >= 0 && inaChannel < 3 ? inaChannel : 0),
    _ina3221(),
    _rawDac(0),
    _isOn(false),
    _active(0),
    _tempReader(nullptr),
    _tempSensorId(0),
    _tempLimitEmergency(85.0f),
    _tempMonitoringEnabled(false)
{
  for (int i = 0; i < 3; ++i) {
    _shuntOhms[i] = 0.1f;
    _channels[i].calibM = 0.0f;
    _channels[i].calibB = 0.0f;
    _channels[i].targetAmps = 0.0f;
    _channels[i].maxCurrent = 10.0f;
  }
  _shuntOhms[_inaChannel] = shuntOhms;
}

// --- Init ---
bool ElectronicLoad::begin() {
  Wire.begin();
  _ina3221.begin();
  for (uint8_t ch = 0; ch < 3; ++ch) {
    _ina3221.setShuntResistance(ch, _shuntOhms[ch]);
  }
  _rawDac = 0;
  _isOn = false;
  writeRawDac(0);
  return true;
}

// --- Control ---
void ElectronicLoad::turnOn() {
  turnOn((uint8_t)_inaChannel);
}

void ElectronicLoad::turnOn(uint8_t channel) {
  if (!isValidChannel(channel)) return;
  _active = channel;
  _isOn = true;
  int dacValue = currentToDacValue(channel, _channels[channel].targetAmps);
  writeRawDac(static_cast<uint16_t>(dacValue));
}

void ElectronicLoad::turnOff() {
  // Deterministically zero the DAC via centralized path
  writeRawDac(0);
  _isOn = false;
  if (isValidChannel(_active)) _channels[_active].targetAmps = 0.0f;
}

void ElectronicLoad::update() {
  if (!_isOn) return;

  // OCP for active channel
  float current = getCurrent((uint8_t)_active);
  if (!isnan(current) && current >= _channels[_active].maxCurrent) {
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

  int dacValue = currentToDacValue(_active, _channels[_active].targetAmps);
  writeRawDac(static_cast<uint16_t>(dacValue));
}

bool ElectronicLoad::setTargetCurrent(uint8_t channel, float targetAmps) {
  if (!isValidChannel(channel)) return false;
  if (!isfinite(targetAmps)) targetAmps = 0.0f;
  // Clamp within [0, maxCurrent]
  if (targetAmps < 0.0f) targetAmps = 0.0f;
  if (targetAmps > _channels[channel].maxCurrent) targetAmps = _channels[channel].maxCurrent;

  _channels[channel].targetAmps = targetAmps;
  int raw = currentToDacValue(channel, targetAmps);
  writeRawDac(static_cast<uint16_t>(raw));
  _isOn = (targetAmps > 0.0f);
  return true;
}

float ElectronicLoad::getTargetCurrent() const { return getTargetCurrent((uint8_t)_inaChannel); }
float ElectronicLoad::getTargetCurrent(uint8_t channel) const {
  if (!isValidChannel(channel)) return 0.0f;
  return _channels[channel].targetAmps;
}

// --- Calibration ---
void ElectronicLoad::calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax) {
  calibrate((uint8_t)_inaChannel, dacMin, currentMeasMin, dacMax, currentMeasMax);
}

void ElectronicLoad::calibrate(uint8_t channel, int dacMin, float currentMeasMin, int dacMax, float currentMeasMax) {
  if (!isValidChannel(channel) || dacMin == dacMax) return;
  float m = (currentMeasMax - currentMeasMin) / float(dacMax - dacMin);
  float b = currentMeasMin - m * float(dacMin);
  _channels[channel].calibM = m;
  _channels[channel].calibB = b;
}

bool ElectronicLoad::autoCalibrate(uint8_t channel, int dacLow, int dacHigh, int samples, unsigned long delayMs) {
  if (!isValidChannel(channel)) {
    Serial.printf("[ElectronicLoad] autoCalibrate: invalid channel %u\n", channel);
    return false;
  }
  if (samples <= 0) samples = 1;
  if (delayMs == 0) delayMs = 1;
  if (dacLow < 0) dacLow = 0;
  if (dacHigh < 0) dacHigh = 0;
  if (dacLow > DAC_MAX) dacLow = DAC_MAX;
  if (dacHigh > DAC_MAX) dacHigh = DAC_MAX;
  if (dacLow == dacHigh) {
    Serial.printf("[ElectronicLoad] autoCalibrate: dacLow == dacHigh (%d)\n", dacLow);
    return false;
  }

  Serial.printf("[ElectronicLoad] autoCalibrate: ch=%u low=%d high=%d samples=%d delay=%lu\n",
                channel, dacLow, dacHigh, samples, delayMs);

  const bool priorOn = _isOn;
  const uint16_t priorRaw = _rawDac;
  const float priorTarget = _channels[channel].targetAmps;

  // Ensure channel active
  turnOn(channel);

  auto sampleAvg = [&](uint16_t rawVal)->float {
    writeRawDac(rawVal);
    float acc = 0.0f;
    for (int i = 0; i < samples; ++i) {
      delay(delayMs);
      float cur = getCurrent(channel);
      if (!isfinite(cur)) {
        Serial.printf("[ElectronicLoad] autoCalibrate: NaN reading at raw=%u\n", rawVal);
        return NAN;
      }
      acc += cur;
    }
    return acc / float(samples);
  };

  const float I_low = sampleAvg((uint16_t)dacLow);
  const float I_high = sampleAvg((uint16_t)dacHigh);

  if (!isfinite(I_low) || !isfinite(I_high)) {
    Serial.printf("[ElectronicLoad] autoCalibrate: invalid samples (low=%.6f high=%.6f)\n", I_low, I_high);
    writeRawDac(0);
    _isOn = false;
    _channels[channel].targetAmps = 0.0f;
    return false;
  }

  const float x1 = float(dacLow);
  const float x2 = float(dacHigh);
  const float y1 = I_low;
  const float y2 = I_high;
  const float denom = (x2 - x1);
  if (denom == 0.0f) {
    Serial.printf("[ElectronicLoad] autoCalibrate: zero denom\n");
    writeRawDac(0);
    _isOn = false;
    _channels[channel].targetAmps = 0.0f;
    return false;
  }
  const float M = (y2 - y1) / denom;
  const float B = y1 - M * x1;

  if (!isfinite(M) || !isfinite(B)) {
    Serial.printf("[ElectronicLoad] autoCalibrate: non-finite M/B (M=%.6f B=%.6f)\n", M, B);
    writeRawDac(0);
    _isOn = false;
    _channels[channel].targetAmps = 0.0f;
    return false;
  }

  _channels[channel].calibM = M;
  _channels[channel].calibB = B;

  // Optionally update max current estimate using DAC_MAX
  const float I_max_est = M * float(DAC_MAX) + B;
  if (isfinite(I_max_est) && I_max_est > 0.0f) _channels[channel].maxCurrent = I_max_est;

  Serial.printf("[ElectronicLoad] autoCalibrate: DONE ch=%u M=%.6f B=%.6f Imax~%.3fA\n",
                channel, M, B, _channels[channel].maxCurrent);

  // Restore prior state
  if (priorOn) {
    const int rawRestore = currentToDacValue(channel, priorTarget);
    writeRawDac((uint16_t)rawRestore);
    _isOn = (priorTarget > 0.0f);
    _channels[channel].targetAmps = priorTarget;
  } else {
    writeRawDac(0);
    _isOn = false;
    _channels[channel].targetAmps = 0.0f;
  }

  (void)priorRaw; // not currently used but preserved for future
  return true;
}

// --- Limits ---
void ElectronicLoad::setMaxCurrentLimit(float limitAmps) {
  // apply to default channel
  if (isValidChannel((uint8_t)_inaChannel)) _channels[_inaChannel].maxCurrent = limitAmps;
}

void ElectronicLoad::setMaxCurrentLimit(uint8_t channel, float limitAmps) {
  if (isValidChannel(channel)) _channels[channel].maxCurrent = limitAmps;
}

float ElectronicLoad::getMaxCurrentLimit() const {
  if (!isValidChannel((uint8_t)_inaChannel)) return NAN;
  return _channels[_inaChannel].maxCurrent;
}

float ElectronicLoad::getMaxCurrentLimit(uint8_t channel) const {
  if (!isValidChannel(channel)) return NAN;
  return _channels[channel].maxCurrent;
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
  // clamp using header constant
  if (value < 0) value = 0;
  if (value > int(ElectronicLoad::DAC_MAX)) value = int(ElectronicLoad::DAC_MAX);

  Wire.beginTransmission(_i2cAddress);
  Wire.write(0x40);
  uint8_t high = (value >> 4) & 0xFF;
  uint8_t low  = (value & 0x0F) << 4;
  Wire.write(high);
  Wire.write(low);
  uint8_t result = Wire.endTransmission();
  return (result == 0);
}

int ElectronicLoad::currentToDacValue(uint8_t channel, float amps) const {
  if (!isValidChannel(channel)) return 0;
  const float M = _channels[channel].calibM;
  const float B = _channels[channel].calibB;
  if (!isfinite(amps)) return 0;
  // If no calibration slope, avoid divide by zero
  float rawF = (M == 0.0f) ? 0.0f : ((amps - B) / M);
  if (!isfinite(rawF)) rawF = 0.0f;
  if (rawF < 0.0f) rawF = 0.0f;
  if (rawF > float(ElectronicLoad::DAC_MAX)) rawF = float(ElectronicLoad::DAC_MAX);
  return int(rawF + 0.5f);
}

void ElectronicLoad::writeRawDac(uint16_t raw) {
  if (raw > DAC_MAX) raw = DAC_MAX;
  writeDacValue(int(raw));
  _rawDac = raw;
}

uint16_t ElectronicLoad::getRawDac() const { return _rawDac; }