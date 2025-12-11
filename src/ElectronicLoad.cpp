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

// Constructor: i2cAddress (MCP4725), inaChannel (INA3221 channel: 1, 2, or 3)
ElectronicLoad::ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms)
  : _i2cAddress(i2cAddress),
    _inaChannel(inaChannel),
    _ina3221(),
    _calib_m(0.0f),
    _calib_b(0.0f),
    _isOn(false),
    _targetAmps(0.0f),
    _maxCurrentLimit(10.0f),
    _tempReader(nullptr),
    _tempSensorId(0),
    _tempLimitEmergency(85.0f),
    _tempMonitoringEnabled(false),
    _shuntOhms(shuntOhms) {}

bool ElectronicLoad::begin() {
  Wire.begin();
  _ina3221.begin();
  // Configura el valor del shunt para el canal seleccionado
  _ina3221.setShuntResistance(_inaChannel, _shuntOhms);
  writeDacValue(0);
  return true;
}

void ElectronicLoad::turnOn() {
  _isOn = true;
  int dacValue = currentToDacValue(_targetAmps);
  writeDacValue(dacValue);
}

void ElectronicLoad::turnOff() {
  _isOn = false;
  writeDacValue(0);
}

void ElectronicLoad::update() {
  if (!_isOn) return;

  // Over Current Protection (OCP)
  float current = getCurrent();
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

  // Apply target current to DAC
  int dacValue = currentToDacValue(_targetAmps);
  writeDacValue(dacValue);
}

void ElectronicLoad::calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax) {
  if (dacMax == dacMin) {
    _calib_m = 0.0f;
    _calib_b = 0.0f;
    return;
  }
  _calib_m = (currentMeasMax - currentMeasMin) / float(dacMax - dacMin);
  _calib_b = currentMeasMin - _calib_m * float(dacMin);
}

bool ElectronicLoad::setTargetCurrent(float targetAmps) {
  if (targetAmps < 0.0f) return false;
  _targetAmps = targetAmps;
  if (_isOn) {
    int dacValue = currentToDacValue(_targetAmps);
    writeDacValue(dacValue);
  }
  return true;
}

void ElectronicLoad::setMaxCurrentLimit(float limitAmps) {
  if (limitAmps > 0.0f) {
    _maxCurrentLimit = limitAmps;
  }
}

void ElectronicLoad::setTemperatureReader(TemperatureReaderCallback callback, int sensorId) {
  _tempReader = callback;
  _tempSensorId = sensorId;
  _tempMonitoringEnabled = (_tempReader != nullptr);
}

void ElectronicLoad::setTemperatureLimit(float degreesC) {
  _tempLimitEmergency = degreesC;
  _tempMonitoringEnabled = (_tempReader != nullptr);
}

float ElectronicLoad::getVoltage() {
    return _ina3221.getBusVoltage(_inaChannel);
}

float ElectronicLoad::getCurrent() {
    return _ina3221.getCurrentAmps(_inaChannel);
}

float ElectronicLoad::getPower() {
  return getVoltage() * getCurrent();
}

float ElectronicLoad::getTargetCurrent() const {
  return _targetAmps;
}

float ElectronicLoad::getTemperature() {
  if (_tempReader) {
    return _tempReader(_tempSensorId);
  }
  return NAN;
}

bool ElectronicLoad::isOn() const {
  return _isOn;
}

bool ElectronicLoad::isTemperatureMonitoringEnabled() const {
  return _tempMonitoringEnabled;
}

float ElectronicLoad::getMaxCurrentLimit() const {
  return _maxCurrentLimit;
}

// --- Private helpers ---

bool ElectronicLoad::writeDacValue(int value) {
  if (value < DAC_MIN) value = DAC_MIN;
  if (value > DAC_MAX) value = DAC_MAX;

  Wire.beginTransmission(_i2cAddress);
  Wire.write(0x40); // Write DAC register (fast mode to DAC)
  uint8_t high = (value >> 4) & 0xFF;             // D11..D4
  uint8_t low  = (value & 0x0F) << 4;             // D3..D0 followed by 4 'x' bits
  Wire.write(high);
  Wire.write(low);
  uint8_t result = Wire.endTransmission();
  return (result == 0);
}

int ElectronicLoad::currentToDacValue(float amps) const {
  if (_calib_m == 0.0f) {
    return 0;
  }
  float dacFloat = (amps - _calib_b) / _calib_m;
  if (dacFloat < float(DAC_MIN)) dacFloat = float(DAC_MIN);
  if (dacFloat > float(DAC_MAX)) dacFloat = float(DAC_MAX);
  return int(dacFloat + 0.5f);
}

void ElectronicLoad::autoCalibrate(int dac1, int dac2, int samples, unsigned long delayMs) {
    int dacValues[2] = {dac1, dac2};
    float currents[2] = {0.0f, 0.0f};

    turnOn();
    for (int idx = 0; idx < 2; ++idx) {
        int dac = dacValues[idx];
        writeRawDac(dac); // Aplica el valor de DAC directamente
        delay(500);

        float sumI = 0.0f;
        for (int i = 0; i < samples; ++i) {
            delay(delayMs);
            sumI += getCurrent();
        }
        currents[idx] = sumI / float(samples);
        Serial.print("DAC="); Serial.print(dac);
        Serial.print(" MeasuredCurrent="); Serial.print(currents[idx], 6);
        Serial.println(" A");
    }

    calibrate(dac1, currents[0], dac2, currents[1]);
    Serial.println("Calibration complete.");
    Serial.print("m = "); Serial.println((currents[1] - currents[0]) / float(dac2 - dac1), 8);
    Serial.print("b = "); Serial.println(currents[0] - ((currents[1] - currents[0]) / float(dac2 - dac1)) * dac1, 8);

    turnOff();
}

void ElectronicLoad::writeRawDac(int value) {
    writeDacValue(value);
}