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
  // Constructor: MCP4725 I2C address, INA3221 channel (1, 2, or 3)
  ElectronicLoad(uint8_t i2cAddress, int inaChannel, float shuntOhms = 0.1f);

  bool begin();
  void turnOn();
  void turnOff();
  void update();

  void calibrate(int dacMin, float currentMeasMin, int dacMax, float currentMeasMax);

  // Automatic calibration using INA3221 readings.
  // Applies two DAC values, measures current with INA3221, and calibrates the load
  void autoCalibrate(int dac1, int dac2, int samples = 10, unsigned long delayMs = 200);

  bool setTargetCurrent(float targetAmps);
  void setMaxCurrentLimit(float limitAmps);

  void setTemperatureReader(TemperatureReaderCallback callback, int sensorId);
  void setTemperatureLimit(float degreesC);

  float getVoltage();
  float getCurrent();
  float getTargetCurrent() const;
  float getTemperature();
  bool isOn() const;
  bool isTemperatureMonitoringEnabled() const;
  float getPower();
  float getMaxCurrentLimit() const;

  void writeRawDac(int value);

private:
  uint8_t _i2cAddress;
  int _inaChannel;
  Adafruit_INA3221 _ina3221;

  float _calib_m;
  float _calib_b;

  bool _isOn;
  float _targetAmps;
  float _maxCurrentLimit;
  float _shuntOhms;

  TemperatureReaderCallback _tempReader;
  int _tempSensorId;
  float _tempLimitEmergency;
  bool _tempMonitoringEnabled;

  bool writeDacValue(int value);
  int currentToDacValue(float amps) const;
};