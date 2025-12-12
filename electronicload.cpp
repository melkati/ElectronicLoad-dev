/*
  eMariete Programmable Constant Current Electronic Load
  Implementation with added inline documentation and clarifying comments.
*/

#include "ElectronicLoad.h"
#include <math.h>
#include <Wire.h>

namespace {
constexpr float kDefaultShuntOhms = 0.1f;
constexpr float kDefaultSlope = 0.000792f; // Derived from DAC=160->0.132320A and DAC=400->0.322400A // Safe placeholder; real device must be calibrated
constexpr float kDefaultOffset = 0.005600f; // Same calibration, offset term
constexpr float kDefaultMaxCurrent = 1.0f;
constexpr int kDacMin = 0;
constexpr int kDacMax = 4095;

/**
 * @brief Clamp a float to a specified range.
 * @param value input value
 * @param minValue lower bound
 * @param maxValue upper bound
 * @return clamped value
 */
inline float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}
}

ElectronicLoad::ElectronicLoad(uint8_t dacAddress)
    : _dacAddress(dacAddress), _activeChannel(0), _isOn(false), _inaReady(false),
      _tempReader(nullptr), _tempSensorId(0), _tempLimitEmergency(85.0f), _tempMonitoringEnabled(false) {
  // Initialize each channel with reasonable defaults. Users should call
  // `setShuntResistance()` and `calibrate()` to obtain accurate behavior.
  for (uint8_t ch = 0; ch < ChannelCount; ++ch) {
    _channels[ch] = ChannelState{ kDefaultShuntOhms, kDefaultSlope, kDefaultOffset, 0.0f, kDefaultMaxCurrent, true };
  }
}

/**
 * @brief Initialize I2C, INA3221 driver and set initial DAC output to zero.
 * @return true on successful initialization
 */
bool ElectronicLoad::begin() {
  Wire.begin();
  _ina3221.begin();
  _inaReady = true;
  // Propagate configured shunt resistances to the INA driver
  for (uint8_t ch = 0; ch < ChannelCount; ++ch) {
    _ina3221.setShuntResistance(ch, _channels[ch].shuntOhms);
  }
  // Ensure DAC output is zero (no load) on startup
  writeDacValue(0);
  return true;
}

void ElectronicLoad::turnOn() {
  turnOn(_activeChannel);
}

/**
 * @brief Activate a channel and write its current target to the DAC.
 * If the channel is disabled or invalid, the call is ignored.
 */
void ElectronicLoad::turnOn(uint8_t channel) {
  if (!isValidChannel(channel) || !_channels[channel].enabled) {
    return;
  }
  _activeChannel = channel;
  _isOn = true;
  int dacValue = currentToDacValue(channel, _channels[channel].targetAmps);
  writeDacValue(dacValue);
}

/** Turn the load off and set DAC to 0 to remove load. */
void ElectronicLoad::turnOff() {
  _isOn = false;
  writeDacValue(0);
}

/**
 * @brief Main periodic worker that keeps the hardware in sync with requested state.
 *
 * - If off, does nothing.
 * - Verifies channel validity and safety limits.
 * - Reads temperature (via user callback) if enabled and guards against overtemperature.
 * - Computes DAC value from target current and writes it.
 */
void ElectronicLoad::update() {
  if (!_isOn) {
    return;
  }

  const uint8_t channel = _activeChannel;
  if (!isValidChannel(channel) || !_channels[channel].enabled) {
    // Turn off if the active channel is invalid or disabled
    turnOff();
    return;
  }

  // Safety: check measured current against configured maxCurrent
  float current = getCurrent(channel);
  if (!isnan(current) && current >= _channels[channel].maxCurrent) {
    turnOff();
    if (Serial) {
      Serial.println(F("[ElectronicLoad] ERROR: Overcurrent detected. Load turned off."));
    }
    return;
  }

  // Optional temperature monitoring using the user-provided callback
  if (_tempMonitoringEnabled && _tempReader) {
    float tempC = _tempReader(_tempSensorId);
    if (!isnan(tempC) && tempC >= _tempLimitEmergency) {
      turnOff();
      if (Serial) {
        Serial.println(F("[ElectronicLoad] ERROR: Overtemperature detected. Load turned off."));
      }
      return;
    }
  }

  // Compute and apply the DAC value corresponding to the target current
  int dacValue = currentToDacValue(channel, _channels[channel].targetAmps);
  writeDacValue(dacValue);
}

void ElectronicLoad::setActiveChannel(uint8_t channel) {
  if (!isValidChannel(channel)) {
    return;
  }
  // If the load is currently on, switching the active channel re-applies
  // the DAC value for the newly selected channel.
  if (_isOn) {
    turnOn(channel);
  } else {
    _activeChannel = channel;
  }
}

uint8_t ElectronicLoad::getActiveChannel() const {
  return _activeChannel;
}

void ElectronicLoad::enableChannel(uint8_t channel) {
  if (isValidChannel(channel)) {
    _channels[channel].enabled = true;
  }
}

void ElectronicLoad::disableChannel(uint8_t channel) {
  if (isValidChannel(channel)) {
    _channels[channel].enabled = false;
    // If we disabled an active channel while on, immediately turn off.
    if (_isOn && _activeChannel == channel) {
      turnOff();
    }
  }
}

bool ElectronicLoad::isChannelEnabled(uint8_t channel) const {
  return isValidChannel(channel) ? _channels[channel].enabled : false;
}

/**
 * @brief Set the desired target current for a channel.
 * If the load is currently on for that channel, the DAC is updated immediately.
 */
bool ElectronicLoad::setTargetCurrent(uint8_t channel, float targetAmps) {
  if (!isValidChannel(channel) || targetAmps < 0.0f) {
    return false;
  }
  _channels[channel].targetAmps = targetAmps;
  if (_isOn && _activeChannel == channel) {
    int dacValue = currentToDacValue(channel, targetAmps);
    writeDacValue(dacValue);
  }
  return true;
}

bool ElectronicLoad::setTargetCurrent(float targetAmps) {
  return setTargetCurrent(_activeChannel, targetAmps);
}

float ElectronicLoad::getTargetCurrent(uint8_t channel) const {
  return isValidChannel(channel) ? _channels[channel].targetAmps : NAN;
}

float ElectronicLoad::getTargetCurrent() const {
  return getTargetCurrent(_activeChannel);
}

void ElectronicLoad::setMaxCurrentLimit(uint8_t channel, float limitAmps) {
  if (isValidChannel(channel) && limitAmps > 0.0f) {
    _channels[channel].maxCurrent = limitAmps;
  }
}

void ElectronicLoad::setMaxCurrentLimit(float limitAmps) {
  setMaxCurrentLimit(_activeChannel, limitAmps);
}

float ElectronicLoad::getMaxCurrentLimit(uint8_t channel) const {
  return isValidChannel(channel) ? _channels[channel].maxCurrent : NAN;
}

float ElectronicLoad::getMaxCurrentLimit() const {
  return getMaxCurrentLimit(_activeChannel);
}

void ElectronicLoad::setShuntResistance(uint8_t channel, float ohms) {
  if (!isValidChannel(channel) || ohms <= 0.0f) {
    return;
  }
  _channels[channel].shuntOhms = ohms;
  // Update the INA3221 if it's already initialized
  if (_inaReady) {
    _ina3221.setShuntResistance(channel, ohms);
  }
}

float ElectronicLoad::getShuntResistance(uint8_t channel) const {
  return isValidChannel(channel) ? _channels[channel].shuntOhms : NAN;
}

/**
 * @brief Set calibration parameters using two known DAC/current pairs.
 * The function computes a linear mapping slope (calibM) and offset (calibB).
 */
void ElectronicLoad::calibrate(uint8_t channel, int dacMin, float currentMin, int dacMax, float currentMax) {
  if (!isValidChannel(channel) || dacMin == dacMax) {
    return;
  }
  ChannelState &state = _channels[channel];
  state.calibM = (currentMax - currentMin) / float(dacMax - dacMin);
  state.calibB = currentMin - state.calibM * float(dacMin);
}

void ElectronicLoad::calibrate(int dacMin, float currentMin, int dacMax, float currentMax) {
  calibrate(_activeChannel, dacMin, currentMin, dacMax, currentMax);
}

/**
 * @brief Automatically derive calibration parameters by applying two DAC values
 * and measuring the resulting currents using the INA3221.
 *
 * The routine:
 * - Enables and turns on the requested channel
 * - Applies `dacLow` and `dacHigh`
 * - Takes `samples` measurements at each DAC value separated by `delayMs`
 * - Computes average currents and calls `calibrate()` when valid data is available
 */
bool ElectronicLoad::autoCalibrate(uint8_t channel, int dacLow, int dacHigh, int samples, unsigned long delayMs) {
  if (!isValidChannel(channel) || samples <= 0 || dacLow == dacHigh) {
    return false;
  }

  const int dacRequest[2] = { dacLow, dacHigh };
  int appliedDac[2] = { 0, 0 };
  float measuredCurrents[2] = { NAN, NAN };
  bool wasOn = _isOn;
  uint8_t previousChannel = _activeChannel;

  enableChannel(channel);
  turnOn(channel);

  for (int idx = 0; idx < 2; ++idx) {
    int dacValue = static_cast<int>(clampFloat(static_cast<float>(dacRequest[idx]), static_cast<float>(kDacMin), static_cast<float>(kDacMax)));
    appliedDac[idx] = dacValue;
    writeDacValue(dacValue);
    delay(500); // allow settling

    float sumCurrent = 0.0f;
    int validSamples = 0;
    for (int i = 0; i < samples; ++i) {
      delay(delayMs);
      float current = getCurrent(channel);
      if (!isnan(current)) {
        sumCurrent += current;
        ++validSamples;
      }
    }
    if (validSamples > 0) {
      measuredCurrents[idx] = sumCurrent / float(validSamples);
    }
    if (Serial) {
      Serial.print(F("[ElectronicLoad] autoCalibrate DAC="));
      Serial.print(dacValue);
      Serial.print(F(" current="));
      Serial.println(measuredCurrents[idx], 6);
    }
  }

  // If both measurements are valid and distinct, compute calibration line
  if (!isnan(measuredCurrents[0]) && !isnan(measuredCurrents[1]) && measuredCurrents[0] != measuredCurrents[1]) {
    calibrate(channel, appliedDac[0], measuredCurrents[0], appliedDac[1], measuredCurrents[1]);
    if (Serial) {
      Serial.println(F("[ElectronicLoad] autoCalibrate complete."));
    }
    return true;
  } else if (Serial) {
    Serial.println(F("[ElectronicLoad] autoCalibrate skipped (invalid measurements)."));
  }

  // Restore previous on/off/channel state
  if (!wasOn) {
    turnOff();
  } else {
    turnOn(previousChannel);
  }
  return false;
}

/** Read the bus voltage from the INA3221 for a channel. */
float ElectronicLoad::getVoltage(uint8_t channel) {
  return isValidChannel(channel) ? _ina3221.getBusVoltage(channel) : NAN;
}

float ElectronicLoad::getVoltage() {
  return getVoltage(_activeChannel);
}

/** Read current (in amps) from the INA3221 for a channel. */
float ElectronicLoad::getCurrent(uint8_t channel) {
  return isValidChannel(channel) ? _ina3221.getCurrentAmps(channel) : NAN;
}

float ElectronicLoad::getCurrent() {
  return getCurrent(_activeChannel);
}

/** Compute instantaneous power for the channel (V * I). */
float ElectronicLoad::getPower(uint8_t channel) {
  float voltage = getVoltage(channel);
  float current = getCurrent(channel);
  return (isnan(voltage) || isnan(current)) ? NAN : voltage * current;
}

float ElectronicLoad::getPower() {
  return getPower(_activeChannel);
}

/**
 * @brief Configure temperature callback and enable monitoring.
 * The callback should return a Celsius temperature or NAN if unavailable.
 */
void ElectronicLoad::setTemperatureReader(TemperatureReaderCallback callback, int sensorId) {
  _tempReader = callback;
  _tempSensorId = sensorId;
  _tempMonitoringEnabled = (_tempReader != nullptr);
}

void ElectronicLoad::setTemperatureLimit(float degreesC) {
  _tempLimitEmergency = degreesC;
}

/** Query temperature via the configured callback. */
float ElectronicLoad::getTemperature() const {
  return (_tempReader) ? _tempReader(_tempSensorId) : NAN;
}

bool ElectronicLoad::isTemperatureMonitoringEnabled() const {
  return _tempMonitoringEnabled;
}

/**
 * @brief Write a 12-bit value to the MCP4725 DAC using fast-mode registers.
 * @param value DAC value expected in the range [kDacMin..kDacMax]
 * @return true if I2C transmission succeeded (endTransmission == 0)
 */
bool ElectronicLoad::writeDacValue(int value) {
  int clamped = static_cast<int>(clampFloat(static_cast<float>(value), static_cast<float>(kDacMin), static_cast<float>(kDacMax)));
  Wire.beginTransmission(_dacAddress);
  Wire.write(0x40); // Fast write command for MCP4725
  Wire.write((clamped >> 4) & 0xFF);
  Wire.write((clamped & 0x0F) << 4);
  return Wire.endTransmission() == 0;
}

/**
 * @brief Convert requested amps to DAC units using stored calibration.
 * @return nearest integer DAC value, clamped to valid range
 */
int ElectronicLoad::currentToDacValue(uint8_t channel, float amps) const {
  if (!isValidChannel(channel)) {
    return 0;
  }
  const ChannelState &state = _channels[channel];
  if (state.calibM == 0.0f) {
    // Avoid division by zero: no calibration available
    return 0;
  }
  float dacFloat = (amps - state.calibB) / state.calibM;
  dacFloat = clampFloat(dacFloat, static_cast<float>(kDacMin), static_cast<float>(kDacMax));
  return static_cast<int>(dacFloat + 0.5f);
}

/** Validate channel index. */
bool ElectronicLoad::isValidChannel(uint8_t channel) const {
  return channel < ChannelCount;
}

bool ElectronicLoad::isOn() const { return _isOn; }

uint16_t ElectronicLoad::getRawDac() const {
  return static_cast<uint16_t>( currentToDacValue(_activeChannel, _channels[_activeChannel].targetAmps) );
}

// // Wrapper that keeps existing void autoCalibrate(...) and provides a bool-returning helper
// bool ElectronicLoad::autoCalibrateBool(uint8_t channel, int dacLow, int dacHigh, int samples, unsigned long delayMs) {
//   autoCalibrate(channel, dacLow, dacHigh, samples, delayMs);
//   // The current implementation cannot reliably report failure; return true as best-effort.
//   // If you later compute success/failure, update this to return false on failure.
//   return true;
// }












