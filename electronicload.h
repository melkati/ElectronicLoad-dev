/*
  eMariete Programmable Constant Current Electronic Load
  https://github.com/melkati/eMariete-Programmable-Constant-Current-Electronic-Load

  Documentation comments added to enable automatic documentation generation.
*/

#pragma once

#include <Arduino.h>
#include <Adafruit_INA3221.h>

using TemperatureReaderCallback = float (*)(int sensorId);

/**
 * @brief ElectronicLoad provides a programmable constant-current electronic load
 *
 * This class wraps control of a DAC (MCP4725) and measurements from an INA3221
 * to implement a multi-channel programmable electronic load. It supports:
 * - Per-channel shunt configuration
 * - Per-channel calibration (linear mapping from DAC to current)
 * - Auto-calibration routine using INA3221 measurements
 * - Safety limits (max current, optional temperature monitoring)
 *
 * The class is intentionally lightweight: begin() initializes I2C devices,
 * turnOn/turnOff control the applied DAC value, update() must be called
 * periodically to apply the target current and perform safety checks.
 */
class ElectronicLoad {
public:
  /// Number of independent channels supported by the hardware + driver
  static constexpr uint8_t ChannelCount = 3;

  /**
   * @brief Construct a new ElectronicLoad object
   * @param dacAddress I2C 7-bit address of the MCP4725 DAC (e.g. 0x60)
   */
  explicit ElectronicLoad(uint8_t dacAddress);

  /**
   * @brief Initialize I2C devices (INA3221) and apply default DAC output (0)
   * @return true if initialization sequences were executed (always true on success)
   */
  bool begin();

  /** Turn the load on for the currently active channel. */
  void turnOn();
  /** Turn the load on for a specific channel. */
  void turnOn(uint8_t channel);
  /** Turn the load off (DAC set to 0). */
  void turnOff();

  /**
   * @brief Perform periodic tasks
   *
   * - Read measurements from INA3221 via `getCurrent()`/`getVoltage()`
   * - Enforce `maxCurrent` safety limit
   * - Enforce temperature emergency limit if a temperature reader is set
   * - Convert `targetAmps` to a DAC value and write it
   *
   * This function should be called frequently from `loop()` when the load is on.
   */
  void update();

  /** Set which channel will be affected by subsequent set/get calls. */
  void setActiveChannel(uint8_t channel);
  uint8_t getActiveChannel() const;

  void enableChannel(uint8_t channel);
  void disableChannel(uint8_t channel);
  bool isChannelEnabled(uint8_t channel) const;

  /**
   * @brief Set the requested target current for a channel.
   * @param channel channel index [0..ChannelCount-1]
   * @param targetAmps desired current in amperes (non-negative)
   * @return true if the value was accepted
   */
  bool setTargetCurrent(uint8_t channel, float targetAmps);
  bool setTargetCurrent(float targetAmps);
  float getTargetCurrent(uint8_t channel) const;
  float getTargetCurrent() const;

  /**
   * @brief Per-channel maximum allowed current (safety cut-off)
   * @param channel channel index
   * @param limitAmps positive current limit in amperes
   */
  void setMaxCurrentLimit(uint8_t channel, float limitAmps);
  void setMaxCurrentLimit(float limitAmps);
  float getMaxCurrentLimit(uint8_t channel) const;
  float getMaxCurrentLimit() const;

  /**
   * @brief Configure the shunt resistor value used for current measurement.
   * @param channel channel index
   * @param ohms shunt resistance in ohms (must be > 0)
   *
   * The INA3221 driver is updated immediately if the INA is ready.
   */
  void setShuntResistance(uint8_t channel, float ohms);
  float getShuntResistance(uint8_t channel) const;

  /**
   * @brief Calibrate the linear mapping between DAC and measured current
   * @param channel channel index
   * @param dacMin DAC value that produced currentMin (integer)
   * @param currentMin measured current at `dacMin` (amps)
   * @param dacMax DAC value that produced currentMax (integer)
   * @param currentMax measured current at `dacMax` (amps)
   *
   * After calibration, the driver uses the linear function:
   *   current = calibM * dac + calibB
   * to convert target current to DAC values.
   */
  void calibrate(uint8_t channel, int dacMin, float currentMin, int dacMax, float currentMax);
  void calibrate(int dacMin, float currentMin, int dacMax, float currentMax);

  /**
   * @brief Automatic calibration helper
   * @param channel channel to calibrate
   * @param dacLow DAC value for the low calibration point
   * @param dacHigh DAC value for the high calibration point
   * @param samples number of measurement samples per point (default 10)
   * @param delayMs delay between samples in milliseconds (default 200)
   *
   * The routine temporarily enables and turns on the channel, applies the
   * two DAC values, gathers INA3221 current measurements and computes the
   * linear calibration parameters.
   */
  bool autoCalibrate(uint8_t channel, int dacLow, int dacHigh, int samples = 10, unsigned long delayMs = 200);

  // Compatibility helpers expected by the sketch
  bool isOn() const;
  uint16_t getRawDac() const;

  // Optional wrapper that returns a bool for older sketches expecting a bool result
  bool autoCalibrateBool(uint8_t channel, int dacLow, int dacHigh, int samples = 10, unsigned long delayMs = 200);

  /** Read bus voltage for a channel (INA3221). */
  float getVoltage(uint8_t channel);
  float getVoltage();

  /** Read current for a channel (INA3221). */
  float getCurrent(uint8_t channel);
  float getCurrent();

  /** Compute real-time power for a channel (V * I). */
  float getPower(uint8_t channel);
  float getPower();

  /**
   * @brief Set a callback that returns a temperature in Celsius for a given id.
   * @param callback function pointer (returns float Celsius or NAN if unavailable)
   * @param sensorId id forwarded to the callback (useful for multiplexed readers)
   *
   * When a callback is set, the library enables temperature monitoring checks.
   */
  void setTemperatureReader(TemperatureReaderCallback callback, int sensorId);

  /** Set the emergency temperature (C) that triggers an immediate turnOff(). */
  void setTemperatureLimit(float degreesC);
  /** Query the current temperature via the configured callback (or NAN). */
  float getTemperature() const;
  bool isTemperatureMonitoringEnabled() const;

private:
  struct ChannelState {
    float shuntOhms;   ///< Shunt resistor value used by INA3221 for this channel
    float calibM;      ///< Slope (amps per DAC unit) after calibration
    float calibB;      ///< Offset (amps) after calibration
    float targetAmps;  ///< Desired current to apply when channel is active
    float maxCurrent;  ///< Safety maximum current (amps)
    bool enabled;      ///< Whether the channel is allowed to be used
  };

  bool writeDacValue(int value);
  int currentToDacValue(uint8_t channel, float amps) const;
  bool isValidChannel(uint8_t channel) const;

  uint8_t _dacAddress;
  uint8_t _activeChannel;
  bool _isOn;
  bool _inaReady;

  ChannelState _channels[ChannelCount];
  Adafruit_INA3221 _ina3221;

  TemperatureReaderCallback _tempReader;
  int _tempSensorId;
  float _tempLimitEmergency;
  bool _tempMonitoringEnabled;
};
