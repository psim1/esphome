#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ens160 {

/// This class implements support for the ENS160 relative humidity and temperature i2c sensor.
class ENS160Component : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
  void set_co2(sensor::Sensor *co2) { co2_ = co2; }
  void set_tvoc(sensor::Sensor *tvoc) { tvoc_ = tvoc; }
  void set_aqi(sensor::Sensor *aqi) { aqi_ = aqi; }

  void set_humidity(sensor::Sensor *humidity) { humidity_ = humidity; }
  void set_temperature(sensor::Sensor *temperature) { temperature_ = temperature; }

  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void send_env_data_();
  bool reset_();
  bool check_part_id_();
  bool clear_command_();
  bool check_status_();
  bool set_mode_(uint8_t /*mode*/);
  bool get_firmware_();
  bool set_config_();
  void read_config_();
  bool set_value_(uint8_t /*reg*/, uint8_t /*mode*/);
  uint8_t read_value_(uint8_t /*reg*/);

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    INVALID_ID,
    VALIDITY_INVALID,
    READ_FAILED,
    WRITE_FAILED,
    STD_OPMODE_FAILED,
  } error_code_{NONE};

  enum ValidityFlag {
    NORMAL_OPERATION = 0,
    WARMING_UP,
    INITIAL_STARTUP,
    INVALID_OUTPUT,
  } validity_flag_;

  uint8_t retry_counter_{0};

  bool warming_up_{false};
  bool initial_startup_{false};

  uint8_t firmware_ver_major_{0};
  uint8_t firmware_ver_minor_{0};
  uint8_t firmware_ver_build_{0};

  sensor::Sensor *co2_{nullptr};
  sensor::Sensor *tvoc_{nullptr};
  sensor::Sensor *aqi_{nullptr};

  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *temperature_{nullptr};
};

}  // namespace ens160
}  // namespace esphome
