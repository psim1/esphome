#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ens160 {

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

  uint16_t read_u16_le_(uint8_t a_register);
  uint8_t read_u8_(uint8_t a_register);

  enum Operation {
    HW_ID = 0x00,  // 2 byte Device Identity 0x01, 0x60
    SET_OPMODE = 0x10,
    SEND_COMMAND = 0x12,
    SET_TEMP = 0x13,       // 2 bytes Host Ambient Temperature Information
    SET_RH = 0x15,         // 2 bytes Host Relative Humidity Information
    DATA_AQI = 0x21,       // 1 byte Air Quality Index - range from 0 to 7, values from 1 to 5
    DATA_TVOC = 0x22,      // 2 bytes TVOC Concentration (ppb). Range 0 to 65,000
    DATA_ECO2 = 0x24,      // 2 bytes Equivalent CO2 Concentration (ppm). Range 400 to 65,000
    DEVICE_STATUS = 0x20,  // 1 byte Operating Mode
  };

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

  bool warming_up_{false};
  bool initial_startup_{false};

  uint8_t firmware_ver_major_{0};
  uint8_t firmware_ver_minor_{0};
  uint8_t firmware_ver_build_{0};

  enum DataValidity { OK = 0, WARM_UP, START_UP, INVALID };

  enum OpMode { DEEP_SLEEP = 0, IDLE, SENSING_MODE, RESET = 0xF0 };

  enum SendCommand { NOP = 0, GET_APPVER = 0x0E, CLEAR_REGISTERS = 0xCC };

  enum GPRRead {
    Register0 = 0x48,
    Register1,
    Register2,
    Register3,
    Register4,
    Register5,
    Register6,
    Register7,
  };

  sensor::Sensor *co2_{nullptr};
  sensor::Sensor *tvoc_{nullptr};
  sensor::Sensor *aqi_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  sensor::Sensor *temperature_{nullptr};
};

}  // namespace ens160
}  // namespace esphome
