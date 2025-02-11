// ENS160 sensor with I2C interface from ScioSense
//
// Datasheet: https://www.sciosense.com/wp-content/uploads/documents/SC-001224-DS-7-ENS160-Datasheet.pdf
//
// Implementation based on:
//   https://github.com/sciosense/ENS160_driver

#include "ens160.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ens160 {

static const char *const TAG = "ens160";

static const uint8_t ENS160_BOOTING = 10;

static const uint16_t ENS160_PART_ID = 0x0160;

static const uint8_t ENS160_REG_PART_ID = 0x00;
static const uint8_t ENS160_REG_OPMODE = 0x10;
static const uint8_t ENS160_REG_CONFIG = 0x11;
static const uint8_t ENS160_REG_COMMAND = 0x12;
static const uint8_t ENS160_REG_TEMP_IN = 0x13;
static const uint8_t ENS160_REG_DATA_STATUS = 0x20;
static const uint8_t ENS160_REG_DATA_AQI = 0x21;
static const uint8_t ENS160_REG_DATA_TVOC = 0x22;
static const uint8_t ENS160_REG_DATA_ECO2 = 0x24;

static const uint8_t ENS160_REG_GPR_READ_0 = 0x48;
static const uint8_t ENS160_REG_GPR_READ_1 = ENS160_REG_GPR_READ_0 + 1;
static const uint8_t ENS160_REG_GPR_READ_2 = ENS160_REG_GPR_READ_0 + 2;
static const uint8_t ENS160_REG_GPR_READ_3 = ENS160_REG_GPR_READ_0 + 3;
static const uint8_t ENS160_REG_GPR_READ_4 = ENS160_REG_GPR_READ_0 + 4;
static const uint8_t ENS160_REG_GPR_READ_5 = ENS160_REG_GPR_READ_0 + 5;
static const uint8_t ENS160_REG_GPR_READ_6 = ENS160_REG_GPR_READ_0 + 6;
static const uint8_t ENS160_REG_GPR_READ_7 = ENS160_REG_GPR_READ_0 + 7;

static const uint8_t ENS160_COMMAND_NOP = 0x00;
static const uint8_t ENS160_COMMAND_CLRGPR = 0xCC;
static const uint8_t ENS160_COMMAND_GET_APPVER = 0x0E;

static const uint8_t ENS160_OPMODE_RESET = 0xF0;
static const uint8_t ENS160_OPMODE_IDLE = 0x01;
static const uint8_t ENS160_OPMODE_STD = 0x02;

static const uint8_t ENS160_DATA_STATUS_STATAS = 0x80;
static const uint8_t ENS160_DATA_STATUS_STATER = 0x40;
static const uint8_t ENS160_DATA_STATUS_VALIDITY = 0x0C;
static const uint8_t ENS160_DATA_STATUS_NEWDAT = 0x02;
static const uint8_t ENS160_DATA_STATUS_NEWGPR = 0x01;

// enable interrupts on data available
static const uint8_t ENS160_CONFIG_INT_DATA = 0x23;
static const uint8_t ENS160_CONFIG_INT_OFF = 0x20;

// helps remove reserved bits in aqi data register
static const uint8_t ENS160_DATA_AQI = 0x07;

void ENS160Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ENS160...");

  delay(ENS160_BOOTING);

  if (!this->reset_())
    return;

  if (!this->check_part_id_())
    return;

  if (!this->set_mode_(ENS160_OPMODE_IDLE))
    return;

  if (!this->clear_command_())
    return;

  if (!this->get_firmware_())
    return;

  if (!this->set_config_())
    return;

  // set mode to standard
  if (!this->set_mode_(ENS160_OPMODE_STD))
    return;
}

// Set any register value and handle errors
bool ENS160Component::set_value_(uint8_t reg, uint8_t mode) {
  // set mode to reset
  if (!this->write_byte(reg, mode)) {
    this->error_code_ = WRITE_FAILED;
    this->mark_failed();
    return false;
  }

  delay(ENS160_BOOTING);
  return true;
}

uint8_t ENS160Component::read_value_(uint8_t reg) {
  uint8_t status_value;
  if (!this->read_byte(reg, &status_value)) {
    this->error_code_ = READ_FAILED;
    this->mark_failed();
    return 0;
  }
  delay(ENS160_BOOTING);
  return status_value;
}

// Set config to use data registers, disable interrupts.
bool ENS160Component::set_config_() { return this->set_value_(ENS160_REG_CONFIG, ENS160_CONFIG_INT_OFF); }

// Sends a reset to the ENS160. Returns false on I2C problems.
bool ENS160Component::reset_() { return this->set_value_(ENS160_REG_OPMODE, ENS160_OPMODE_RESET); }

// check config value
void ENS160Component::read_config_() {
  ESP_LOGV(TAG, "Config: Byte data    0x%x", this->read_value_(ENS160_REG_CONFIG));
}

bool ENS160Component::set_mode_(uint8_t mode) {
  if (!this->set_value_(ENS160_REG_OPMODE, mode))
    return false;

  // read opmode and check it is set
  uint8_t op_mode = this->read_value_(ENS160_REG_OPMODE);
  if (this->status_has_error())
    return false;

  if (op_mode != mode) {
    this->error_code_ = STD_OPMODE_FAILED;
    this->mark_failed();
    return false;
  }

  return true;
}

// Reads the part ID and confirms valid sensor
bool ENS160Component::check_part_id_() {
  // check part_id
  uint16_t part_id;
  if (!this->read_bytes(ENS160_REG_PART_ID, reinterpret_cast<uint8_t *>(&part_id), 2)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return false;
  }
  if (part_id != ENS160_PART_ID) {
    this->error_code_ = INVALID_ID;
    this->mark_failed();
    return false;
  }

  delay(ENS160_BOOTING);

  return true;
}

// clear command
bool ENS160Component::clear_command_() {
  // clear command
  this->set_value_(ENS160_REG_COMMAND, ENS160_COMMAND_NOP);
  this->set_value_(ENS160_REG_COMMAND, ENS160_COMMAND_CLRGPR);
  if (this->status_has_error())
    return false;

  return this->check_status_();
}

// read firmware version
bool ENS160Component::get_firmware_() {
  if (!this->set_value_(ENS160_REG_COMMAND, ENS160_COMMAND_GET_APPVER))
    return false;

  this->firmware_ver_major_ = this->read_value_(ENS160_REG_GPR_READ_4);
  this->firmware_ver_minor_ = this->read_value_(ENS160_REG_GPR_READ_5);
  this->firmware_ver_build_ = this->read_value_(ENS160_REG_GPR_READ_6);

  return true;
}

bool ENS160Component::check_status_() {
  // check status
  uint8_t status_value = this->read_value_(ENS160_REG_DATA_STATUS);
  if (this->status_has_error())
    return false;

  this->validity_flag_ = static_cast<ValidityFlag>((ENS160_DATA_STATUS_VALIDITY & status_value) >> 2);

  if (this->validity_flag_ == INVALID_OUTPUT) {
    this->error_code_ = VALIDITY_INVALID;
    this->mark_failed();
    return false;
  }
  delay(ENS160_BOOTING);
  return true;
}

void ENS160Component::update() {
  uint8_t status_value, data_ready;

  if (!this->read_byte(ENS160_REG_DATA_STATUS, &status_value)) {
    ESP_LOGW(TAG, "Error reading status register");
    this->status_set_warning();
    return;
  }

  // verbose status logging
  ESP_LOGV(TAG, "Status: ENS160 STATAS bit    0x%x",
           (ENS160_DATA_STATUS_STATAS & (status_value)) == ENS160_DATA_STATUS_STATAS);
  ESP_LOGV(TAG, "Status: ENS160 STATER bit    0x%x",
           (ENS160_DATA_STATUS_STATER & (status_value)) == ENS160_DATA_STATUS_STATER);
  ESP_LOGV(TAG, "Status: ENS160 VALIDITY FLAG 0x%02x", (ENS160_DATA_STATUS_VALIDITY & status_value) >> 2);
  ESP_LOGV(TAG, "Status: ENS160 NEWDAT bit    0x%x",
           (ENS160_DATA_STATUS_NEWDAT & (status_value)) == ENS160_DATA_STATUS_NEWDAT);
  ESP_LOGV(TAG, "Status: ENS160 NEWGPR bit    0x%x",
           (ENS160_DATA_STATUS_NEWGPR & (status_value)) == ENS160_DATA_STATUS_NEWGPR);

  data_ready = ENS160_DATA_STATUS_NEWDAT & status_value;
  this->validity_flag_ = static_cast<ValidityFlag>((ENS160_DATA_STATUS_VALIDITY & status_value) >> 2);

  switch (validity_flag_) {
    case NORMAL_OPERATION:
      if (ENS160_DATA_STATUS_NEWGPR & status_value) {
        ESP_LOGV(TAG, "Status: GPR data0    0x%x", this->read_value_(ENS160_REG_GPR_READ_0));
        ESP_LOGV(TAG, "Status: GPR data1    0x%x", this->read_value_(ENS160_REG_GPR_READ_1));
        ESP_LOGV(TAG, "Status: GPR data2    0x%x", this->read_value_(ENS160_REG_GPR_READ_2));
        ESP_LOGV(TAG, "Status: GPR data3    0x%x", this->read_value_(ENS160_REG_GPR_READ_3));
        ESP_LOGV(TAG, "Status: GPR data4    0x%x", this->read_value_(ENS160_REG_GPR_READ_4));
        ESP_LOGV(TAG, "Status: GPR data5    0x%x", this->read_value_(ENS160_REG_GPR_READ_5));
        ESP_LOGV(TAG, "Status: GPR data6    0x%x", this->read_value_(ENS160_REG_GPR_READ_6));
        ESP_LOGV(TAG, "Status: GPR data7    0x%x", this->read_value_(ENS160_REG_GPR_READ_7));
      }
      if (data_ready != ENS160_DATA_STATUS_NEWDAT) {
        ESP_LOGD(TAG, "ENS160 readings unavailable - Normal Operation but readings not ready.");
        this->retry_counter_++;
        if (this->retry_counter_ < 5) {
          return;
        } else {
          this->retry_counter_ = 0;
        }
        ESP_LOGD(TAG, "ENS160 readings unavailable - Reading data anyway.");
      }
      break;
    case INITIAL_STARTUP:
      if (!this->initial_startup_) {
        this->initial_startup_ = true;
        ESP_LOGI(TAG, "ENS160 readings unavailable - 1 hour startup required after first power on");
      }
      return;
    case WARMING_UP:
      if (!this->warming_up_) {
        this->warming_up_ = true;
        ESP_LOGI(TAG, "ENS160 readings not available yet - Warming up requires 3 minutes");
        this->send_env_data_();
      }
      return;
    case INVALID_OUTPUT:
      ESP_LOGE(TAG, "ENS160 Invalid Status - No Invalid Output");
      this->status_set_warning();
      return;
  }

  // read new data
  uint16_t data_eco2;
  if (!this->read_bytes(ENS160_REG_DATA_ECO2, reinterpret_cast<uint8_t *>(&data_eco2), 2)) {
    ESP_LOGW(TAG, "Error reading eCO2 data register");
    this->status_set_warning();
    return;
  }
  if (this->co2_ != nullptr) {
    this->co2_->publish_state(data_eco2);
  }

  uint16_t data_tvoc;
  if (!this->read_bytes(ENS160_REG_DATA_TVOC, reinterpret_cast<uint8_t *>(&data_tvoc), 2)) {
    ESP_LOGW(TAG, "Error reading TVOC data register");
    this->status_set_warning();
    return;
  }
  if (this->tvoc_ != nullptr) {
    this->tvoc_->publish_state(data_tvoc);
  }

  uint8_t data_aqi;
  if (!this->read_byte(ENS160_REG_DATA_AQI, &data_aqi)) {
    ESP_LOGW(TAG, "Error reading AQI data register");
    this->status_set_warning();
    return;
  }
  if (this->aqi_ != nullptr) {
    // remove reserved bits, just in case they are used in future
    data_aqi = ENS160_DATA_AQI & data_aqi;

    this->aqi_->publish_state(data_aqi);
  }

  this->status_clear_warning();

  // set temperature and humidity compensation data
  this->send_env_data_();
}

void ENS160Component::send_env_data_() {
  if (this->temperature_ == nullptr && this->humidity_ == nullptr)
    return;

  float temperature = NAN;
  if (this->temperature_ != nullptr)
    temperature = this->temperature_->state;

  if (std::isnan(temperature) || temperature < -40.0f || temperature > 85.0f) {
    ESP_LOGW(TAG, "Invalid external temperature - compensation values not updated");
    return;
  } else {
    ESP_LOGV(TAG, "External temperature compensation: %.1f°C", temperature);
  }

  float humidity = NAN;
  if (this->humidity_ != nullptr)
    humidity = this->humidity_->state;

  if (std::isnan(humidity) || humidity < 0.0f || humidity > 100.0f) {
    ESP_LOGW(TAG, "Invalid external humidity - compensation values not updated");
    return;
  } else {
    ESP_LOGV(TAG, "External humidity compensation:    %.1f%%", humidity);
  }

  uint16_t t = (uint16_t) ((temperature + 273.15f) * 64.0f);
  uint16_t h = (uint16_t) (humidity * 512.0f);

  uint8_t data[4];
  data[0] = t & 0xff;
  data[1] = (t >> 8) & 0xff;
  data[2] = h & 0xff;
  data[3] = (h >> 8) & 0xff;

  if (!this->write_bytes(ENS160_REG_TEMP_IN, data, 4)) {
    ESP_LOGE(TAG, "Error writing compensation values");
    this->status_set_warning();
    return;
  }
}

void ENS160Component::dump_config() {
  ESP_LOGCONFIG(TAG, "ENS160:");

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Communication failed! Is the sensor connected?");
      break;
    case READ_FAILED:
      ESP_LOGE(TAG, "Error reading from register");
      break;
    case WRITE_FAILED:
      ESP_LOGE(TAG, "Error writing to register");
      break;
    case INVALID_ID:
      ESP_LOGE(TAG, "Sensor reported an invalid ID. Is this a ENS160?");
      break;
    case VALIDITY_INVALID:
      ESP_LOGE(TAG, "Invalid Device Status - No valid output");
      break;
    case STD_OPMODE_FAILED:
      ESP_LOGE(TAG, "Device failed to achieve Standard Operating Mode");
      break;
    case NONE:
      ESP_LOGD(TAG, "Setup successful");
      break;
  }
  this->read_config_();
  ESP_LOGI(TAG, "Firmware Version: %d.%d.%d", this->firmware_ver_major_, this->firmware_ver_minor_,
           this->firmware_ver_build_);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "CO2 Sensor:", this->co2_);
  LOG_SENSOR("  ", "TVOC Sensor:", this->tvoc_);
  LOG_SENSOR("  ", "AQI Sensor:", this->aqi_);

  if (this->temperature_ != nullptr && this->humidity_ != nullptr) {
    LOG_SENSOR("  ", "  Temperature Compensation:", this->temperature_);
    LOG_SENSOR("  ", "  Humidity Compensation:", this->humidity_);
  } else {
    ESP_LOGCONFIG(TAG, "  Compensation: Not configured");
  }
}

}  // namespace ens160
}  // namespace esphome
