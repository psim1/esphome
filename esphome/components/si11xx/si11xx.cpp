// ENS160 sensor with I2C interface from ScioSense
//
// Datasheet: https://www.sciosense.com/wp-content/uploads/documents/SC-001224-DS-7-ENS160-Datasheet.pdf
//
// Implementation based on:
//   http://fabo.io/206.html

#include "si11xx.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace si11xx {

static const char *const TAG = "si11xx";

static const uint8_t SI11X_BOOT_DELAY = 30;
static const uint8_t SI11X_DELAY = 2;

static const uint8_t SI_REG_PARTID = 0x00;  // Device type register
static const uint8_t SI_REG_REVID = 0x01;   // Device revision
static const uint8_t SI_REG_SEQID = 0x02;   // Device seq
static const uint8_t SI_INT_CFG = 0x03;     // INT Output Enable
static const uint8_t SI_IRQ_ENABLE = 0x04;  // ALS Interrupt Enable
static const uint8_t SI1132_DEVICE = 0x32;  // Device types supported
static const uint8_t SI1145_DEVICE = 0x45;

// Registers
static const uint8_t SI_REG_HW_KEY = 0x07;
static uint8_t SI_REG_MEASRATE0 = 0x08;
static uint8_t SI_REG_MEASRATE1 = 0x09;
static const uint8_t SI_REG_INTCFG = 0x03;
static const uint8_t SI_REG_IRQEN = 0x04;
static const uint8_t SI_REG_PSLED21 = 0x0F;

// Coefficient registers
static const uint8_t SI_READ_UCOEFFS = 0x12;
static const uint8_t SI_REG_UCOEFF0 = 0x13;
static const uint8_t SI_REG_UCOEFF1 = 0x14;
static const uint8_t SI_REG_UCOEFF2 = 0x15;
static const uint8_t SI_REG_UCOEFF3 = 0x16;

// Commands
static const uint8_t SI_REG_PARAM_WR = 0x17;
static const uint8_t SI_REG_PARAM_RD = 0x2E;
static const uint8_t SI_REG_COMMAND = 0x18;
static const uint8_t SI_REG_RESPONSE = 0x20;

static const uint8_t SI1145_REG_IRQSTAT = 0x21;
static const uint8_t SI1145_REG_IRQSTAT_ALS = 0x01;
static const uint8_t REG_PS1_DATA0 = 0x26;
static const uint8_t REG_CHIP_STAT = 0x30;

static const uint8_t SI_PARAM_QUERY = 0x80;
static const uint8_t SI_PARAM_SET = 0xA0;
static const uint8_t SI_NOP = 0x00;
static const uint8_t SI_RESET = 0x01;
static const uint8_t BUSADDR = 0x02;
static const uint8_t SI_PS_FORCE = 0x05;
static const uint8_t SI_ALS_FORCE = 0x06;
static const uint8_t SI_PSALS_FORCE = 0x07;
static const uint8_t SI_PS_PAUSE = 0x09;
static const uint8_t SI_ALS_PAUSE = 0x0A;
static const uint8_t SI_PSALS_PAUSE = 0x0B;
static const uint8_t PS_AUTO = 0x0D;
static const uint8_t ALS_AUTO = 0x0E;
static const uint8_t PSALS_AUTO = 0x0F;
static const uint8_t SI_GET_CAL = 0x12;
static const uint8_t SI_HW_KEY_DEFAULT = 0x17;

// Chip list Parameter RAM Offset
static const uint8_t SI_CHIPLIST_PARAM_OFFSET = 0x01;
// Specific sensor parameter registers
static const uint8_t SI_PSLED12SEL_PARAM_OFFSET = 0x02;
static const uint8_t SI_ALS_ENCODING_PARAM_OFFSET = 0x06;
static const uint8_t SI_PS1_ADCMUX_PARAM_OFFSET = 0x07;
static const uint8_t SI_PS_ADC_COUNTER_PARAM_OFFSET = 0x0A;
static const uint8_t SI_PS_ADC_GAIN_PARAM_OFFSET = 0x0B;
static const uint8_t SI_PS_ADC_MISC_PARAM_OFFSET = 0x0C;
static const uint8_t SI_ALS_IR_ADC_MUX_PARAM_OFFSET = 0x0E;
static const uint8_t SI_AUX_ADC_MUX_PARAM_OFFSET = 0x0F;
static const uint8_t SI_ALS_VIS_ADC_COUNTER_PARAM_OFFSET = 0x10;
static const uint8_t SI_ALS_VIS_ADC_GAIN_PARAM_OFFSET = 0x11;
static const uint8_t SI_ALS_VIS_ADC_MISC_PARAM_OFFSET = 0x12;
static const uint8_t SI_ALS_IR_ADC_COUNTER_PARAM_OFFSET = 0x1D;
static const uint8_t SI_ALS_IR_ADC_GAIN_PARAM_OFFSET = 0x1E;
static const uint8_t SI_ALS_IR_ADC_MISC_PARAM_OFFSET = 0x1F;

// Chip select registers
static const uint8_t SI_CHIPLIST_EN_UV = 0x80;
static const uint8_t SI_CHIPLIST_EN_AUX = 0x40;
static const uint8_t SI_CHIPLIST_EN_ALS_IR = 0x20;
static const uint8_t SI_CHIPLIST_EN_ALS_VIS = 0x10;
static const uint8_t SI_CHIPLIST_EN_PS3 = 0x04;
static const uint8_t SI_CHIPLIST_EN_PS2 = 0x02;
static const uint8_t SI_CHIPLIST_EN_PS1 = 0x01;

// Visible Data register
static const uint8_t SI_REG_VISIBLE_DATA = 0x22;
// IR Data register
static const uint8_t SI_REG_IR_DATA = 0x24;
// Proximity Data register
static const uint8_t SI_REG_PROX_DATA = 0x26;
// Auxiliary Data register
static const uint8_t SI_REG_UV_DATA = 0x2C;

static const uint8_t SI_I2C_PARAM_OFFSET = 0x00;

// ALS VIS ALIGN
static const uint8_t SI_ALS_VIS_ALIGN = 0x10;
// ALS IR ALIGN
static const uint8_t SI_ALS_IR_ALIGN = 0x20;

// ADC Clock 1 : 50 ns
static const uint8_t SI_001_ADC_CLOCK = 0x00;
// ADC Clock 7 : 350 ns
static const uint8_t SI_007_ADC_CLOCK = 0x10;
// ADC Clock 15 : 750 ns
static const uint8_t SI_015_ADC_CLOCK = 0x20;
// ADC Clock 31 : 1.15 us
static const uint8_t SI_031_ADC_CLOCK = 0x30;
// ADC Clock 63 : 3.15 us
static const uint8_t SI_063_ADC_CLOCK = 0x40;
// ADC Clock 127 : 6.35 us
static const uint8_t SI_127_ADC_CLOCK = 0xA0;
// ADC Clock 255 : 12.75 us
static const uint8_t SI_255_ADC_CLOCK = 0x60;
// ADC Clock 511 : 25.55 us
static const uint8_t SI_511_ADC_CLOCK = 0x70;

// Divided ADC Clocks
static const uint8_t SI_01_DIVIDED_ADC_CLOCK = 0x00;
static const uint8_t SI_16_DIVIDED_ADC_CLOCK = 0x04;
static const uint8_t SI_64_DIVIDED_ADC_CLOCK = 0x06;

// Normal signal range
static const uint8_t SI_NORMAL_SIGNAL_RANGE = 0x00;
// High signal range
static const uint8_t SI_HIGH_SIGNAL_RANGE = 0x20;

// ALS IR Adcmux SMALLIR
static const uint8_t SI_ALS_IR_ADCMUX_SMALLIR = 0x00;

// ADCMUX options
static const uint8_t SI_AUX_ADCMUX_TEMPERATURE = 0x65;
static const uint8_t SI_AUX_ADCMUX_VOLTAGE = 0x75;

// Parameter values
static const uint8_t SI_PSLED12SEL_PS1NONE = 0x00;
static const uint8_t SI_PSLED12SEL_PS1LED1 = 0x01;
static const uint8_t SI_PSLED12SEL_PS1LED2 = 0x02;
static const uint8_t SI_PSLED12SEL_PS1LED3 = 0x04;
static const uint8_t SI_PSADCMISC_PSMODE = 0x04;
static const uint8_t SI_ADCMUX_SMALL_IR_PHOTDIODE = 0x00;
static const uint8_t SI_ADCMUX_LARGE_IR_PHOTDIODE = 0x03;
static const uint8_t SI_INTCFG_INTOE = 0x01;
static const uint8_t SI_INTCFG_INTMODE = 0x02;
static const uint8_t SI_IRQEN_ALSEVERYSAMPLE = 0x01;
static const uint8_t SI_IRQEN_PS1EVERYSAMPLE = 0x04;
static const uint8_t SI_IRQEN_PS2EVERYSAMPLE = 0x08;
static const uint8_t SI_IRQEN_PS3EVERYSAMPLE = 0x10;

static const uint8_t LOOP_TIMEOUT_MS = 30;
static const int8_t ALIGN_LEFT = 1;
static const int8_t ALIGN_RIGHT = -1;

uint8_t SI11xComponent::read_value_(uint8_t reg) {
  uint8_t data;
  if (!this->read_byte(reg, &data)) {
    this->error_code_ = READ_FAILED;
    this->mark_failed();
    return 0;
  }
  delay(SI11X_DELAY);
  return data;
}

/*uint16_t SI11xComponent::read_value16_(uint8_t reg) {
  uint16_t data;
  if (!this->read_byte_16(reg, &data)) {
    this->error_code_ = READ_FAILED;
    this->mark_failed();
    return 0;
  }
  delay(SI11X_DELAY);
  return data;
}*/

uint16_t SI11xComponent::read_value16_(uint8_t reg) {
  uint8_t lsb;
  uint8_t msb;
  if (!this->read_byte(reg, &lsb)) {
    this->error_code_ = READ_FAILED;
    this->mark_failed();
    return 0;
  }
  if (!this->read_byte(reg + 1, &msb)) {
    this->error_code_ = READ_FAILED;
    this->mark_failed();
    return 0;
  }
  ESP_LOGD(TAG, "read_value16_ reg:0x%02x msb:0x%02X lsb:0x%02X = %d", reg, msb, lsb, (msb * 256) + lsb);
  delay(SI11X_DELAY);
  return (msb * 256) + lsb;
}

// Set any register value and handle errors
bool SI11xComponent::set_value_(uint8_t reg, uint8_t mode) {
  if (!this->write_byte(reg, mode)) {
    this->error_code_ = WRITE_FAILED;
    this->mark_failed();
    return false;
  }

  delay(SI11X_DELAY);
  return true;
}

void SI11xComponent::write_param_(uint8_t register_addr, uint8_t value) {
  this->set_value_(SI_REG_PARAM_WR, value);
  this->send_command_(register_addr | SI_PARAM_SET);
}

// Special command handling protocol
// Write 0x00 to command register, read and verify response
// write command, read response regeister is non-zero
bool SI11xComponent::send_command_(uint8_t register_or_value) {
  uint8_t response;
  uint8_t count = 0;
  while (count < LOOP_TIMEOUT_MS) {
    this->set_value_(SI_REG_COMMAND, SI_NOP);
    response = this->read_value_(SI_REG_RESPONSE);
    if (response == 0) {
      break;
    }
    delay(1);
    count++;
  }

  if (response != 0) {
    ESP_LOGW(TAG, "send_command_ failed to clear register for op: 0x%x", register_or_value);
  }

  if (this->set_value_(SI_REG_COMMAND, register_or_value)) {
    this->wait_until_sleep_();
    response = this->read_value_(SI_REG_RESPONSE);
    if (response != 0) {
      // successful
      return true;
    }
  }

  this->error_code_ = COMMAND_FAILED;
  this->status_momentary_error("command_failure", 1000);
  ESP_LOGE(TAG, "send_command_ failed for op: 0x%x", register_or_value);
  return false;
}

/**
 @brief software reset Si11xx
*/
void SI11xComponent::reset_() {
  delay(SI11X_BOOT_DELAY);  // minimum startup time
  this->set_value_(SI_REG_COMMAND, SI_RESET);
  delay(SI11X_DELAY);
  this->set_value_(SI_REG_HW_KEY, SI_HW_KEY_DEFAULT);
  delay(SI11X_DELAY);

  // clear interrupts
  this->set_value_(SI_INT_CFG, 0x00);
  delay(SI11X_DELAY);
  this->set_value_(SI_IRQ_ENABLE, 0x00);
  delay(SI11X_DELAY);
}

void SI11xComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SI11xx...");
  this->get_device_type_();
  this->reset_();
  if (this->device_type_ == SI1132_DEVICE) {
    this->configuration_1132_();
    return;
  } else if (this->device_type_ == SI1145_DEVICE) {
    this->configuration_1145_();
    return;
  }

  this->error_code_ = INVALID_ID;
  this->mark_failed();
}

void SI11xComponent::update() {
  // loop return sensor values

  // uv_sensor_ and uvi_sensor_
  float data_uv = this->read_uv_index();
  if (this->status_has_error()) {
    ESP_LOGW(TAG, "Error reading UV data");
    this->status_set_warning();
    return;
  }
  if (this->uvi_sensor_ != nullptr) {
    this->uvi_sensor_->publish_state(data_uv);
  }

  // ir_sensor_
  uint16_t data_ir = this->read_ir();
  if (this->status_has_error()) {
    ESP_LOGW(TAG, "Error reading IR data");
    this->status_set_warning();
    return;
  }
  if (this->ir_sensor_ != nullptr) {
    this->ir_sensor_->publish_state(data_ir);
  }

  // light_sensor_
  uint16_t data_light = read_visible();
  if (this->status_has_error()) {
    ESP_LOGW(TAG, "Error reading Visible light data");
    this->status_set_warning();
    return;
  }
  if (this->light_sensor_ != nullptr) {
    this->light_sensor_->publish_state(data_light);
  }

  this->status_clear_warning();
}

void SI11xComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "si11x:");

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
      ESP_LOGE(TAG, "Sensor reported an invalid ID. Is this a si11x?");
      break;
    case VALIDITY_INVALID:
      ESP_LOGE(TAG, "Invalid Device Status - No valid output");
      break;
    case COMMAND_FAILED:
      ESP_LOGE(TAG, "Device command failed");
      break;
    case NONE:
      ESP_LOGD(TAG, "Setup successful");
      break;
  }

  LOG_I2C_DEVICE(this);

  LOG_SENSOR("  ", "Light Sensor:", this->light_sensor_);
  LOG_SENSOR("  ", "IR Sensor:", this->ir_sensor_);
  LOG_SENSOR("  ", "UVI Sensor:", this->uvi_sensor_);

  LOG_UPDATE_INTERVAL(this);
  this->read_config_();
}

void SI11xComponent::get_device_type_() {
  this->device_type_ = this->read_value_(SI_REG_PARTID);
  this->device_seq_ = this->read_value_(SI_REG_SEQID);
  this->device_rev_ = this->read_value_(SI_REG_REVID);

  // fix measure rate registers if seq = 0x01. Code error noted in specification documents
  if (this->device_seq_ == 1) {
    SI_REG_MEASRATE0 = 0x0A;
    SI_REG_MEASRATE1 = 0x08;
  }
}

// check config value
void SI11xComponent::read_config_() {
  ESP_LOGI(TAG, "Type: 0x%x Rev: %d Seq: %d", this->device_type_, this->device_rev_, this->device_seq_);
  ESP_LOGD(TAG, "Measure rate: 0x%02X%02X", this->read_value_(SI_REG_MEASRATE1), this->read_value_(SI_REG_MEASRATE0));
  ESP_LOGD(TAG, "Outside mode: %d", this->outside_mode_);
  ESP_LOGD(TAG, "Proximity enabled: %d", this->proximity_led_attached_);

  uint32_t i = get_update_interval();
  uint32_t p = (int) (i / (2 * 31.25));
  uint8_t lsb = p & 0xFF;
  uint8_t msb = (p >> 8) & 0xFF;
  ESP_LOGV(TAG, "Intervals i:%d p:%d msb:0x%02X lsb:0x%02X", i, p, msb, lsb);
}

bool SI11xComponent::configuration_1132_() {
  // enable UVindex measurement coefficients!
  // this->set_calibrated_coefficients_();
  this->set_value_(SI_REG_UCOEFF0, 0x7B);
  this->set_value_(SI_REG_UCOEFF1, 0x6B);
  this->set_value_(SI_REG_UCOEFF2, 0x01);
  this->set_value_(SI_REG_UCOEFF3, 0x00);

  // SET PARAM_WR(Chiplist)
  uint8_t Chiplist = SI_CHIPLIST_EN_AUX | SI_CHIPLIST_EN_ALS_IR | SI_CHIPLIST_EN_ALS_VIS;
  // uint8_t Chiplist = SI_CHIPLIST_EN_UV | SI_CHIPLIST_EN_AUX | SI_CHIPLIST_EN_ALS_IR | SI_CHIPLIST_EN_ALS_VIS;
  this->write_param_(SI_CHIPLIST_PARAM_OFFSET, Chiplist);

  // SET PARAM_WR(ALS_ENCODING)
  this->write_param_(SI_ALS_ENCODING_PARAM_OFFSET, SI_ALS_VIS_ALIGN | SI_ALS_IR_ALIGN);

  // Visible
  this->set_ambient_light_params_();

  // IR
  this->set_infrared_params_();

  // SET AUX_ADCMUX for UV
  this->write_param_(SI_AUX_ADC_MUX_PARAM_OFFSET, SI_AUX_ADCMUX_TEMPERATURE);

  // Rate setting.
  this->set_measure_rate_();

  // Set auto run
  this->send_command_(ALS_AUTO);
  delay(10);

  // device capable and external IR attached
  this->proximity_supported_ = false;

  return true;
}

/**
 @brief Set Config
*/
bool SI11xComponent::configuration_1145_() {
  // enable UVindex measurement coefficients!
  // this->set_calibrated_coefficients_();
  this->write_param_(SI_REG_UCOEFF0, 0x29);
  this->write_param_(SI_REG_UCOEFF1, 0x89);
  this->write_param_(SI_REG_UCOEFF2, 0x02);
  this->write_param_(SI_REG_UCOEFF3, 0x00);

  // SET enabled sensors
  uint8_t Chiplist = SI_CHIPLIST_EN_UV | SI_CHIPLIST_EN_ALS_IR | SI_CHIPLIST_EN_ALS_VIS;
  if (this->proximity_led_attached_) {
    Chiplist |= SI_CHIPLIST_EN_PS1;
  }
  this->write_param_(SI_CHIPLIST_PARAM_OFFSET, Chiplist);

  this->set_proximity_params_();
  this->set_infrared_params_();
  this->set_ambient_light_params_();

  // Rate setting
  this->set_measure_rate_();

  // Set auto run
  if (this->proximity_led_attached_) {
    this->send_command_(PSALS_AUTO);
  } else {
    this->send_command_(ALS_AUTO);
  }
  delay(10);

  // device capable if external IR attached
  this->proximity_supported_ = this->proximity_led_attached_;

  return true;
}

void SI11xComponent::set_measure_rate_() {
  // Rate setting. X * 31.25us = Yms
  // Convert half update interval to polling rate
  uint32_t p = (int) (get_update_interval() / (2 * 31.25));
  uint8_t lsb = p & 0xFF;
  uint8_t msb = (p >> 8) & 0xFF;

  this->set_value_(SI_REG_MEASRATE0, lsb);  // 255 * 31.25uS = 8ms
  this->set_value_(SI_REG_MEASRATE1, msb);  // 255 * 31.25uS = 8ms
}

void SI11xComponent::set_calibrated_coefficients_() {
  SI114X_CAL_S si114x_cal;

  /* UV Coefficients */
  this->si114x_get_calibration_(&si114x_cal, 0);
  this->si114x_set_ucoef_(nullptr, &si114x_cal);

  // enable UVindex measurement coefficients!
  // writeI2c(SI_REG_UCOEFF0, readI2c_8(0x22));
  // writeI2c(SI_REG_UCOEFF1, readI2c_8(0x26));
  // writeI2c(SI_REG_UCOEFF2, readI2c_8(0x2A));
  // writeI2c(SI_REG_UCOEFF3, readI2c_8(0x2D));

  // 0x22 to 0x2D.
}

void SI11xComponent::set_proximity_params_() {
  if (!this->proximity_led_attached_)
    return;

  // enable interrupt on every sample
  this->set_value_(SI_REG_INTCFG, SI_INTCFG_INTOE);
  this->set_value_(SI_REG_IRQEN, SI_IRQEN_ALSEVERYSAMPLE);

  // program LED current
  this->set_value_(SI_REG_PSLED21, 0x03);  // 20mA for LED 1 only
  this->write_param_(SI_PS1_ADCMUX_PARAM_OFFSET, SI_ADCMUX_LARGE_IR_PHOTDIODE);
  // prox sensor #1 uses LED #1
  this->write_param_(SI_PSLED12SEL_PARAM_OFFSET, SI_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  this->write_param_(SI_PS_ADC_GAIN_PARAM_OFFSET, SI_01_DIVIDED_ADC_CLOCK);
  // take 511 clocks to measure
  this->write_param_(SI_PS_ADC_COUNTER_PARAM_OFFSET, SI_511_ADC_CLOCK);
  // in prox mode, high range
  if (this->outside_mode_) {
    this->write_param_(SI_PS_ADC_MISC_PARAM_OFFSET, SI_HIGH_SIGNAL_RANGE | SI_PSADCMISC_PSMODE);
  } else {
    this->write_param_(SI_PS_ADC_MISC_PARAM_OFFSET, SI_NORMAL_SIGNAL_RANGE | SI_PSADCMISC_PSMODE);
  }
}

void SI11xComponent::set_ambient_light_params_() {
  // fastest clocks, clock div 1
  this->write_param_(SI_ALS_VIS_ADC_GAIN_PARAM_OFFSET, SI_01_DIVIDED_ADC_CLOCK);
  // take 511 clocks to measure
  this->write_param_(SI_ALS_VIS_ADC_COUNTER_PARAM_OFFSET, SI_511_ADC_CLOCK);
  // high range mode for direct sunlight
  if (this->outside_mode_) {
    this->write_param_(SI_ALS_VIS_ADC_MISC_PARAM_OFFSET, SI_HIGH_SIGNAL_RANGE);
  } else {
    this->write_param_(SI_ALS_VIS_ADC_MISC_PARAM_OFFSET, SI_NORMAL_SIGNAL_RANGE);
  }
}

void SI11xComponent::set_infrared_params_() {
  this->write_param_(SI_ALS_IR_ADC_MUX_PARAM_OFFSET, SI_ALS_IR_ADCMUX_SMALLIR);

  // fastest clocks, clock div 1
  this->write_param_(SI_ALS_IR_ADC_GAIN_PARAM_OFFSET, SI_01_DIVIDED_ADC_CLOCK);
  // take 511 clocks to measure
  this->write_param_(SI_ALS_IR_ADC_COUNTER_PARAM_OFFSET, SI_511_ADC_CLOCK);
  // high range mode for direct sunlight
  if (this->outside_mode_) {
    this->write_param_(SI_ALS_IR_ADC_MISC_PARAM_OFFSET, SI_HIGH_SIGNAL_RANGE);
  } else {
    this->write_param_(SI_ALS_IR_ADC_MISC_PARAM_OFFSET, SI_NORMAL_SIGNAL_RANGE);
  }
}

/**
 @brief The proximity measurement
 @param [out] prox The proximity measurement
*/
uint16_t SI11xComponent::read_proximity() {
  if (!this->proximity_supported_ && this->proximity_led_attached_)
    return 0;
  return this->read_value16_(SI_REG_PROX_DATA);
}

/**
 @brief Read UV
 @param [out] uv rawdata
*/
uint16_t SI11xComponent::read_uv() {
  uint16_t uv = this->read_value16_(SI_REG_UV_DATA);
  // this->convert_data_(uv);
  return uv;
  // return this->convert_data_(uv);
}

/**
 @brief Read UV
 @param [out] uv rawdata/100 -> UV INDEX
*/
float SI11xComponent::read_uv_index() {
  float uv = read_uv() / 100.0;
  return uv;
}

/**
 @brief Read IR
 @param [out] ir data (lux)
*/
uint16_t SI11xComponent::read_ir() {
  uint16_t ir = read_value16_(SI_REG_IR_DATA);
  // return this->convert_data_(ir);
  //  ir = ((ir - 250) / 2.44) * 14.5;
  return ir;
}

/**
 @brief Read Visible
 @param [out] Visible data (lux)
*/
uint16_t SI11xComponent::read_visible() {
  uint16_t visible = read_value16_(SI_REG_VISIBLE_DATA);
  // this->convert_data_(visible);
  //  visible = ((visible - 256) / 0.282) * 14.5;
  return visible;
}

uint16_t SI11xComponent::convert_data_(uint16_t data) {
  // msb * 256 + lsb
  uint8_t lsb = data & 0x00FF;
  uint8_t msb = (data & 0xFF00) >> 8;
  ESP_LOGD(TAG, "Conversion data:0x%04x (%d) msb:0x%02X lsb:0x%02X", data, data, msb, lsb);
  return (msb * 256) + lsb;
}

/**
 @brief Read I2C Data
 @param [in] register_addr register address
 @param [in] num Data Length
 @param [out] *buf Read Data
*/
uint16_t SI11xComponent::read_i2c_(uint8_t register_addr, uint8_t num, uint8_t *buf) { return (uint16_t) 0; }

/*****************************************************************************
 * @brief
 *   Structure Definition for calref array
 ******************************************************************************/
struct CalRefT {
  uint32_t sirpd_adchi_irled; /**< Small IR PD gain, IR LED, Hi ADC Range     */
  uint32_t sirpd_adclo_irled; /**< Small IR PD gain, IR LED, Lo ADC Range     */
  uint32_t sirpd_adclo_whled; /**< Small IR PD gain, White LED, Lo ADC Range  */
  uint32_t vispd_adchi_whled; /**< VIS PD gain, White LED, Lo ADC Range       */
  uint32_t vispd_adclo_whled; /**< VIS PD gain, White LED, Lo ADC Range       */
  uint32_t lirpd_adchi_irled; /**< Large IR PD gain, IR LED, Hi ADC Range     */
  uint32_t ledi_65ma;         /**< LED Current Ratio at 65 mA                 */
  uint8_t ucoef[4];           /**< UV Coefficient Storage                     */
};

/*****************************************************************************
 * @brief
 *   Factory Calibration Reference Values
 ******************************************************************************/
struct CalRefT calref[2] = {{
                                FLT_TO_FX20(4.021290),    // sirpd_adchi_irled
                                FLT_TO_FX20(57.528500),   // sirpd_adclo_irled
                                FLT_TO_FX20(2.690010),    // sirpd_adclo_whled
                                FLT_TO_FX20(0.042903),    // vispd_adchi_whled
                                FLT_TO_FX20(0.633435),    // vispd_adclo_whled
                                FLT_TO_FX20(23.902900),   // lirpd_adchi_irled
                                FLT_TO_FX20(56.889300),   // ledi_65ma
                                {0x7B, 0x6B, 0x01, 0x00}  // default ucoef
                            },
                            {
                                FLT_TO_FX20(2.325484),    // sirpd_adchi_irled
                                FLT_TO_FX20(33.541500),   // sirpd_adclo_irled
                                FLT_TO_FX20(1.693750),    // sirpd_adclo_whled
                                FLT_TO_FX20(0.026775),    // vispd_adchi_whled
                                FLT_TO_FX20(0.398443),    // vispd_adclo_whled
                                FLT_TO_FX20(12.190900),   // lirpd_adchi_irled
                                FLT_TO_FX20(56.558200),   // ledi_65ma
                                {0xdb, 0x8f, 0x01, 0x00}  // default ucoef
                            }};
/*****************************************************************************
 * @brief
 *   The fx20_divide and fx20_multiply uses this structure to pass
 *   values into it.
 ******************************************************************************/
struct operand_t {
  uint32_t op1; /**< Operand 1 */
  uint32_t op2; /**< Operand 2 */
};
/*****************************************************************************
 * @brief
 *   Converts the 12-bit factory test value from the Si114x and returns the
 *   fixed-point representation of this 12-bit factory test value.
 ******************************************************************************/
uint32_t SI11xComponent::decode_(uint32_t input) {
  int32_t exponent, exponent_bias9;
  uint32_t mantissa;

  if (input == 0)
    return 0.0;

  exponent_bias9 = (input & 0x0f00) >> 8;
  exponent = exponent_bias9 - 9;

  mantissa = input & 0x00ff;  // fraction
  mantissa |= 0x0100;         // add in integer

  // representation in 12 bit integer, 20 bit fraction
  mantissa = mantissa << (12 + exponent);
  return mantissa;
}
/*****************************************************************************
 * @brief
 *   This performs a shift_left function. For convenience, a negative
 *   shift value will shift the value right. Value pointed will be
 *   overwritten.
 ******************************************************************************/
void SI11xComponent::shift_left_(uint32_t *value_p, int8_t shift) {
  if (shift > 0) {
    *value_p = *value_p << shift;
  } else {
    *value_p = *value_p >> (-shift);
  }
}

/*****************************************************************************
 * @brief
 *   The buffer[] is assumed to point to a byte array that containst the
 *   factory calibration values after writing 0x12 to the command register
 *   This function takes the 12 bytes from the Si114x, then converts it
 *   to a fixed point representation, with the help of the decode() function
 ******************************************************************************/
uint32_t SI11xComponent::collect_(const uint8_t *buffer, uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment) {
  uint16_t value;
  uint8_t msb_ind = msb_addr - 0x22;
  uint8_t lsb_ind = lsb_addr - 0x22;

  if (alignment == 0) {
    value = buffer[msb_ind] << 4;
    value += buffer[lsb_ind] >> 4;
  } else {
    value = buffer[msb_ind] << 8;
    value += buffer[lsb_ind];
    value &= 0x0fff;
  }

  if ((value == 0x0fff) || (value == 0x0000))
    return FX20_BAD_VALUE;
  return decode_(value);
}

/*****************************************************************************
 * @brief
 *   Returns a fixed-point (20-bit fraction) after dividing op1/op2
 ******************************************************************************/
uint32_t SI11xComponent::fx20_divide_(struct operand_t *operand_p) {
  int8_t numerator_sh = 0, denominator_sh = 0;
  uint32_t result;
  uint32_t *numerator_p;
  uint32_t *denominator_p;

  if (operand_p == nullptr)
    return FX20_BAD_VALUE;

  numerator_p = &operand_p->op1;
  denominator_p = &operand_p->op2;

  if ((*numerator_p == FX20_BAD_VALUE) || (*denominator_p == FX20_BAD_VALUE) || (*denominator_p == 0))
    return FX20_BAD_VALUE;

  fx20_round_(numerator_p);
  fx20_round_(denominator_p);
  numerator_sh = align_(numerator_p, ALIGN_LEFT);
  denominator_sh = align_(denominator_p, ALIGN_RIGHT);

  result = *numerator_p / ((uint16_t) (*denominator_p));
  shift_left_(&result, 20 - numerator_sh - denominator_sh);

  return result;
}

/*****************************************************************************
 * @brief
 *   Returns a fixed-point (20-bit fraction) after multiplying op1*op2
 ******************************************************************************/
uint32_t SI11xComponent::fx20_multiply_(struct operand_t *operand_p) {
  uint32_t result;
  int8_t val1_sh, val2_sh;
  uint32_t *val1_p;
  uint32_t *val2_p;

  if (operand_p == nullptr)
    return FX20_BAD_VALUE;

  val1_p = &(operand_p->op1);
  val2_p = &(operand_p->op2);

  fx20_round_(val1_p);
  fx20_round_(val2_p);

  val1_sh = align_(val1_p, ALIGN_RIGHT);
  val2_sh = align_(val2_p, ALIGN_RIGHT);

  result = (uint32_t) (((uint32_t) (*val1_p)) * ((uint32_t) (*val2_p)));
  shift_left_(&result, -20 + val1_sh + val2_sh);

  return result;
}

/*****************************************************************************
 * @brief
 *   Rounds the uint32_t value pointed by value_p to 16 bits.
 ******************************************************************************/
void SI11xComponent::fx20_round_(uint32_t *value_p) {
  int8_t shift;
  uint32_t mask1 = 0xffff8000;
  uint32_t mask2 = 0xffff0000;
  uint32_t lsb = 0x00008000;

  shift = align_(value_p, ALIGN_LEFT);
  if (((*value_p) & mask1) == mask1) {
    *value_p = 0x80000000;
    shift -= 1;
  } else {
    *value_p += lsb;
    *value_p &= mask2;
  }

  shift_left_(value_p, -shift);
}

/*****************************************************************************
 * @brief
 *   Aligns the value pointed by value_p to either the LEFT or RIGHT
 *   the number of shifted bits is returned. The value in value_p is
 *   overwritten.
 ******************************************************************************/
int8_t SI11xComponent::align_(uint32_t *value_p, int8_t direction) {
  int8_t local_shift, shift;
  uint32_t mask;

  // Check invalid value_p and *value_p, return without shifting if bad.
  if (value_p == nullptr)
    return 0;
  if (*value_p == 0)
    return 0;

  // Make sure direction is valid
  switch (direction) {
    case ALIGN_LEFT:
      local_shift = 1;
      mask = 0x80000000L;
      break;

    case ALIGN_RIGHT:
      local_shift = -1;
      mask = 0x00000001L;
      break;

    default:
      // Invalid direction, return without shifting
      return 0;
  }

  shift = 0;
  while (true) {
    if (*value_p & mask)
      break;
    shift++;
    shift_left_(value_p, local_shift);
  }
  return shift;
}

/****************************************************************************
 * @brief
 *   Populates the SI114X_CAL_S structure
 * @details
 *   Performs some initial checking based on the security
 *   level. If the checks fail, the function returns without attempting
 *   to fetch calibration values. The reason for the checking is that the
 *   Si114x uses the same registers to store calibration values as used for
 *   storing proximity and ambient light measurements. Therefore, this function
 *   needs to be used only if there is no possibility of an autonomous process
 *   possibly overwriting the output registers.
 *
 *   If the checks are successful, then the si114x retrieves the compressed
 *   values from the Si114x, then populates the SI114X_CAL_S   structure whose
 *   pointer is passed to si114x_calibration()
 *
 *   If there are any errors, si114x_cal   is populated with default values
 *
 * @param[in] si114x_handle
 *   The programmer's toolkit handle
 * @param[out] si114x_cal
 *   Points to the SI114X_CAL_S structure that will hold the calibration values
 *   from the Si114x. If there are any errors, si114x_cal is populated with
 *   default values.
 * @param[in] security
 * >        0            Minimal checking
 * >
 * >        1            Checks to make sure that interface registers
 * >                     are zero, otherwise, returns -1
 * >                     interface registers are zero only when the Si114x
 * >                     has been reset, and no autonomous measurements have
 * >                     started.
 * @retval   0
 *  Success
 * @retval  -1
 *   Security Check failed
 * @retval  -2
 *   i2c communication error
 * @retval  -3
 *   Chip does not support factory calibration
 * @retval  -4
 *   Null pointers found for si114x_handle or si114x_cal
 ******************************************************************************/
/*
 * Side-effects:
 *     - Writes 0x11 to command reg to retrieve factory calibration values in
 *       buffer[0] to buffer[11]
 *
 *     - Calls the various helper functions such as vispd_correction()
 *       irpd_correction, to populate the SI114X_CAL_S structure
 *
 *     - Writes 0x12 to command reg to retrieve factory cal_index to
 *       buffer[12] to buffer[13]
 ******************************************************************************/
int16_t SI11xComponent::si114x_get_calibration_(SI114X_CAL_S *si114x_cal, uint8_t security) {
  uint8_t buffer[14];
  int16_t retval = 0;
  uint8_t response;

  if (si114x_cal == nullptr) {
    retval = -4;
    goto error_exit;
  }

  // if requested, check to make sure the interface registers are zero
  // as an indication of a device that has not started any autonomous
  // operation
  if (security == 1) {
    int8_t i;

    retval = read_i2c_(SI_REG_VISIBLE_DATA, 12, buffer);
    // retval = Si114xBlockRead( si114x_handle, REG_ALS_VIS_DATA0, 12, buffer );
    if (retval != 0) {
      retval = -2;
      goto error_exit;
    }

    for (i = 0; i < 12; i++) {
      if (buffer[i] != 0) {
        retval = -1;
        goto error_exit;
      }
    }
  }

  // Request for the calibration data
  if (!this->send_command_(0x12)) {
    retval = -2;
    goto error_exit;
  }

  // Wait for the response register to increment
  do {
    response = this->read_value_(SI_REG_RESPONSE);
    // response = Si114xReadFromRegister( si114x_handle, REG_RESPONSE );
    //  If the upper nibbles are non-zero, something is wrong
    if (response == 0x80) {
      // calibration code has not been implemented on this device
      // leading to command error. So, rather than returning an
      // error, handle the error by Nop and set ratios to -1.0
      // and return normally.
      this->set_value_(SI_REG_COMMAND, SI_NOP);
      // Si114xNop( si114x_handle );
      retval = -3;
      goto error_exit;
    } else if (response & 0xfff0) {
      // if upper nibble is anything but 0x80, exit with an error
      retval = -2;
      goto error_exit;
    }
    if (response != 1) {
      delay(1);
    }
  } while (response != 1);

  // Retrieve the 12 bytes from the interface registers
  retval = read_i2c_(SI_REG_VISIBLE_DATA, 12, buffer);
  if (retval != 0) {
    retval = -2;
    goto error_exit;
  }

  retval = si114x_get_cal_index_(buffer);

  if (retval != 0) {
    retval = -2;
    goto error_exit;
  }

  si114x_cal->ledi_ratio = ledi_ratio_(buffer);
  si114x_cal->vispd_correction = vispd_correction_(buffer);
  si114x_cal->irpd_correction = irpd_correction_(buffer);
  si114x_cal->adcrange_ratio = adcrange_ratio_(buffer);
  si114x_cal->ucoef_p = calref[find_cal_index_(buffer)].ucoef;
  si114x_cal->irsize_ratio = irsize_ratio_(buffer);
  return 0;

error_exit:
  si114x_cal->ledi_ratio = FX20_ONE;
  si114x_cal->vispd_correction = FX20_ONE;
  si114x_cal->irpd_correction = FX20_ONE;
  si114x_cal->adcrange_ratio = FLT_TO_FX20(14.5);
  si114x_cal->ucoef_p = nullptr;
  si114x_cal->irsize_ratio = FLT_TO_FX20(6.0);
  return retval;
}

/*****************************************************************************
 * @brief
 *   Initializes the Si113x/46/47/48 UCOEF Registers.
 *
 * @details
 *   Takes in an optional input ucoef pointer, then modifies
 *   it based on calibration. If the input ucoef is NULL, default values for
 *   clear overlay is assumed and then the si114x_cal is used to adjust it
 *   before setting the ucoef. Note that the Si114x ucoef registers are
 *   updated by this function; no additional action is required. This routine
 *   also performs the necessary querying of the chip identification to make
 *   sure that it is a uvindex-capable device.
 * @param[in] input_ucoef
 *   if NULL, a clear overlay is assumed, and datasheet values for ucoef is
 *   used. Otherwise, pointer to 4 bytes array representing the reference
 *   coefficients is passed.
 * @param[in] si114x_cal
 *   Points to the SI114X_CAL_S structure that holds the calibration values
 *   from the Si113x/4x
 * @retval   0
 *   Success
 * @retval  -1
 *   The device is neither Si1132, Si1145, Si1146 nor Si1147
 * @retval  <-1
 *   Error
 ******************************************************************************/
int16_t SI11xComponent::si114x_set_ucoef_(uint8_t *input_ucoef, SI114X_CAL_S *si114x_cal) {
  int8_t response;
  uint8_t temp;
  uint32_t vc = FX20_ONE, ic = FX20_ONE, long_temp;
  struct operand_t op;
  uint8_t *ref_ucoef = si114x_cal->ucoef_p;
  // uint8_t          out_ucoef[4];

  if (input_ucoef != nullptr)
    ref_ucoef = input_ucoef;

  if (ref_ucoef == nullptr)
    return -1;

  // retrieve part identification
  response = this->read_value_(SI_REG_PARTID);
  // response = Si114xReadFromRegister( si114x_handle, REG_PART_ID );
  switch (response) {
    case 0x32:
    case 0x45:
    case 0x46:
    case 0x47:
      temp = 1;
      break;
    default:
      temp = 0;
      break;
  }
  if (!temp)
    return -1;

  if (si114x_cal != nullptr) {
    if (si114x_cal->vispd_correction > 0)
      vc = si114x_cal->vispd_correction;
    if (si114x_cal->irpd_correction > 0)
      ic = si114x_cal->irpd_correction;
  }

  op.op1 = ref_ucoef[0] + ((ref_ucoef[1]) << 8);
  op.op2 = vc;
  long_temp = fx20_multiply_(&op);
  coefficients_[0] = (long_temp & 0x00ff);
  coefficients_[1] = (long_temp & 0xff00) >> 8;

  op.op1 = ref_ucoef[2] + (ref_ucoef[3] << 8);
  op.op2 = ic;
  long_temp = fx20_multiply_(&op);
  coefficients_[2] = (long_temp & 0x00ff);
  coefficients_[3] = (long_temp & 0xff00) >> 8;

  this->set_value_(SI_REG_UCOEFF0, coefficients_[0]);
  this->set_value_(SI_REG_UCOEFF1, coefficients_[1]);
  this->set_value_(SI_REG_UCOEFF2, coefficients_[2]);
  response = this->set_value_(SI_REG_UCOEFF3, coefficients_[3]);
  // response = Si114xBlockWrite( si114x_handle, REG_UCOEF0 , 4, out_ucoef);

  return response;
}

/*****************************************************************************
 * @brief
 *   This is a helper function called from si114x_get_calibration_()
 *   Writes 0x11 to the Command Register, then populates buffer[12]
 *   and buffer[13] with the factory calibration index
 ******************************************************************************/
int16_t SI11xComponent::si114x_get_cal_index_(uint8_t *buf) {
  int16_t retval;

  if (buf == nullptr)
    return -1;

  // Retrieve the index
  if (!this->send_command_(0x11))
    return -1;

  retval = read_i2c_(REG_PS1_DATA0, 2, &(buf[12]));
  // retval = Si114xBlockRead( si114x_handle, REG_PS1_DATA0, 2, &(buffer[12]) );
  if (retval != 0)
    return -1;

  return 0;
}

/*****************************************************************************
 * @brief
 *   Waits until the Si113x/4x is sleeping before proceeding
 *   Bit 0 - sleep
 *   Bit 1 - suspended
 *   Bit 2 - running
 ******************************************************************************/
void SI11xComponent::wait_until_sleep_() {
  uint8_t response = 0;
  uint8_t count = 0;
  // This loops until the Si114x is known to be in its sleep state
  // or if an i2c error occurs
  while (count < LOOP_TIMEOUT_MS) {
    response = this->read_value_(REG_CHIP_STAT);
    if (response == 1)  // Sleep mode
      break;
    count++;
    delay(1);
  }
}

/*****************************************************************************
 * @brief
 *   Returns the ratio to adjust for differences in IRLED drive strength. Note
 *   that this does not help with LED irradiance variation.
 ******************************************************************************/
uint32_t SI11xComponent::ledi_ratio_(uint8_t *buffer) {
  struct operand_t op;
  uint32_t result;
  int16_t index;

  index = find_cal_index_(buffer);

  if (index < 0)
    result = FX20_ONE;

  // op.op1 = LED_DRV65_REF; op.op2 = LED_DRV65;
  op.op1 = calref[index].ledi_65ma;
  op.op2 = LED_DRV65;
  result = fx20_divide_(&op);

  if (result == FX20_BAD_VALUE)
    result = FX20_ONE;

  return result;
}

/*****************************************************************************
 * @brief
 *   Due to small differences in factory test setup, the reference calibration
 *   values may have slight variation. This function retrieves the calibration
 *   index stored in the Si114x so that it is possible to know which calibration
 *   reference values to use.
 ******************************************************************************/
int16_t SI11xComponent::find_cal_index_(const uint8_t *buffer) {
  int16_t index;
  uint8_t size;

  // buffer[12] is the LSB, buffer[13] is the MSB
  index = (int16_t) (buffer[12] + ((uint16_t) (buffer[13]) << 8));

  switch (index) {
    case -1:
      index = 0;
      break;
    case -2:
      index = 0;
      break;
    case -3:
      index = 1;
      break;
    default:
      index = -(4 + index);
  }

  size = sizeof(calref) / sizeof(calref[0]);

  if (index < size)
    return index;

  return -1;
}
/*****************************************************************************
 * @brief
 *   Returns the calibration ratio to be applied to VIS measurements
 ******************************************************************************/
uint32_t SI11xComponent::vispd_correction_(uint8_t *buffer) {
  struct operand_t op;
  uint32_t result;
  int16_t index = find_cal_index_(buffer);

  if (index < 0)
    result = FX20_ONE;

  op.op1 = calref[index].vispd_adclo_whled;
  op.op2 = VISPD_ADCLO_WHLED;
  result = fx20_divide_(&op);

  if (result == FX20_BAD_VALUE)
    result = FX20_ONE;

  return result;
}
/*****************************************************************************
 * @brief
 *   Returns the calibration ratio to be applied to IR measurements
 ******************************************************************************/
uint32_t SI11xComponent::irpd_correction_(uint8_t *buffer) {
  struct operand_t op;
  uint32_t result;
  int16_t index = find_cal_index_(buffer);

  if (index < 0)
    result = FX20_ONE;

  // op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
  op.op1 = calref[index].sirpd_adclo_irled;
  op.op2 = SIRPD_ADCLO_IRLED;
  result = fx20_divide_(&op);

  if (result == FX20_BAD_VALUE)
    result = FX20_ONE;

  return result;
}
/*****************************************************************************
 * @brief
 *   Returns the ratio to correlate between x_RANGE=0 and x_RANGE=1
 *   It is typically 14.5, but may have some slight component-to-component
 *   differences.
 ******************************************************************************/
uint32_t SI11xComponent::adcrange_ratio_(uint8_t *buffer) {
  struct operand_t op;
  uint32_t result;

  op.op1 = SIRPD_ADCLO_IRLED;
  op.op2 = SIRPD_ADCHI_IRLED;
  result = fx20_divide_(&op);

  if (result == FX20_BAD_VALUE)
    result = FLT_TO_FX20(14.5);

  return result;
}
/***************************************************************************
 * @brief
 *   Returns the ratio to correlate between measurements made from large PD
 *   to measurements made from small PD.
 ******************************************************************************/
uint32_t SI11xComponent::irsize_ratio_(uint8_t *buffer) {
  struct operand_t op;
  uint32_t result;

  op.op1 = LIRPD_ADCHI_IRLED;
  op.op2 = SIRPD_ADCHI_IRLED;

  result = fx20_divide_(&op);

  if (result == FX20_BAD_VALUE)
    result = FLT_TO_FX20(6.0);

  return result;
}

}  // namespace si11xx
}  // namespace esphome
