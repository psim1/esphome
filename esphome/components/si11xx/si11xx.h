#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace si11xx {

// INCLUDE_SI114X_CALIBRATIONCODE
#define FLT_TO_FX20(x) (((int32_t) (((x) *1048576) + .5)))
#define FX20_ONE FLT_TO_FX20(1.000000)
#define FX20_BAD_VALUE 0xffffffff

//                                          msb   lsb  align
//                                          i2c   i2c  ment
//                                          addr  addr
#define SIRPD_ADCHI_IRLED (collect_(buffer, 0x23, 0x22, 0))
#define SIRPD_ADCLO_IRLED (collect_(buffer, 0x22, 0x25, 1))
#define SIRPD_ADCLO_WHLED (collect_(buffer, 0x24, 0x26, 0))
#define VISPD_ADCHI_WHLED (collect_(buffer, 0x26, 0x27, 1))
#define VISPD_ADCLO_WHLED (collect_(buffer, 0x28, 0x29, 0))
#define LIRPD_ADCHI_IRLED (collect_(buffer, 0x29, 0x2a, 1))
#define LED_DRV65 (collect_(buffer, 0x2b, 0x2c, 0))

using SI114X_CAL_S = struct {
  uint32_t vispd_correction; /**< VIS Photodiode Correction        */
  uint32_t irpd_correction;  /**< IR  Photodiode Correction        */
  uint32_t adcrange_ratio;   /**< _RANGE Ratio                     */
  uint32_t irsize_ratio;     /**< Large IR vs Small IR Ratio       */
  uint32_t ledi_ratio;       /**< LED Drive Current Correction     */
  uint8_t *ucoef_p;          /**< Pointer to UV Coefficients       */
};

/// This class implements support for the ENS160 relative humidity and temperature i2c sensor.
class SI11xComponent : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  uint16_t read_uv();
  float read_uv_index();
  uint16_t read_proximity();
  uint16_t read_ir();
  uint16_t read_visible();

  void load_coefficients();
  // control behaviour of sensor
  bool proximity_led_attached_{false};
  bool outside_mode_{true};
  uint8_t CoEfficientValues[4];

  void set_light_sensor(sensor::Sensor *light_sensor) { this->light_sensor_ = light_sensor; }
  void set_ir_sensor(sensor::Sensor *ir_sensor) { this->ir_sensor_ = ir_sensor; }
  void set_uvi_sensor(sensor::Sensor *uvi_sensor) { this->uvi_sensor_ = uvi_sensor; }
  void set_uv_sensor(sensor::Sensor *uv_sensor) { this->uv_sensor_ = uv_sensor; }

 protected:
  sensor::Sensor *light_sensor_{nullptr};
  sensor::Sensor *ir_sensor_{nullptr};
  sensor::Sensor *uvi_sensor_{nullptr};
  sensor::Sensor *uv_sensor_{nullptr};

 private:
  uint8_t device_type_{0};
  uint8_t device_rev_{0};
  uint8_t device_seq_{0};
  bool proximity_supported_{false};
  uint8_t coefficients_[4];

  void get_device_type_();
  void reset_();
  uint8_t read_value_(uint8_t /*reg*/);
  uint16_t read_value16_(uint8_t /*reg*/);
  bool set_value_(uint8_t /*reg*/, uint8_t /*mode*/);
  void write_param_(uint8_t /*register_addr*/, uint8_t /*value*/);
  bool send_command_(uint8_t /*register_or_value*/);

  uint16_t convert_data_(uint16_t /*data*/);

  bool configuration_1132_();
  bool configuration_1145_();
  void set_measure_rate_();
  void read_config_();
  void set_ambient_light_params_();
  void set_infrared_params_();
  void set_proximity_params_();
  uint16_t read_i2c_(uint8_t register_addr, uint8_t num, uint8_t *buffer);
  void set_calibrated_coefficients_();
  int16_t si114x_get_calibration_(SI114X_CAL_S * /*si114x_cal*/, uint8_t /*security*/);
  int16_t si114x_set_ucoef_(uint8_t * /*input_ucoef*/, SI114X_CAL_S * /*si114x_cal*/);
  int16_t si114x_get_cal_index_(uint8_t * /*buffer*/);
  void wait_until_sleep_();
  int16_t find_cal_index_(const uint8_t * /*buffer*/);
  uint32_t ledi_ratio_(uint8_t * /*buffer*/);
  uint32_t vispd_correction_(uint8_t * /*buffer*/);
  uint32_t irpd_correction_(uint8_t * /*buffer*/);
  uint32_t adcrange_ratio_(uint8_t * /*buffer*/);
  uint32_t irsize_ratio_(uint8_t * /*buffer*/);
  int8_t align_(uint32_t * /*value_p*/, int8_t /*direction*/);
  void fx20_round_(uint32_t * /*value_p*/);
  uint32_t fx20_multiply_(struct operand_t * /*operand_p*/);
  uint32_t fx20_divide_(struct operand_t * /*operand_p*/);
  uint32_t collect_(const uint8_t * /*buffer*/, uint8_t /*msb_addr*/, uint8_t /*lsb_addr*/, uint8_t /*alignment*/);
  void shift_left_(uint32_t * /*value_p*/, int8_t /*shift*/);
  uint32_t decode_(uint32_t input);

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    INVALID_ID,
    VALIDITY_INVALID,
    READ_FAILED,
    WRITE_FAILED,
    COMMAND_FAILED,
  } error_code_{NONE};
};

}  // namespace si11xx
}  // namespace esphome
