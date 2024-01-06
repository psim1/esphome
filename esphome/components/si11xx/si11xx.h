#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace si11xx {



// INCLUDE_SI114X_CALIBRATIONCODE
#define FLT_TO_FX20(x) ((int32_t) ((x * 1048576) + .5))
#define FX20_ONE FLT_TO_FX20(1.000000)
#define FX20_BAD_VALUE 0xffffffff

//                                            msb   lsb   align
//                                            i2c   i2c   ment
//                                            addr  addr
#define SIRPD_ADCHI_IRLED (collect(buffer, 0x23, 0x22, 0))
#define SIRPD_ADCLO_IRLED (collect(buffer, 0x22, 0x25, 1))
#define SIRPD_ADCLO_WHLED (collect(buffer, 0x24, 0x26, 0))
#define VISPD_ADCHI_WHLED (collect(buffer, 0x26, 0x27, 1))
#define VISPD_ADCLO_WHLED (collect(buffer, 0x28, 0x29, 0))
#define LIRPD_ADCHI_IRLED (collect(buffer, 0x29, 0x2a, 1))
#define LED_DRV65 (collect(buffer, 0x2b, 0x2c, 0))



typedef struct {
  uint32_t vispd_correction; /**< VIS Photodiode Correction        */
  uint32_t irpd_correction;  /**< IR  Photodiode Correction        */
  uint32_t adcrange_ratio;   /**< _RANGE Ratio                     */
  uint32_t irsize_ratio;     /**< Large IR vs Small IR Ratio       */
  uint32_t ledi_ratio;       /**< LED Drive Current Correction     */
  uint8_t *ucoef_p;          /**< Pointer to UV Coefficients       */
} SI114X_CAL_S;

/// This class implements support for the ENS160 relative humidity and temperature i2c sensor.
class SI11xComponent : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  uint16_t readUV(void);
  float readUVIndex(void);
  char *readUVScore(float);

  uint16_t readProximity(void);

  uint16_t readIR(void);
  uint16_t readVisible(void);

  uint16_t readDevice(void);

  void loadCoEfficients(void);
  // control behaviour of sensor
  bool ProximityLedAttached = false;
  bool OutsideMode = true;
  uint8_t CoEfficientValues[4];

 private:
  uint8_t _i2caddr;
  uint8_t _device_type_{0};
  bool proximity_supported_{0};
  uint8_t _coefficients[4];

  void get_device_();
  void reset_();
  uint8_t read_value_(uint8_t /*reg*/);
  uint16_t read_value16_(uint8_t /*reg*/);
  bool set_value_(uint8_t /*reg*/, uint8_t /*mode*/);
  void write_param_(uint8_t /*register_addr*/, uint8_t /*value*/);

  bool configuration_1132_();
  bool configuration_1145_();
  void set_ambient_light_params_();
  void setInfraRedParams(void);
  void setProximityParams(void);
  uint16_t writeI2c(uint8_t register_addr, uint8_t value);
  uint16_t readI2c(uint8_t register_addr, uint8_t num, uint8_t *buffer);
  uint8_t readI2c_8(uint8_t register_addr);
  uint16_t readI2c_16(uint8_t register_addr);
  void set_calibrated_coefficients_();
  int16_t si114x_get_calibration(SI114X_CAL_S *si114x_cal, uint8_t security);
  int16_t si114x_set_ucoef(uint8_t *input_ucoef, SI114X_CAL_S *si114x_cal);
  int16_t si114x_get_cal_index(uint8_t *buffer);
  int16_t _waitUntilSleep(void);
  int16_t find_cal_index(uint8_t *buffer);
  uint32_t ledi_ratio(uint8_t *buffer);
  uint32_t vispd_correction(uint8_t *buffer);
  uint32_t irpd_correction(uint8_t *buffer);
  uint32_t adcrange_ratio(uint8_t *buffer);
  uint32_t irsize_ratio(uint8_t *buffer);
  int8_t align(uint32_t *value_p, int8_t direction);
  void fx20_round(uint32_t *value_p);
  uint32_t fx20_multiply(struct operand_t *operand_p);
  uint32_t fx20_divide(struct operand_t *operand_p);
  uint32_t collect(uint8_t *buffer, uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment);
  void shift_left(uint32_t *value_p, int8_t shift);
  uint32_t decode(uint32_t input);

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    INVALID_ID,
    VALIDITY_INVALID,
    READ_FAILED,
    WRITE_FAILED,
    STD_OPMODE_FAILED,
  } error_code_{NONE};
};

}  // namespace si11x
}  // namespace esphome
