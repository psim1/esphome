import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    #    CONF_GAIN,
    CONF_LIGHT,
    #    CONF_RESOLUTION,
    UNIT_LUX,
    ICON_BRIGHTNESS_5,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
)

CODEOWNERS = ["@psim1"]
DEPENDENCIES = ["i2c"]

si11xx_ns = cg.esphome_ns.namespace("si11xx")

SI11xComponent = si11xx_ns.class_("SI11xComponent", cg.PollingComponent, i2c.I2CDevice)

CONF_INFRA_RED = "infra_red"
CONF_UV_INDEX = "uv_index"
CONF_UV = "uv"
CONF_WINDOW_CORRECTION_FACTOR = "window_correction_factor"

UNIT_COUNTS = "#"
UNIT_UVI = "UVI"

# SI11XGAIN = si11xx_ns.enum("LTR390GAIN")
# GAIN_OPTIONS = {
#    "X1": SI11XGAIN.LTR390_GAIN_1,
#    "X3": SI11XGAIN.LTR390_GAIN_3,
#    "X6": SI11XGAIN.LTR390_GAIN_6,
#    "X9": SI11XGAIN.LTR390_GAIN_9,
#    "X18": SI11XGAIN.LTR390_GAIN_18,
# }

# SI11XRESOLUTION = si11xx_ns.enum("LTR390RESOLUTION")
# RES_OPTIONS = {
#    20: SI11XRESOLUTION.LTR390_RESOLUTION_20BIT,
#    19: SI11XRESOLUTION.LTR390_RESOLUTION_19BIT,
#    18: SI11XRESOLUTION.LTR390_RESOLUTION_18BIT,
#    17: SI11XRESOLUTION.LTR390_RESOLUTION_17BIT,
#    16: SI11XRESOLUTION.LTR390_RESOLUTION_16BIT,
#    13: SI11XRESOLUTION.LTR390_RESOLUTION_13BIT,
# }

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SI11xComponent),
            cv.Optional(CONF_LIGHT): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INFRA_RED): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
                unit_of_measurement=UNIT_UVI,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UV): sensor.sensor_schema(
                unit_of_measurement=UNIT_COUNTS,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            # cv.Optional(CONF_GAIN, default="X3"): cv.enum(GAIN_OPTIONS),
            # cv.Optional(CONF_RESOLUTION, default=18): cv.enum(RES_OPTIONS),
            cv.Optional(CONF_WINDOW_CORRECTION_FACTOR, default=1.0): cv.float_range(
                min=1.0
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x60)),
    cv.has_at_least_one_key(CONF_LIGHT, CONF_INFRA_RED, CONF_UV_INDEX, CONF_UV),
)

TYPES = {
    CONF_LIGHT: "set_light_sensor",
    CONF_INFRA_RED: "set_ir_sensor",
    CONF_UV_INDEX: "set_uvi_sensor",
    CONF_UV: "set_uv_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # cg.add(var.set_gain_value(config[CONF_GAIN]))
    # cg.add(var.set_res_value(config[CONF_RESOLUTION]))
    # cg.add(var.set_wfac_value(config[CONF_WINDOW_CORRECTION_FACTOR]))

    for key, funcName in TYPES.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, funcName)(sens))
