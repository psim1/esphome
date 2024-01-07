import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    #    CONF_GAIN,
    CONF_LIGHT,
    ICON_GAUGE,
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
CONF_OUTSIDE = "outside_mode"
CONF_PROXIMITY = "enable_proximity"

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
                unit_of_measurement=UNIT_LUX,
                icon=ICON_BRIGHTNESS_5,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UV_INDEX): sensor.sensor_schema(
                unit_of_measurement=UNIT_UVI,
                icon=ICON_GAUGE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OUTSIDE, default=False): cv.boolean,
            cv.Optional(CONF_PROXIMITY, default=False): cv.boolean,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x60)),
    cv.has_at_least_one_key(CONF_LIGHT, CONF_INFRA_RED, CONF_UV_INDEX),
)

TYPES = {
    CONF_LIGHT: "set_light_sensor",
    CONF_INFRA_RED: "set_ir_sensor",
    CONF_UV_INDEX: "set_uvi_sensor",
}


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_outside_mode(config[CONF_OUTSIDE]))
    cg.add(var.set_proximity_mode(config[CONF_PROXIMITY]))

    for key, funcName in TYPES.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, funcName)(sens))
