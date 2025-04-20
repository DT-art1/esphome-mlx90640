import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import i2c, sensor,  select, binary_sensor, number
from esphome.cpp_helpers import setup_entity
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PRESENCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    ENTITY_CATEGORY_CONFIG
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["camera", "sensor", "select", "binary_sensor", "number"]

# namespace in c++
mlx90640_ns = cg.esphome_ns.namespace("mlx90640")
# classname in c++
MLX90640Component = mlx90640_ns.class_("MLX90640", cg.Component, i2c.I2CDevice)
SelectRefreshRate = mlx90640_ns.class_("SelectRefreshRate")
SelectResolution = mlx90640_ns.class_("SelectResolution")
NumberPresenceDetectorThreshold = mlx90640_ns.class_("NumberPresenceDetectorThreshold")
NumberPresenceDetectorHysteresis = mlx90640_ns.class_("NumberPresenceDetectorHysteresis")

CONF_AMBIENT = "ambient"
CONF_MINIMUM = "minimum"
CONF_MAXIMUM = "maximum"

RefreshRate = mlx90640_ns.enum("RefreshRate")

CONF_REFRESH_RATE = "refresh_rate"
CONF_DEFAULT_REFRESH_RATE = "default_refresh_rate"
CONF_REFRESH_RATE_SELECTS = {
    "IR_REFRESH_RATE_0_DOT_5_HZ": RefreshRate.IR_REFRESH_RATE_0_DOT_5_HZ,
    "IR_REFRESH_RATE_1_HZ": RefreshRate.IR_REFRESH_RATE_1_HZ,
    "IR_REFRESH_RATE_2_HZ": RefreshRate.IR_REFRESH_RATE_2_HZ,
    "IR_REFRESH_RATE_4_HZ": RefreshRate.IR_REFRESH_RATE_4_HZ,
    "IR_REFRESH_RATE_8_HZ": RefreshRate.IR_REFRESH_RATE_8_HZ,
    "IR_REFRESH_RATE_16_HZ": RefreshRate.IR_REFRESH_RATE_16_HZ,
    "IR_REFRESH_RATE_32_HZ": RefreshRate.IR_REFRESH_RATE_32_HZ,
    "IR_REFRESH_RATE_64_HZ": RefreshRate.IR_REFRESH_RATE_64_HZ
}

Resolution = mlx90640_ns.enum("Resolution")

CONF_RESOLUTION = "resolution"
CONF_DEFAULT_RESOLUTION = "default_resolution"

CONF_RESOLUTION_SELECTS = {
    "RESOLUTION_16_BIT": Resolution.RESOLUTION_16_BIT,
    "RESOLUTION_17_BIT": Resolution.RESOLUTION_17_BIT,
    "RESOLUTION_18_BIT": Resolution.RESOLUTION_18_BIT,
    "RESOLUTION_19_BIT": Resolution.RESOLUTION_19_BIT,
}

CONF_PRESENCE_DETECTOR = "presence_detector"
CONF_PRESENCE_DETECTOR_THRESHOLD = "presence_detector_threshold"
CONF_PRESENCE_DETECTOR_HYSTERESIS = "presence_detector_hysteresis"

CONF_DEFAULT_PRESENCE_DETECTOR_THRESHOLD = "default_presence_detector_threshold"
CONF_DEFAULT_PRESENCE_DETECTOR_HYSTERESIS = "default_presence_detector_hysteresis"

CONFIG_SCHEMA = cv.ENTITY_BASE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MLX90640Component),
        cv.Optional(CONF_DEFAULT_RESOLUTION, default="RESOLUTION_16_BIT"): cv.enum(
            CONF_RESOLUTION_SELECTS, upper=True
        ),
        cv.Optional(CONF_DEFAULT_REFRESH_RATE, default="IR_REFRESH_RATE_2_HZ"): cv.enum(
            CONF_REFRESH_RATE_SELECTS, upper=True
        ),
        cv.Optional(CONF_DEFAULT_PRESENCE_DETECTOR_THRESHOLD, default=3.5): cv.float_range(0.1, 10.0),
        cv.Optional(CONF_DEFAULT_PRESENCE_DETECTOR_HYSTERESIS, default=0.2): cv.float_range(0.0, 5.0),
        cv.Optional(CONF_AMBIENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MINIMUM): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MAXIMUM): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REFRESH_RATE): select.select_schema(
            SelectRefreshRate,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_RESOLUTION): select.select_schema(
            SelectResolution,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
        cv.Optional(CONF_PRESENCE_DETECTOR): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PRESENCE,
        ),
        cv.Optional(CONF_PRESENCE_DETECTOR_THRESHOLD): number.number_schema(
            class_=NumberPresenceDetectorThreshold,
            device_class=DEVICE_CLASS_TEMPERATURE,
            unit_of_measurement=UNIT_CELSIUS,
        ),
        cv.Optional(CONF_PRESENCE_DETECTOR_HYSTERESIS): number.number_schema(
            class_=NumberPresenceDetectorHysteresis,
            device_class=DEVICE_CLASS_TEMPERATURE,
            unit_of_measurement=UNIT_CELSIUS,
        ),
    }
).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x33))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_default_resolution(config[CONF_DEFAULT_RESOLUTION]))
    cg.add(var.set_default_refresh_rate(config[CONF_DEFAULT_REFRESH_RATE]))
    cg.add(var.set_default_presence_detector_threshold(config[CONF_DEFAULT_PRESENCE_DETECTOR_THRESHOLD]))
    cg.add(var.set_default_presence_detector_hysteresis(config[CONF_DEFAULT_PRESENCE_DETECTOR_HYSTERESIS]))

    if CONF_AMBIENT in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT])
        cg.add(var.set_ambient_sensor(sens))
    if CONF_MINIMUM in config:
        sens = await sensor.new_sensor(config[CONF_MINIMUM])
        cg.add(var.set_minimum_temperature_sensor(sens))
    if CONF_MAXIMUM in config:
        sens = await sensor.new_sensor(config[CONF_MAXIMUM])
        cg.add(var.set_maximum_temperature_sensor(sens))
    if CONF_REFRESH_RATE in config:
        sel = await select.new_select(config.get(CONF_REFRESH_RATE), options=list(CONF_REFRESH_RATE_SELECTS.keys()))
        cg.add(var.set_refresh_rate_select(sel))
        cg.add(sel.set_parent(var))
    if CONF_RESOLUTION in config:
        sel = await select.new_select(config.get(CONF_RESOLUTION), options=list(CONF_RESOLUTION_SELECTS.keys()))
        cg.add(var.set_resolution_select(sel))
        cg.add(sel.set_parent(var))
    if CONF_PRESENCE_DETECTOR in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE_DETECTOR])
        cg.add(var.set_presence_detector_sensor(sens))
    if CONF_PRESENCE_DETECTOR_THRESHOLD in config:
        sens = await number.new_number(config[CONF_PRESENCE_DETECTOR_THRESHOLD], min_value=0.1, max_value=10.0, step=0.1)
        cg.add(var.set_presence_detector_threshold_sensor(sens))
        cg.add(sens.set_parent(var))
    if CONF_PRESENCE_DETECTOR_HYSTERESIS in config:
        sens = await number.new_number(config[CONF_PRESENCE_DETECTOR_HYSTERESIS], min_value=0.0, max_value=5.0, step=0.1)
        cg.add(var.set_presence_detector_hysteresis_sensor(sens))
        cg.add(sens.set_parent(var))
