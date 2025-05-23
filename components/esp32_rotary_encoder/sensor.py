import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    UNIT_STEPS,
    ICON_ROTATE_RIGHT,
    CONF_VALUE,
    CONF_PIN_A,
    CONF_PIN_B,
    CONF_RESTORE_MODE,
)

esp32_rotary_encoder_ns = cg.esphome_ns.namespace("esp32_rotary_encoder")

RotaryEncoderRestoreMode = esp32_rotary_encoder_ns.enum("RotaryEncoderRestoreMode")
RESTORE_MODES = {
    "RESTORE_DEFAULT_ZERO": RotaryEncoderRestoreMode.ROTARY_ENCODER_RESTORE_DEFAULT_ZERO,
    "ALWAYS_ZERO": RotaryEncoderRestoreMode.ROTARY_ENCODER_ALWAYS_ZERO,
}

CONF_PUBLISH_INITIAL_VALUE = "publish_initial_value"

RotaryEncoderSensor = esp32_rotary_encoder_ns.class_(
    "RotaryEncoderSensor", sensor.Sensor, cg.Component
)
RotaryEncoderSetValueAction = esp32_rotary_encoder_ns.class_(
    "RotaryEncoderSetValueAction", automation.Action
)
RotaryEncoderEnableAction = esp32_rotary_encoder_ns.class_(
    "RotaryEncoderEnableAction", automation.Action
)
RotaryEncoderDisableAction = esp32_rotary_encoder_ns.class_(
    "RotaryEncoderDisableAction", automation.Action
)


def validate_min_max_value(config):
    if CONF_MIN_VALUE in config and CONF_MAX_VALUE in config:
        min_val = config[CONF_MIN_VALUE]
        max_val = config[CONF_MAX_VALUE]
        if min_val >= max_val:
            raise cv.Invalid(
                f"Max value {max_val} must be smaller than min value {min_val}"
            )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        RotaryEncoderSensor,
        unit_of_measurement=UNIT_STEPS,
        icon=ICON_ROTATE_RIGHT,
        accuracy_decimals=0,
    )
    .extend(
        {
            cv.Required(CONF_PIN_A): cv.All(pins.internal_gpio_input_pin_schema),
            cv.Required(CONF_PIN_B): cv.All(pins.internal_gpio_input_pin_schema),
            cv.Optional(CONF_MIN_VALUE): cv.int_,
            cv.Optional(CONF_MAX_VALUE): cv.int_,
            cv.Optional(CONF_PUBLISH_INITIAL_VALUE, default=False): cv.boolean,
            cv.Optional(CONF_RESTORE_MODE, default="RESTORE_DEFAULT_ZERO"): cv.enum(
                RESTORE_MODES, upper=True, space="_"
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    validate_min_max_value,
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    pin_a = await cg.gpio_pin_expression(config[CONF_PIN_A])
    cg.add(var.set_pin_a(pin_a))
    pin_b = await cg.gpio_pin_expression(config[CONF_PIN_B])
    cg.add(var.set_pin_b(pin_b))
    cg.add(var.set_publish_initial_value(config[CONF_PUBLISH_INITIAL_VALUE]))
    cg.add(var.set_restore_mode(config[CONF_RESTORE_MODE]))

    if CONF_MIN_VALUE in config:
        cg.add(var.set_min_value(config[CONF_MIN_VALUE]))
    if CONF_MAX_VALUE in config:
        cg.add(var.set_max_value(config[CONF_MAX_VALUE]))


@automation.register_action(
    "sensor.esp32_rotary_encoder.set_value",
    RotaryEncoderSetValueAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(sensor.Sensor),
            cv.Required(CONF_VALUE): cv.templatable(cv.int_),
        }
    ),
)
async def sensor_template_publish_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_VALUE], args, int)
    cg.add(var.set_value(template_))
    return var


@automation.register_action(
    "sensor.esp32_rotary_encoder.enable",
    RotaryEncoderEnableAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(sensor.Sensor),
        }
    ),
)
async def sensor_enable_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var


@automation.register_action(
    "sensor.esp32_rotary_encoder.disable",
    RotaryEncoderDisableAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(sensor.Sensor),
        }
    ),
)
async def sensor_disable_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var
