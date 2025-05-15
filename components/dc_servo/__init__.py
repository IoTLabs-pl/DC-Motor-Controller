from esphome import automation
import esphome.codegen as cg
from esphome.components import sensor, number
import esphome.config_validation as cv

from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_UPDATE_INTERVAL,
    CONF_POSITION,
    CONF_TRIGGER_ID,
)

from .const import (
    CONF_KD,
    CONF_KI,
    CONF_KP,
    CONF_POSITION_CONTROLLER,
    CONF_SPEED_CONTROLLER,
    CONF_SAMPLE_TIME_MULTIPLIER,
    CONF_SPEED_INPUT,
    CONF_MOVE_TIME,
    CONF_ACCELERATION_TIME,
    CONF_ACCELERATION_RISING_TIME,
    CONF_ON_DIRECTION_CHANGE,
    CONF_LOG_TAG
)

MULTI_CONF = True

dc_servo_ns = cg.global_ns.namespace("dc_servo")
PIDConfigStruct = dc_servo_ns.namespace("PID").struct("Config").template(cg.float_)
MotionPlannerParamsStruct = (
    dc_servo_ns.namespace("planner").struct("MotionParameters").template(1)
)

dc_servo_component_ns = cg.esphome_ns.namespace("dc_servo_component")
DCServoClass = dc_servo_component_ns.class_("DCServo", cg.PollingComponent)

DCServoGoToAction = dc_servo_component_ns.class_(
    "DCServoGoToAction", automation.WaitUntilAction
)
DCServoStopAction = dc_servo_component_ns.class_("DCServoStopAction", automation.Action)

DCServoIsMovingForwardCondition = dc_servo_component_ns.class_(
    "DCServoIsMovingForwardCondition", automation.Condition
)
DCServoIsMovingBackwardCondition = dc_servo_component_ns.class_(
    "DCServoIsMovingBackwardCondition", automation.Condition
)
DCServoIsStoppedCondition = dc_servo_component_ns.class_(
    "DCServoIsStoppedCondition", automation.Condition
)

DirectionEnum = dc_servo_component_ns.enum("Direction")
DCServoDirectionChangeTrigger = dc_servo_component_ns.class_(
    "DCServoDirectionChangeTrigger", automation.Trigger.template(DirectionEnum)
)


PID_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_KP): cv.positive_float,
        cv.Optional(CONF_KI, default=0): cv.positive_float,
        cv.Optional(CONF_KD, default=0): cv.positive_float,
    }
)

CONFIG_SCHEMA = cv.polling_component_schema("1s").extend(
    {
        cv.GenerateID(): cv.declare_id(DCServoClass),
        cv.Optional(CONF_LOG_TAG):cv.string_strict,
        cv.Required(CONF_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_SPEED_INPUT): cv.use_id(sensor.Sensor),
        cv.Required(CONF_OUTPUT): cv.use_id(number.Number),
        cv.Required(CONF_POSITION_CONTROLLER): PID_SCHEMA,
        cv.Required(CONF_SPEED_CONTROLLER): PID_SCHEMA,
        cv.Optional(CONF_SAMPLE_TIME_MULTIPLIER, default=1): cv.positive_not_null_int,
        cv.Required(CONF_MOVE_TIME): cv.positive_not_null_time_period,
        cv.Optional(CONF_ACCELERATION_TIME, default=0): cv.positive_time_period,
        cv.Optional(CONF_ACCELERATION_RISING_TIME, default=0): cv.positive_time_period,
        cv.Optional(CONF_ON_DIRECTION_CHANGE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DCServoDirectionChangeTrigger
                ),
            }
        ),
    }
)


def generate_pid_config(**kwargs):
    return cg.StructInitializer(PIDConfigStruct, *kwargs.items())


def calculate_motion_params(
    move_time: cv.TimePeriod,
    acceleration_time: cv.TimePeriod,
    acceleration_rising_time: cv.TimePeriod,
):
    # convert ms to s
    move_time = move_time.total_nanoseconds / 1e9
    acceleration_time = acceleration_time.total_nanoseconds / 1e9
    acceleration_rising_time = acceleration_rising_time.total_nanoseconds / 1e9

    t_jerk, t_c_acc, t_c_speed = (
        acceleration_rising_time,
        acceleration_time - 2 * acceleration_rising_time,
        move_time - 2 * acceleration_time,
    )

    t = 2 * t_jerk + t_c_acc + t_c_speed

    max_speed = 1 / t
    initializer_args = (("max_speed", max_speed),)

    if t_c_acc + t_jerk > 0:
        max_acceleration = max_speed / (t_jerk + t_c_acc)
        initializer_args += (("max_acceleration", max_acceleration),)

    if t_jerk > 0:
        max_jerk = max_acceleration / t_jerk
        initializer_args += (("max_jerk", max_jerk),)

    return cg.ArrayInitializer(*(e[1] for e in initializer_args))


async def to_code(config):
    cg.add_build_flag("-std=gnu++17")
    cg.add_platformio_option("build_unflags", "-std=gnu++11")
    cg.add_library("https://github.com/IoTLabs-pl/fruckig", "0.12.2")
    cg.add_global(dc_servo_component_ns.using)

    input_ = await cg.get_variable(config[CONF_INPUT])
    if CONF_SPEED_INPUT in config:
        speed_input = await cg.get_variable(config[CONF_SPEED_INPUT])
    else:
        speed_input = cg.nullptr

    output_ = await cg.get_variable(config[CONF_OUTPUT])

    args = (
        config[CONF_ID],
        config[CONF_UPDATE_INTERVAL],
        generate_pid_config(**config[CONF_POSITION_CONTROLLER]),
        input_,
        generate_pid_config(**config[CONF_SPEED_CONTROLLER]),
        speed_input,
        config[CONF_SAMPLE_TIME_MULTIPLIER],
        output_,
        calculate_motion_params(
            config[CONF_MOVE_TIME],
            config[CONF_ACCELERATION_TIME],
            config[CONF_ACCELERATION_RISING_TIME],
        ),
        str(config.get(CONF_LOG_TAG, config[CONF_ID])),
    )

    dc_servo = cg.new_Pvariable(*args)

    await cg.register_component(dc_servo, config)

    for conf in config.get(CONF_ON_DIRECTION_CHANGE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], dc_servo)
        await automation.build_automation(trigger, [(DirectionEnum, "direction")], conf)


ACTION_BASE_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(DCServoClass)})


@automation.register_action(
    "dc_servo.go_to",
    DCServoGoToAction,
    ACTION_BASE_SCHEMA.extend(
        {
            cv.Required(CONF_POSITION): cv.templatable(cv.zero_to_one_float),
        }
    ),
)
async def dc_servo_go_to_action_to_code(config, action_id, template_arg, args):
    servo = await cg.get_variable(config[CONF_ID])
    action_variable = cg.new_Pvariable(action_id, template_arg, servo)
    position_value_template = await cg.templatable(config[CONF_POSITION], args, float)
    cg.add(action_variable.set_position(position_value_template))
    await cg.register_component(action_variable, {})
    return action_variable


@automation.register_action(
    "dc_servo.stop",
    DCServoStopAction,
    automation.maybe_simple_id(ACTION_BASE_SCHEMA),
)
async def dc_servo_stop_action_to_code(config, action_id, template_arg, args):
    servo = await cg.get_variable(config[CONF_ID])
    action_variable = cg.new_Pvariable(action_id, template_arg, servo)
    return action_variable


@automation.register_condition(
    "dc_servo.is_moving_forward",
    DCServoIsMovingForwardCondition,
    automation.maybe_simple_id(ACTION_BASE_SCHEMA),
)
async def dc_servo_is_moving_forward_condition_to_code(
    config, condition_id, template_arg, args
):
    servo = await cg.get_variable(config[CONF_ID])
    condition_variable = cg.new_Pvariable(condition_id, template_arg, servo)
    return condition_variable


@automation.register_condition(
    "dc_servo.is_moving_backward",
    DCServoIsMovingBackwardCondition,
    automation.maybe_simple_id(ACTION_BASE_SCHEMA),
)
async def dc_servo_is_moving_backward_condition_to_code(
    config, condition_id, template_arg, args
):
    servo = await cg.get_variable(config[CONF_ID])
    condition_variable = cg.new_Pvariable(condition_id, template_arg, servo)
    return condition_variable


@automation.register_condition(
    "dc_servo.is_stopped",
    DCServoIsStoppedCondition,
    automation.maybe_simple_id(ACTION_BASE_SCHEMA),
)
async def dc_servo_is_stopped_condition_to_code(
    config, condition_id, template_arg, args
):
    servo = await cg.get_variable(config[CONF_ID])
    condition_variable = cg.new_Pvariable(condition_id, template_arg, servo)
    return condition_variable
