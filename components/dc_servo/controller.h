#pragma once

#include <algorithm>
#include <limits>
#include <valarray>

#include "ruckig/ruckig.hpp"

#include "pid.h"
#include "planner.h"

namespace dc_servo
{
    namespace controller
    {
        enum Status
        {
            DC_SERVO_CONTROLLER_STATUS_IDLE = 0,
            DC_SERVO_CONTROLLER_STATUS_RUNNING = 1,
            DC_SERVO_CONTROLLER_STATUS_COASTING = 2,
            DC_SERVO_CONTROLLER_STATUS_ERROR = -1
        };

        enum Direction
        {
            DC_SERVO_CONTROLLER_DIRECTION_FORWARD = 1,
            DC_SERVO_CONTROLLER_DIRECTION_STOP = 0,
            DC_SERVO_CONTROLLER_DIRECTION_BACKWARD = -1
        };

        bool is_moving(Status status);

        class Controller
        {

        public:
            Controller(float sample_time,
                       PID::Config<float> pos_pid,
                       PID::Config<float> speed_pid,
                       uint8_t control_interval_factor,
                       planner::MotionParameters<1> motion_parameters) : motion_planner_{sample_time * control_interval_factor, motion_parameters},
                                                                         coast_loops_{(uint8_t)(1000 / sample_time)},
                                                                         speed_controller_{sample_time, speed_pid},
                                                                         position_controller_{sample_time * control_interval_factor, pos_pid},
                                                                         inner_loops_{control_interval_factor}

            {
            }

            Status get_status() const;
            Direction get_direction() const;

            void set_current_position(float current_position);
            void set_target_position(float target_position);
            void set_target_speed(float target_speed);

            float update(float current_position, float current_speed);
            float get_calculated_position() const;
            float get_calculated_speed() const;
            float get_current_position() const;
            float get_target_position() const;

        private:
            planner::Planner<1> motion_planner_;
            const uint8_t coast_loops_;

            Direction direction_ = DC_SERVO_CONTROLLER_DIRECTION_STOP;
            Status status_ = DC_SERVO_CONTROLLER_STATUS_IDLE;
            void set_status(Status new_status);
            uint8_t current_state_loop_counter_ = 0;

            PID::PID<float> speed_controller_;
            PID::PID<float> position_controller_;

            const uint8_t inner_loops_;

            float control_value_ = 0.0f;
            float position_control_value_ = 0.0f;

            bool should_run_position_control_loop_();
            void position_control_loop_(float current_position);
            void speed_control_loop_(float current_speed);
        };
    };
};
