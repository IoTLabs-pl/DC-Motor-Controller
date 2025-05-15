#pragma once

#include <array>
#include <limits>

#include "ruckig/ruckig.hpp"

namespace dc_servo
{
    namespace planner
    {
        template <int N, typename T = float>
        struct MotionParameters
        {
            std::array<T, N> max_speed;
            std::array<T, N> max_acceleration;
            std::array<T, N> max_jerk;

            MotionParameters(T max_speed,
                             T max_acceleration,
                             T max_jerk)
            {
                this->max_speed.fill(max_speed);
                if (max_acceleration)
                    this->max_acceleration.fill(max_acceleration);
                if (max_jerk)
                    this->max_jerk.fill(max_jerk);
            }

            void pass_to_input(ruckig::InputParameter<N> &input) const
            {
                input.max_velocity = std::move(this->max_speed);
                if (this->max_acceleration[0])
                    input.max_acceleration = std::move(this->max_acceleration);
                if (this->max_jerk[0])
                    input.max_jerk = std::move(this->max_jerk);
            }
        };

        template <int N, typename T = float>
        class Planner
        {
        public:
            Planner(T sample_time,
                    MotionParameters<N, T> motion_parameters) : ruckig_{sample_time}
            {
                motion_parameters.pass_to_input(this->input_);
            }

            void set_target_position(std::array<T, N> target_position)
            {
                this->input_.target_position = target_position;
            }

            void set_target_speed(std::array<T, N> target_speed)
            {
                this->input_.target_velocity = target_speed;
            }

            void set_current_position(std::array<T, N> current_position)
            {
                this->input_.current_position = current_position;
            }

            void set_current_speed(std::array<T, N> current_speed)
            {
                this->input_.current_velocity = current_speed;
            }

            void set_position_tracking(bool position_tracking = true)
            {
                this->input_.control_interface = position_tracking ? ruckig::ControlInterface::Position : ruckig::ControlInterface::Velocity;
            }

            ruckig::Result update()
            {
                auto r = this->ruckig_.update(this->input_, this->output_);

                this->output_.pass_to_input(this->input_);

                return r;
            }

            std::array<T, N> get_calculated_position() const { return this->output_.new_position; }
            std::array<T, N> get_calculated_speed() const { return this->output_.new_velocity; }

            std::array<T, N> get_current_position() const { return this->input_.current_position; }
            std::array<T, N> get_target_position() const { return this->input_.target_position; }

            std::array<T, N> get_max_speed() const { return this->input_.max_velocity; }

        protected:
            ruckig::InputParameter<N> input_;
            ruckig::OutputParameter<N> output_;
            ruckig::Ruckig<N> ruckig_;
        };
    }
}