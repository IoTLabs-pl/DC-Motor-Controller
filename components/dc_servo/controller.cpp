#include "controller.h"

namespace dc_servo
{
    namespace controller
    {
        bool is_moving(Status status)
        {
            return status == DC_SERVO_CONTROLLER_STATUS_RUNNING || status == DC_SERVO_CONTROLLER_STATUS_COASTING;
        };

        Status Controller::get_status() const { return this->status_; }
        Direction Controller::get_direction() const
        {
            auto status = this->get_status();

            if (is_moving(status))
            {
                if (this->get_target_position() > this->get_current_position())
                    return DC_SERVO_CONTROLLER_DIRECTION_FORWARD;
                else
                    return DC_SERVO_CONTROLLER_DIRECTION_BACKWARD;
            }
            return DC_SERVO_CONTROLLER_DIRECTION_STOP;
        }

        void Controller::set_current_position(float current_position)
        {
            this->motion_planner_.set_current_position({current_position});
            this->position_controller_.reset();
            this->speed_controller_.reset();
        }

        void Controller::set_target_position(float target_position)
        {
            this->motion_planner_.set_target_position({target_position});
            this->motion_planner_.set_position_tracking(true);
            this->set_status(DC_SERVO_CONTROLLER_STATUS_RUNNING);
        }

        void Controller::set_target_speed(float target_speed)
        {
            this->motion_planner_.set_target_speed({target_speed});
            this->motion_planner_.set_position_tracking(false);
            this->set_status(DC_SERVO_CONTROLLER_STATUS_RUNNING);
        }

        float Controller::update(float current_position, float current_speed)
        {
            auto status = this->get_status();

            if (is_moving(status))
            {
                if (status == Status::DC_SERVO_CONTROLLER_STATUS_COASTING && current_state_loop_counter_ > this->coast_loops_)
                {
                    this->set_status(DC_SERVO_CONTROLLER_STATUS_IDLE);
                    this->motion_planner_.set_current_speed({0});
                }
                else
                {
                    if (this->should_run_position_control_loop_())
                        this->position_control_loop_(current_position);

                    this->speed_control_loop_(current_speed);
                }
            }

            this->current_state_loop_counter_ += 1;

            return this->control_value_;
        }

        float Controller::get_calculated_position() const { return this->motion_planner_.get_calculated_position()[0]; }
        float Controller::get_calculated_speed() const { return this->motion_planner_.get_calculated_speed()[0]; }
        float Controller::get_current_position() const { return this->motion_planner_.get_current_position()[0]; }
        float Controller::get_target_position() const { return this->motion_planner_.get_target_position()[0]; }

        void Controller::set_status(Status new_status)
        {
            auto old_status = this->get_status();

            if (old_status == new_status)
                return;

            this->status_ = new_status;
            this->current_state_loop_counter_ = 0;
        }

        bool Controller::should_run_position_control_loop_()
        {
            return current_state_loop_counter_ % this->inner_loops_ == 0;
        }

        void Controller::position_control_loop_(float current_position)
        {
            auto motion_planner_status = this->motion_planner_.update();

            switch (motion_planner_status)
            {
            case ruckig::Result::Working:
                this->set_status(DC_SERVO_CONTROLLER_STATUS_RUNNING);
                break;
            case ruckig::Result::Finished:
                this->set_status(DC_SERVO_CONTROLLER_STATUS_COASTING);
                break;
            default:
                this->set_status(DC_SERVO_CONTROLLER_STATUS_ERROR);
                break;
            }

            this->position_control_value_ = this->position_controller_.update(this->motion_planner_.get_calculated_position()[0], current_position);
        }

        void Controller::speed_control_loop_(float current_speed)
        {
            auto setpoint_speed = this->motion_planner_.get_calculated_speed()[0] + this->position_control_value_;

            this->control_value_ = this->speed_controller_.update(setpoint_speed, current_speed);
        }
    }
};
