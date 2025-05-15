#pragma once

#include <esphome/core/automation.h>
#include <esphome/core/component.h>
#include <esphome/core/hal.h>
#include <esphome/components/sensor/sensor.h>
#include <esphome/components/switch/switch.h>
#include <esphome/components/number/number.h>

#include "controller.h"
#include "pid.h"

static const char *TAG = "dc_servo";

namespace controller = ::dc_servo::controller;
namespace PID = ::dc_servo::PID;
namespace planner = ::dc_servo::planner;

namespace esphome
{
    namespace dc_servo_component
    {
        enum Direction
        {
            DC_SERVO_DIRECTION_FORWARD = 1,
            DC_SERVO_DIRECTION_STOP = 0,
            DC_SERVO_DIRECTION_BACKWARD = -1
        };

        class DCServo : public controller::Controller, public PollingComponent
        {
        public:
            DCServo(
                uint32_t sample_time,
                PID::Config<float> pos_pid,
                sensor::Sensor *position_input,
                PID::Config<float> speed_pid,
                sensor::Sensor *speed_input,
                uint8_t control_interval_factor,
                number::Number *level_output,
                planner::MotionParameters<1, float> motion_parameters,
                const char *sub_tag) : controller::Controller{
                                           sample_time / 1000.0f,
                                           {
                                               .kp = pos_pid.kp,
                                               .ki = pos_pid.ki,
                                               .kd = pos_pid.kd,
                                               .min_control = -motion_parameters.max_speed[0],
                                               .max_control = motion_parameters.max_speed[0],
                                           },
                                           {
                                               .kp = speed_pid.kp,
                                               .ki = speed_pid.ki,
                                               .kd = speed_pid.kd,
                                               .min_control = level_output->traits.get_min_value(),
                                               .max_control = level_output->traits.get_max_value(),
                                               .max_increment = (level_output->traits.get_max_value() - level_output->traits.get_min_value()) / 2.0f / control_interval_factor,
                                           },
                                           control_interval_factor,
                                           motion_parameters},
                                       PollingComponent{sample_time}, position_input_{position_input}, speed_input_{speed_input}, output_{level_output}, sub_tag_{sub_tag}

            {
            }

            Direction get_direction() const
            {
                return (Direction)controller::Controller::get_direction();
            }

            void call_setup() override {};
            void call_loop() override {};

            void go_to(float target_position)
            {
                auto reference_position = is_moving(this->get_status()) ? this->get_target_position() : this->position_input_->get_state();
                if (abs((int)100 * target_position - (int)100 * reference_position) < 1)
                    return;

                ESP_LOGD(TAG, "[%s]Going to position %f", sub_tag_, target_position);

                this->sync_position();
                controller::Controller::set_target_position(target_position);

                this->status_update();
            }

            void stop()
            {
                controller::Controller::set_target_speed(0);

                ESP_LOGD(TAG, "[%s]Stopping controller", sub_tag_);

                this->status_update();
            }

            void update() override
            {
                auto current_position = this->position_input_->get_state();
                auto current_speed = this->calculate_current_speed();

                auto control = controller::Controller::update(current_position, current_speed);

                if (this->log_loop_counter_++ % 10 == 0)
                    ESP_LOGI(TAG, "[%s]%d\t%f:%f\t%f:%f\t%f",
                             this->sub_tag_,
                             this->get_status(),
                             current_position,
                             this->get_calculated_position(),
                             current_speed,
                             this->get_calculated_speed(),
                             control);

                this->set_output_value(control);

                this->status_update();
            };

            void add_on_direction_change_callback(std::function<void(Direction)> callback)
            {
                this->direction_change_callback.add(std::move(callback));
            }

        protected:
            sensor::Sensor *const position_input_;
            sensor::Sensor *const speed_input_;
            number::Number *const output_;

            HighFrequencyLoopRequester high_frequency_loop_requester;

            CallbackManager<void(Direction)> direction_change_callback;

            void start_poller()
            {
                ESP_LOGI(TAG, "[%s]Starting high frequency loop", sub_tag_);
                this->high_frequency_loop_requester.start();
                PollingComponent::start_poller();
            }

            void stop_poller()
            {
                PollingComponent::stop_poller();
                this->high_frequency_loop_requester.stop();
                ESP_LOGI(TAG, "[%s]Stopping high frequency loop", sub_tag_);

                this->set_output_value(0);
            }

            float calculate_current_speed()
            {
                if (this->speed_input_ != nullptr)
                    return this->speed_input_->get_state();

                auto current_position = this->position_input_->get_state();
                auto last_position = this->speed_calculation_state_.position;

                auto current_time = millis();
                auto last_time = this->speed_calculation_state_.time;

                auto d_pos = current_position - last_position;
                auto d_time = current_time - last_time;
                auto speed = d_pos * 1000.0f / d_time;

                this->speed_calculation_state_ = {current_position, current_time};
                return speed;
            }

            void set_output_value(float value)
            {
                this->output_->make_call().set_value(value).perform();
            }

            void direction_update()
            {
                auto current = controller::Controller::get_direction();
                auto old = this->direction_;

                if (old == current)
                    return;

                ESP_LOGI(TAG, "[%s]Direction changed: %d -> %d", sub_tag_, old, current);

                if (current == controller::Direction::DC_SERVO_CONTROLLER_DIRECTION_STOP)
                    this->speed_calculation_state_ = {0, 0};

                this->direction_change_callback.call((Direction)current);
                this->direction_ = current;
            }

            void sync_position()
            {
                controller::Controller::set_current_position(this->position_input_->get_state());
            }

            void status_update()
            {
                auto current = controller::Controller::get_status();
                auto old = this->status_;

                if (old == current)
                    return;

                ESP_LOGI(TAG, "[%s]Status changed: %d -> %d", sub_tag_, old, current);

                auto old_running = is_moving(old);
                auto running = is_moving(current);

                if (running && !old_running)
                    this->start_poller();

                if (!running && old_running)
                    this->stop_poller();

                if (current == controller::Status::DC_SERVO_CONTROLLER_STATUS_ERROR)
                    this->status_set_error("Motion planner calculation error");
                else
                    this->status_clear_error();

                this->status_ = current;
                this->direction_update();
            };

        private:
            const char *sub_tag_;

            struct
            {
                float position;
                uint32_t time;
            } speed_calculation_state_;

            controller::Direction direction_ = controller::Direction::DC_SERVO_CONTROLLER_DIRECTION_STOP;
            controller::Status status_ = controller::Status::DC_SERVO_CONTROLLER_STATUS_IDLE;

            unsigned int log_loop_counter_ = 0;
        };
    }
}