#pragma once

#include <algorithm>
#include <limits>
#include <valarray>

#include <esphome/core/log.h>
using esphome::esp_log_printf_;

namespace dc_servo
{
    namespace PID
    {
        template <class Tp>
        struct Config
        {
            Tp kp, ki, kd;
            Tp min_control = -std::numeric_limits<Tp>::infinity();
            Tp max_control = std::numeric_limits<Tp>::infinity();
            Tp max_increment = std::numeric_limits<Tp>::infinity();
        };

        template <class Tp>
        class PID
        {
        public:
            PID(Tp sample_time, Config<Tp> c) : K_{c.kp + c.ki * sample_time + c.kd / sample_time,
                                                   -c.kp - 2 * c.kd / sample_time,
                                                   c.kd / sample_time},
                                                min_control_{c.min_control},
                                                max_control_{c.max_control},
                                                max_increment_{c.max_increment}
            {
                this->reset();
            }

            Tp update(Tp setpoint, Tp process_variable)
            {
                Tp current_error = setpoint - process_variable;

                this->errors_ = {current_error, this->errors_[0], this->errors_[1]};

                ESP_LOGD("PID", "SP: %f PV: %f E: %f %f %f", setpoint, process_variable, this->errors_[0], this->errors_[1], this->errors_[2]);

                this->control_ += std::clamp((this->errors_ * this->K_).sum(), -max_increment_, max_increment_);

                this->control_ = std::clamp(this->control_, this->min_control_, this->max_control_);

                return this->control_;
            }

            void reset()
            {
                this->control_ = (Tp)0;
                this->errors_ = std::valarray<Tp>(3);
            }

        protected:
            std::valarray<Tp> K_;
            std::valarray<Tp> errors_;

            Tp control_;

            Tp min_control_, max_control_, max_increment_;
        };
    }
}