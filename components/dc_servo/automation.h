#pragma once

#include <esphome/core/automation.h>
#include <esphome/core/base_automation.h>

#include "dc_servo.h"

namespace esphome
{
    namespace dc_servo_component
    {
        template <typename... Ts>
        class DCServoIsMovingForwardCondition : public Condition<Ts...>
        {
        public:
            explicit DCServoIsMovingForwardCondition(DCServo *servo) : servo_{servo} {}

            bool check(Ts... x) override
            {
                return this->servo_->get_direction() == DC_SERVO_DIRECTION_FORWARD;
            }

        protected:
            DCServo *servo_;
        };

        template <typename... Ts>
        class DCServoIsMovingBackwardCondition : public Condition<Ts...>
        {
        public:
            explicit DCServoIsMovingBackwardCondition(DCServo *servo) : servo_{servo} {}

            bool check(Ts... x) override
            {
                return this->servo_->get_direction() == DC_SERVO_DIRECTION_BACKWARD;
            }

        protected:
            DCServo *servo_;
        };

        template <typename... Ts>
        class DCServoIsStoppedCondition : public Condition<Ts...>
        {
        public:
            explicit DCServoIsStoppedCondition(DCServo *servo) : servo_{servo} {}

            bool check(Ts... x) override
            {
                return this->servo_->get_direction() == DC_SERVO_DIRECTION_STOP;
            }

        protected:
            DCServo *servo_;
        };

        template <typename... Ts>
        class DCServoGoToAction : public WaitUntilAction<Ts...>
        {
        public:
            explicit DCServoGoToAction(DCServo *servo) : WaitUntilAction<Ts...>{
                                                             new DCServoIsStoppedCondition<Ts...>(servo)},
                                                         servo_{servo}
            {
            }

            TEMPLATABLE_VALUE(float, position)

            void play_complex(Ts... x) override
            {
                auto target = this->position_.value(x...);
                this->servo_->go_to(target);
                WaitUntilAction<Ts...>::play_complex(x...);
            }

        protected:
            DCServo *servo_;
        };

        template <typename... Ts>
        class DCServoStopAction : public Action<Ts...>
        {
        public:
            explicit DCServoStopAction(DCServo *servo) : servo_{servo} {}

            void play(Ts... x) override
            {
                this->servo_->stop();
            }

        protected:
            DCServo *servo_;
        };

        class DCServoDirectionChangeTrigger : public Trigger<Direction>
        {
        public:
            explicit DCServoDirectionChangeTrigger(DCServo *servo)
            {
                servo->add_on_direction_change_callback([this](Direction direction)
                                                        { this->trigger(direction); });
            }
        };
    }
}
