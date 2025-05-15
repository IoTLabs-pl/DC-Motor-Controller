#pragma once

#include <array>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"

#include "driver/pulse_cnt.h"

namespace esphome
{
  namespace esp32_rotary_encoder
  {

    /// All possible restore modes for the rotary encoder
    enum RotaryEncoderRestoreMode
    {
      ROTARY_ENCODER_RESTORE_DEFAULT_ZERO, /// try to restore counter, otherwise set to zero
      ROTARY_ENCODER_ALWAYS_ZERO,          /// do not restore counter, always set to zero
    };

    struct RotaryEncoderSensorStore
    {
      ISRInternalGPIOPin pin_a;
      ISRInternalGPIOPin pin_b;

      int32_t min_value{INT32_MIN};
      int32_t max_value{INT32_MAX};
      int32_t last_state{0};
      int32_t offset{0};

      pcnt_unit_handle_t unit_handle{NULL};
    };

    class RotaryEncoderSensor : public sensor::Sensor, public Component
    {
    public:
      void set_pin_a(InternalGPIOPin *pin_a) { pin_a_ = pin_a; }
      void set_pin_b(InternalGPIOPin *pin_b) { pin_b_ = pin_b; }

      /** Set the restore mode of the rotary encoder.
       *
       * By default (if possible) the last known counter state is restored. Otherwise the value 0 is used.
       * Restoring the state can also be turned off.
       *
       * @param restore_mode The restore mode to use.
       */
      void set_restore_mode(RotaryEncoderRestoreMode restore_mode);

      /// Manually set the value of the counter.
      void set_value(int value);

      void enable();
      void disable();

      void set_min_value(int32_t min_value);
      void set_max_value(int32_t max_value);
      void set_publish_initial_value(bool publish_initial_value) { publish_initial_value_ = publish_initial_value; }

      // ========== INTERNAL METHODS ==========
      // (In most use cases you won't need these)
      void setup() override;
      void dump_config() override;
      void loop() override;

      float get_setup_priority() const override;

    protected:
      InternalGPIOPin *pin_a_;
      InternalGPIOPin *pin_b_;
      bool publish_initial_value_;
      ESPPreferenceObject rtc_;
      RotaryEncoderRestoreMode restore_mode_{ROTARY_ENCODER_RESTORE_DEFAULT_ZERO};

      RotaryEncoderSensorStore store_{};
    };

    template <typename... Ts>
    class RotaryEncoderSetValueAction : public Action<Ts...>
    {
    public:
      RotaryEncoderSetValueAction(RotaryEncoderSensor *encoder) : encoder_(encoder) {}
      TEMPLATABLE_VALUE(int, value)

      void play(Ts... x) override { this->encoder_->set_value(this->value_.value(x...)); }

    protected:
      RotaryEncoderSensor *encoder_;
    };


    template <typename... Ts>
    class RotaryEncoderEnableAction : public Action<Ts...>
    {
    public:
      RotaryEncoderEnableAction(RotaryEncoderSensor *encoder) : encoder_(encoder) {}

      void play(Ts... x) override { this->encoder_->enable(); }

    protected:
      RotaryEncoderSensor *encoder_;
    };

    template <typename... Ts>
    class RotaryEncoderDisableAction : public Action<Ts...>
    {
    public:
      RotaryEncoderDisableAction(RotaryEncoderSensor *encoder) : encoder_(encoder) {}

      void play(Ts... x) override { this->encoder_->disable(); }

    protected:
      RotaryEncoderSensor *encoder_;
    };


  } // namespace esp32_rotary_encoder
} // namespace esphome
