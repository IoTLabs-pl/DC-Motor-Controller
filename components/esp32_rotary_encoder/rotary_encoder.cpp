#include "rotary_encoder.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include "esp_check.h"

namespace esphome
{
  namespace esp32_rotary_encoder
  {

    static const char *const TAG = "esp32_rotary_encoder";

    void RotaryEncoderSensor::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up Rotary Encoder '%s'...", this->name_.c_str());

      int32_t initial_value = 0;
      switch (this->restore_mode_)
      {
      case ROTARY_ENCODER_RESTORE_DEFAULT_ZERO:
        this->rtc_ = global_preferences->make_preference<int32_t>(this->get_object_id_hash());
        if (!this->rtc_.load(&initial_value))
        {
          initial_value = 0;
        }
        break;
      case ROTARY_ENCODER_ALWAYS_ZERO:
        initial_value = 0;
        break;
      }
      initial_value = clamp(initial_value, this->store_.min_value, this->store_.max_value);

      this->store_.offset = initial_value;

      pcnt_unit_config_t unit_config = {
          .low_limit = INT16_MIN,
          .high_limit = INT16_MAX,
          .flags = {.accum_count = 1},
      };
      ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &(this->store_.unit_handle)));
      ESP_ERROR_CHECK(pcnt_unit_add_watch_point(this->store_.unit_handle, unit_config.low_limit));
      ESP_ERROR_CHECK(pcnt_unit_add_watch_point(this->store_.unit_handle, unit_config.high_limit));

      pcnt_glitch_filter_config_t filter_config = {
          .max_glitch_ns = 1023,
      };
      ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(this->store_.unit_handle, &filter_config));

      pcnt_channel_handle_t channel_a_handle = NULL;
      pcnt_chan_config_t channnel_a_config = {
          .edge_gpio_num = this->pin_a_->get_pin(),
          .level_gpio_num = this->pin_b_->get_pin(),
      };
      ESP_ERROR_CHECK(pcnt_new_channel(this->store_.unit_handle, &channnel_a_config, &channel_a_handle));

      pcnt_channel_handle_t channel_b_handle = NULL;
      pcnt_chan_config_t channnel_b_config = {
          .edge_gpio_num = this->pin_b_->get_pin(),
          .level_gpio_num = this->pin_a_->get_pin(),
      };
      ESP_ERROR_CHECK(pcnt_new_channel(this->store_.unit_handle, &channnel_b_config, &channel_b_handle));

      ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
      ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
      ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
      ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

      ESP_ERROR_CHECK(pcnt_unit_enable(this->store_.unit_handle));
    }

    void RotaryEncoderSensor::dump_config()
    {
      LOG_SENSOR("", "Rotary Encoder", this);
      LOG_PIN("  Pin A: ", this->pin_a_);
      LOG_PIN("  Pin B: ", this->pin_b_);

      const LogString *restore_mode = LOG_STR("");
      switch (this->restore_mode_)
      {
      case ROTARY_ENCODER_RESTORE_DEFAULT_ZERO:
        restore_mode = LOG_STR("Restore (Defaults to zero)");
        break;
      case ROTARY_ENCODER_ALWAYS_ZERO:
        restore_mode = LOG_STR("Always zero");
        break;
      }

      ESP_LOGCONFIG(TAG, "  Restore Mode: %s", LOG_STR_ARG(restore_mode));
    }
    void RotaryEncoderSensor::loop()
    {
      int hw_count;
      ESP_ERROR_CHECK(pcnt_unit_get_count(this->store_.unit_handle, &hw_count));

      int32_t counter = clamp(this->store_.offset + hw_count, this->store_.min_value, this->store_.max_value);

      if (this->store_.last_state != counter || this->publish_initial_value_)
      {
        if (this->restore_mode_ == ROTARY_ENCODER_RESTORE_DEFAULT_ZERO)
        {
          this->rtc_.save(&counter);
        }
        this->store_.last_state = counter;
        this->publish_state(counter);
        this->publish_initial_value_ = false;
      }
    }

    float RotaryEncoderSensor::get_setup_priority() const { return setup_priority::DATA; }
    void RotaryEncoderSensor::set_restore_mode(RotaryEncoderRestoreMode restore_mode)
    {
      this->restore_mode_ = restore_mode;
    }
    void RotaryEncoderSensor::set_value(int value)
    {
      this->store_.offset += (value - this->state);
      this->loop();
    }
    void RotaryEncoderSensor::enable()
    {
      ESP_ERROR_CHECK(pcnt_unit_start(this->store_.unit_handle));
    }
    void RotaryEncoderSensor::disable()
    {
      ESP_ERROR_CHECK(pcnt_unit_stop(this->store_.unit_handle));
    }
    void RotaryEncoderSensor::set_min_value(int32_t min_value) { this->store_.min_value = min_value; }
    void RotaryEncoderSensor::set_max_value(int32_t max_value) { this->store_.max_value = max_value; }

  } // namespace rotary_encoder
} // namespace esphome
