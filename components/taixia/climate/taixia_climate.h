#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"
#include "esphome/components/climate/climate.h"
#include "../taixia.h"

namespace esphome {
namespace taixia {

enum TaiXiaPreset : uint8_t {
  CLIMATE_PRESET_AIR_DETECT = climate::CLIMATE_PRESET_ACTIVITY + 1,
  CLIMATE_PRESET_ANTI_MILDEW = climate::CLIMATE_PRESET_ACTIVITY + 2,
  CLIMATE_PRESET_SELF_CLEAN = climate::CLIMATE_PRESET_ACTIVITY + 3,
  CLIMATE_PRESET_BODY_MOTION = climate::CLIMATE_PRESET_ACTIVITY + 4,
};

class Constants {
 public:
  static const char *const AIR_DETECT;
  static const char *const ANTI_MILDEW;
  static const char *const SELF_CLEAN;
  static const char *const BODY_MOTION;
};

class TaiXiaClimate : public climate::Climate, public TaiXiaListener, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  climate::ClimateTraits traits() override;

  void set_sa_id(uint8_t sa_id) { this->sa_id_ = sa_id; }
  void set_max_temperature(float temp) { this->max_temp_ = temp; }
  void set_min_temperature(float temp) { this->min_temp_ = temp; }
  void set_temperature_step(float step) { this->temp_step_ = step; }
  void set_supported_modes(const climate::ClimateModeMask &modes);
  void set_supported_fan_modes(const climate::ClimateFanModeMask &modes);
  void set_supported_swing_modes(const climate::ClimateSwingModeMask &modes);
  void set_supported_preset_modes(const climate::ClimatePresetMask &modes);
  void set_supported_humidity(bool feature) { this->supported_humidity_ = feature; }

  void set_taixia_parent(TaiXia *parent) { this->parent_ = parent; }
  
  // 取得 turn off trigger
  Trigger<> *get_turn_off_trigger() const { return this->turn_off_trigger_; }
  
  // 設定是否要覆寫關機行為
  void set_override_turn_off(bool override) { this->override_turn_off_ = override; }

 protected:
  void control(const climate::ClimateCall &call) override;

  TaiXia *parent_;
  uint8_t sa_id_{SA_ID_CLIMATE};
  bool supports_cool_{true};
  bool supports_heat_{false};
  bool supports_dry_{false};
  bool supports_fan_only_{false};
  bool supported_humidity_{false};
  uint8_t preset_modes_;
  uint16_t fan_modes_;
  uint8_t swing_modes_;
  float max_temp_{19.0};
  float min_temp_{43.0};
  float temp_step_{1.0};
  esphome::climate::ClimateTraits traits_;

  void reset_state_();
  bool update_status_();
  void send_power_off_();
  void send_power_off_(bool wait_response);
  void send_power_on_mode_(climate::ClimateMode mode);
  bool should_restart_from_fan_only_(climate::ClimateMode requested_mode) const;
  void schedule_mode_after_power_cycle_(climate::ClimateMode mode);
  void start_cool_mode_timer_();
  void clear_cool_mode_timer_();
  bool cool_mode_has_elapsed_(uint32_t duration) const;
  void start_anti_mildew_fan_();
  void cancel_anti_mildew_fan_();
  void start_anti_mildew_fan_response_suppression_();
  bool should_suppress_anti_mildew_fan_response_();

  void handle_response(std::vector<uint8_t> &response) override;
  
  // turn off trigger
  Trigger<> *turn_off_trigger_{new Trigger<>()};
  // 是否覆寫關機行為（有設定 on_turn_off 時會是 true）
  bool override_turn_off_{false};
  bool cool_mode_timer_active_{false};
  uint32_t cool_mode_started_at_{0};
  bool anti_mildew_fan_pending_{false};
  uint32_t anti_mildew_turn_off_at_{0};
  bool suppress_anti_mildew_fan_response_{false};
  uint32_t suppress_anti_mildew_fan_response_until_{0};
  bool pending_mode_after_power_cycle_{false};
  climate::ClimateMode pending_power_cycle_mode_{climate::CLIMATE_MODE_OFF};
  uint32_t pending_power_cycle_at_{0};
};

}  // namespace taixia
}  // namespace esphome
