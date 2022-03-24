#pragma once

#include "esphome.h"

const char *TAG = "opentherm_switch";

class OpenthermSwitch : public Switch, public Component {
 public:
  void setup() override {
    auto restored = this->get_initial_state();
    if (!restored.has_value())
      return;

    ESP_LOGD(TAG, "  Restored state %s", ONOFF(*restored));
    if (*restored) {
      this->turn_on();
    } else {
      this->turn_off();
    }
  }

  void write_state(bool state) override {
    // This will be called every time the user requests a state change.

    ESP_LOGD(TAG, "write_state");

    // Acknowledge new state by publishing it
    publish_state(state);
  }
};
