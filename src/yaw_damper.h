#pragma once

#include "config.h"

#include <string>

namespace autorudder {

struct YawDamperInput {
    double dt = 0.01;
    double physical_rudder = 0.0;
    double yaw_rate_z = 0.0;
    double heading = 0.0;
    double collective = 0.0;
    bool heading_valid = false;
    bool collective_valid = false;
    bool telemetry_fresh = false;
    bool aircraft_is_ah64 = false;
    bool input_valid = true;
    bool assist_enabled = true;
};

struct YawDamperOutput {
    double final_rudder = 0.0;
    double assist_offset = 0.0;
    double integral_assist = 0.0;
    double trim_bias = 0.0;
    double collective_feedforward = 0.0;
    double collective = 0.0;
    double collective_rate = 0.0;
    double filtered_yaw_rate = 0.0;
    double heading_rate = 0.0;
    double yaw_rate_command = 0.0;
    double heading = 0.0;
    double heading_ref = 0.0;
    double heading_error = 0.0;
    bool user_override = false;
    bool assist_active = false;
    std::string reason;
};

class YawDamper {
public:
    explicit YawDamper(AppConfig config);

    YawDamperOutput update(const YawDamperInput& input);
    void set_config(AppConfig config);
    void reset();

private:
    AppConfig config_;
    bool has_last_physical_ = false;
    double last_physical_ = 0.0;
    double filtered_yaw_rate_ = 0.0;
    double integrated_yaw_rate_ = 0.0;
    double trim_bias_ = 0.0;
    double trim_candidate_ = 0.0;
    double trim_candidate_yaw_abs_ = 0.0;
    bool has_trim_candidate_ = false;
    bool last_user_override_ = false;
    bool has_last_collective_ = false;
    double last_collective_ = 0.0;
    double assist_offset_ = 0.0;
    bool has_heading_ref_ = false;
    double heading_ref_ = 0.0;
    bool pedal_command_active_ = false;
    bool has_last_heading_ = false;
    double last_heading_ = 0.0;
    double filtered_heading_rate_ = 0.0;
    double release_brake_timer_ = 0.0;
};

double clamp_unit(double value);

}  // namespace autorudder
