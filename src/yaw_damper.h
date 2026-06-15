#pragma once

#include "config.h"

#include <string>

namespace autorudder {

struct YawDamperInput {
    double dt = 0.01;
    double physical_rudder = 0.0;
    double yaw_rate_z = 0.0;
    bool telemetry_fresh = false;
    bool aircraft_is_ah64 = false;
    bool input_valid = true;
    bool assist_enabled = true;
};

struct YawDamperOutput {
    double final_rudder = 0.0;
    double assist_offset = 0.0;
    double filtered_yaw_rate = 0.0;
    bool user_override = false;
    bool assist_active = false;
    std::string reason;
};

class YawDamper {
public:
    explicit YawDamper(AppConfig config);

    YawDamperOutput update(const YawDamperInput& input);
    void reset();

private:
    AppConfig config_;
    bool has_last_physical_ = false;
    double last_physical_ = 0.0;
    double filtered_yaw_rate_ = 0.0;
    double assist_offset_ = 0.0;
};

double clamp_unit(double value);

}  // namespace autorudder
