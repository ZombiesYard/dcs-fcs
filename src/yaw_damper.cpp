#include "yaw_damper.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autorudder {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double move_towards(double current, double target, double max_delta) {
    if (target > current) {
        return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
}

double with_deadband(double value, double deadband) {
    return std::abs(value) < deadband ? 0.0 : value;
}

}  // namespace

double clamp_unit(double value) {
    return clamp(value, -1.0, 1.0);
}

YawDamper::YawDamper(AppConfig config) : config_(std::move(config)) {}

void YawDamper::reset() {
    has_last_physical_ = false;
    last_physical_ = 0.0;
    filtered_yaw_rate_ = 0.0;
    assist_offset_ = 0.0;
}

YawDamperOutput YawDamper::update(const YawDamperInput& input) {
    const double dt = clamp(input.dt, 0.001, 0.2);
    const double physical = clamp_unit(input.physical_rudder);

    if (config_.filter_time <= 0.0) {
        filtered_yaw_rate_ = input.yaw_rate_z;
    } else {
        const double alpha = dt / (config_.filter_time + dt);
        filtered_yaw_rate_ += alpha * (input.yaw_rate_z - filtered_yaw_rate_);
    }

    const double pedal_rate = has_last_physical_ ? std::abs(physical - last_physical_) / dt : 0.0;
    has_last_physical_ = true;
    last_physical_ = physical;

    const bool user_override =
        std::abs(physical) >= config_.pedal_override_threshold ||
        pedal_rate >= config_.pedal_rate_override_threshold;

    const bool allowed =
        input.telemetry_fresh &&
        input.aircraft_is_ah64 &&
        input.input_valid &&
        input.assist_enabled;

    double target_assist = 0.0;
    std::string reason = "assist";
    if (!allowed) {
        if (!input.telemetry_fresh) reason = "stale telemetry";
        else if (!input.aircraft_is_ah64) reason = "not AH-64D";
        else if (!input.input_valid) reason = "input invalid";
        else reason = "disabled";
    } else if (user_override) {
        reason = "pedal override";
    } else {
        const double rate = with_deadband(filtered_yaw_rate_, config_.yaw_rate_deadband);
        target_assist = clamp(
            -config_.assist_sign * config_.kp * rate,
            -config_.max_assist,
            config_.max_assist);
    }

    const bool reducing_magnitude = std::abs(target_assist) < std::abs(assist_offset_);
    const double fade_time = reducing_magnitude ? config_.fade_out_time : config_.fade_in_time;
    const double max_delta = fade_time <= 0.0 ? config_.max_assist : (config_.max_assist * dt / fade_time);
    assist_offset_ = move_towards(assist_offset_, target_assist, max_delta);

    YawDamperOutput output;
    output.final_rudder = clamp_unit(physical + assist_offset_);
    output.assist_offset = assist_offset_;
    output.filtered_yaw_rate = filtered_yaw_rate_;
    output.user_override = user_override;
    output.assist_active = allowed && !user_override && std::abs(assist_offset_) > 0.0001;
    output.reason = std::move(reason);
    return output;
}

}  // namespace autorudder
