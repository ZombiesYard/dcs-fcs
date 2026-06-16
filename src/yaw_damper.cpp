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

double wrap_pi(double angle) {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

double signed_curve(double value) {
    const double magnitude = std::abs(value);
    return std::copysign(magnitude * magnitude, value);
}

}  // namespace

double clamp_unit(double value) {
    return clamp(value, -1.0, 1.0);
}

YawDamper::YawDamper(AppConfig config) : config_(std::move(config)) {}

void YawDamper::set_config(AppConfig config) {
    config_ = std::move(config);
}

void YawDamper::reset() {
    has_last_physical_ = false;
    last_physical_ = 0.0;
    filtered_yaw_rate_ = 0.0;
    integrated_yaw_rate_ = 0.0;
    trim_bias_ = 0.0;
    trim_candidate_ = 0.0;
    trim_candidate_yaw_abs_ = 0.0;
    has_trim_candidate_ = false;
    last_user_override_ = false;
    has_last_collective_ = false;
    last_collective_ = 0.0;
    assist_offset_ = 0.0;
    has_heading_ref_ = false;
    heading_ref_ = 0.0;
    pedal_command_active_ = false;
    has_last_heading_ = false;
    last_heading_ = 0.0;
    filtered_heading_rate_ = 0.0;
    release_brake_timer_ = 0.0;
}

YawDamperOutput YawDamper::update(const YawDamperInput& input) {
    const double dt = clamp(input.dt, 0.001, 0.2);
    const double physical = clamp_unit(input.physical_rudder);
    const double collective = clamp(input.collective, 0.0, 1.0);

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
    const bool heading_mode =
        config_.control_mode == "heading_hold" || config_.control_mode == "heading_command";
    double measured_heading_rate = config_.yaw_rate_sign * filtered_yaw_rate_;
    if (input.heading_valid) {
        if (has_last_heading_) {
            const double raw_heading_rate = wrap_pi(input.heading - last_heading_) / dt;
            if (config_.filter_time <= 0.0) {
                filtered_heading_rate_ = raw_heading_rate;
            } else {
                const double alpha = dt / (config_.filter_time + dt);
                filtered_heading_rate_ += alpha * (raw_heading_rate - filtered_heading_rate_);
            }
        } else {
            filtered_heading_rate_ = measured_heading_rate;
            has_last_heading_ = true;
        }
        last_heading_ = input.heading;
        measured_heading_rate = filtered_heading_rate_;
    } else {
        has_last_heading_ = false;
        filtered_heading_rate_ = measured_heading_rate;
    }

    double collective_rate = 0.0;
    double collective_feedforward = 0.0;
    if (input.collective_valid) {
        collective_rate = has_last_collective_ ? (collective - last_collective_) / dt : 0.0;
        has_last_collective_ = true;
        last_collective_ = collective;
        const double rate_term = clamp(
            config_.collective_sign * config_.collective_rate_gain * collective_rate,
            -config_.collective_rate_limit,
            config_.collective_rate_limit);
        collective_feedforward = clamp(
            config_.collective_sign * config_.collective_gain * collective + rate_term,
            -config_.max_assist,
            config_.max_assist);
    } else {
        has_last_collective_ = false;
    }

    double target_assist = 0.0;
    double integral_assist = 0.0;
    double yaw_rate_command = 0.0;
    double heading_error = 0.0;
    bool direct_turn_output = false;
    std::string reason = "assist";
    if (!allowed) {
        if (!input.telemetry_fresh) reason = "stale telemetry";
        else if (!input.aircraft_is_ah64) reason = "not AH-64D";
        else if (!input.input_valid) reason = "input invalid";
        else reason = "disabled";
        integrated_yaw_rate_ = 0.0;
        has_heading_ref_ = false;
        pedal_command_active_ = false;
        has_last_heading_ = false;
        release_brake_timer_ = 0.0;
    } else if (!heading_mode && user_override) {
        reason = "pedal override";
        integrated_yaw_rate_ = 0.0;
        if (config_.trim_capture_enabled > 0.5 &&
            std::abs(physical) >= config_.trim_capture_min_pedal &&
            std::abs(filtered_yaw_rate_) <= config_.trim_capture_yaw_rate &&
            pedal_rate <= config_.trim_capture_pedal_rate) {
            const double yaw_abs = std::abs(filtered_yaw_rate_);
            if (!has_trim_candidate_ || yaw_abs < trim_candidate_yaw_abs_) {
                trim_candidate_ = clamp(physical - collective_feedforward, -config_.max_assist, config_.max_assist);
                trim_candidate_yaw_abs_ = yaw_abs;
                has_trim_candidate_ = true;
            }
        }
    } else if (heading_mode) {
        const double rate = with_deadband(measured_heading_rate, config_.yaw_rate_deadband);
        bool turn_command = false;
        if (pedal_command_active_) {
            turn_command = std::abs(physical) >= config_.pedal_command_exit_deadzone;
        } else {
            turn_command = std::abs(physical) >= config_.pedal_command_deadzone;
        }
        const bool released_turn_command = pedal_command_active_ && !turn_command;

        if (turn_command) {
            const double command_axis = signed_curve(physical);
            yaw_rate_command = clamp(
                config_.pedal_command_sign * config_.turn_rate_max * command_axis,
                -config_.turn_rate_max,
                config_.turn_rate_max);
            target_assist = clamp_unit(config_.pedal_command_sign * physical);
            direct_turn_output = true;
            if (input.heading_valid) {
                heading_ref_ = input.heading;
                has_heading_ref_ = true;
            }
            integrated_yaw_rate_ = 0.0;
            release_brake_timer_ = 0.0;
            reason = "turn command";
        } else if (input.heading_valid) {
            if (!has_heading_ref_) {
                heading_ref_ = input.heading;
                has_heading_ref_ = true;
            }

            if (released_turn_command) {
                heading_ref_ = input.heading;
                integrated_yaw_rate_ = 0.0;
                release_brake_timer_ = config_.release_brake_time;
            }
            heading_error = with_deadband(wrap_pi(heading_ref_ - input.heading), config_.heading_error_deadband);
            yaw_rate_command = clamp(
                config_.heading_kp * heading_error,
                -config_.heading_rate_limit,
                config_.heading_rate_limit);
            reason = release_brake_timer_ > 0.0 ? "release brake" : "heading hold";
        } else {
            has_heading_ref_ = false;
            integrated_yaw_rate_ = 0.0;
            release_brake_timer_ = 0.0;
            reason = "rate hold no heading";
        }
        pedal_command_active_ = turn_command;

        const double rate_error = yaw_rate_command - rate;
        if (direct_turn_output) {
            integrated_yaw_rate_ = 0.0;
        } else if (!turn_command && config_.ki > 0.0 && config_.integral_limit > 0.0) {
            integrated_yaw_rate_ += rate_error * dt;
            const double state_limit = config_.integral_limit / config_.ki;
            integrated_yaw_rate_ = clamp(integrated_yaw_rate_, -state_limit, state_limit);
            integral_assist = config_.ki * integrated_yaw_rate_;
        } else if (turn_command || config_.ki <= 0.0 || config_.integral_limit <= 0.0) {
            integrated_yaw_rate_ = 0.0;
        }

        if (!direct_turn_output) {
            const bool release_brake = !turn_command && release_brake_timer_ > 0.0;
            const double feedback_kp = release_brake ? config_.release_brake_kp : config_.kp;
            double feedback_assist = config_.yaw_response_sign * (feedback_kp * rate_error + integral_assist);
            if (!turn_command) {
                const double assist_limit = release_brake ? config_.release_brake_max_assist : config_.heading_hold_max_assist;
                feedback_assist = clamp(
                    feedback_assist,
                    -assist_limit,
                    assist_limit);
            }
            if (release_brake_timer_ > 0.0) {
                release_brake_timer_ = std::max(0.0, release_brake_timer_ - dt);
            }
            target_assist = clamp(
                collective_feedforward + feedback_assist,
                -config_.max_assist,
                config_.max_assist);
        }
    } else {
        if (last_user_override_ && has_trim_candidate_) {
            trim_bias_ = trim_candidate_;
            integrated_yaw_rate_ = 0.0;
        }
        has_trim_candidate_ = false;
        const double rate = with_deadband(filtered_yaw_rate_, config_.yaw_rate_deadband);
        if (config_.ki > 0.0 && config_.integral_limit > 0.0) {
            integrated_yaw_rate_ += rate * dt;
            const double state_limit = config_.integral_limit / config_.ki;
            integrated_yaw_rate_ = clamp(integrated_yaw_rate_, -state_limit, state_limit);
            integral_assist = config_.ki * integrated_yaw_rate_;
        } else {
            integrated_yaw_rate_ = 0.0;
        }
        target_assist = clamp(
            collective_feedforward + trim_bias_ - config_.assist_sign * (config_.kp * rate + integral_assist),
            -config_.max_assist,
            config_.max_assist);
    }
    last_user_override_ = user_override;

    if (direct_turn_output) {
        assist_offset_ = target_assist;
    } else {
        const bool reducing_magnitude = std::abs(target_assist) < std::abs(assist_offset_);
        double fade_time = reducing_magnitude ? config_.fade_out_time : config_.fade_in_time;
        if (input.collective_valid &&
            std::abs(collective_rate) >= config_.collective_transient_rate_threshold &&
            config_.collective_transient_fade_time < fade_time) {
            fade_time = config_.collective_transient_fade_time;
        }
        const double max_delta = fade_time <= 0.0 ? config_.max_assist : (config_.max_assist * dt / fade_time);
        assist_offset_ = move_towards(assist_offset_, target_assist, max_delta);
    }

    YawDamperOutput output;
    output.final_rudder = heading_mode && allowed ? clamp_unit(assist_offset_) : clamp_unit(physical + assist_offset_);
    output.assist_offset = assist_offset_;
    output.integral_assist = integral_assist;
    output.trim_bias = trim_bias_;
    output.collective_feedforward = collective_feedforward;
    output.collective = collective;
    output.collective_rate = collective_rate;
    output.filtered_yaw_rate = filtered_yaw_rate_;
    output.heading_rate = measured_heading_rate;
    output.yaw_rate_command = yaw_rate_command;
    output.heading = input.heading;
    output.heading_ref = heading_ref_;
    output.heading_error = heading_error;
    output.user_override = !heading_mode && user_override;
    output.assist_active = allowed && (heading_mode || !user_override) && std::abs(assist_offset_) > 0.0001;
    output.reason = std::move(reason);
    return output;
}

}  // namespace autorudder
