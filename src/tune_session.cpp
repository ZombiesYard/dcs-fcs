#include "tune_session.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace autorudder {
namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double safe_dt(double dt) {
    return clamp(dt, 0.001, 0.25);
}

int sign_with_deadband(double value, double deadband) {
    if (value > deadband) {
        return 1;
    }
    if (value < -deadband) {
        return -1;
    }
    return 0;
}

std::string fixed2(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
}

std::string fixed3(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

}  // namespace

TuneSessionAnalyzer::TuneSessionAnalyzer(TuneConfig config) : config_(config) {}

void TuneSessionAnalyzer::reset() {
    raw_samples_ = 0;
    usable_seconds_ = 0.0;
    excluded_unstable_seconds_ = 0.0;
    normal_seconds_ = 0.0;
    heading_rate_sq_seconds_ = 0.0;
    heading_rate_peak_ = 0.0;
    final_abs_seconds_ = 0.0;
    saturation_seconds_ = 0.0;
    heading_rate_sign_changes_ = 0;
    last_heading_rate_sign_ = 0;
    static_collective_seconds_ = 0.0;
    static_feedback_seconds_ = 0.0;
    static_gain_delta_seconds_ = 0.0;
    collective_transient_seconds_ = 0.0;
    collective_transient_sq_seconds_ = 0.0;
    collective_transient_peak_ = 0.0;
}

void TuneSessionAnalyzer::add(const TuneSample& sample) {
    ++raw_samples_;
    const double dt = safe_dt(sample.dt);
    const bool centered = std::abs(sample.pedal) <= 0.03;
    const bool base_valid =
        sample.fresh &&
        sample.aircraft_is_ah64 &&
        sample.input_valid &&
        sample.heading_valid &&
        sample.heading_hold_mode &&
        centered;
    if (!base_valid) {
        return;
    }

    if (std::abs(sample.heading_rate) > 1.0 || std::abs(sample.heading_error) > 0.7) {
        excluded_unstable_seconds_ += dt;
        return;
    }

    usable_seconds_ += dt;
    const bool collective_quiet = !sample.collective_valid || std::abs(sample.collective_rate) <= 0.03;
    if (collective_quiet) {
        normal_seconds_ += dt;
        heading_rate_sq_seconds_ += sample.heading_rate * sample.heading_rate * dt;
        heading_rate_peak_ = std::max(heading_rate_peak_, std::abs(sample.heading_rate));
        final_abs_seconds_ += std::abs(sample.final_rudder) * dt;
        if (config_.heading_hold_max_assist > 0.0 &&
            std::abs(sample.final_rudder) >= 0.95 * config_.heading_hold_max_assist) {
            saturation_seconds_ += dt;
        }

        const int rate_sign = sign_with_deadband(sample.heading_rate, 0.025);
        if (rate_sign != 0) {
            if (last_heading_rate_sign_ != 0 && rate_sign != last_heading_rate_sign_) {
                ++heading_rate_sign_changes_;
            }
            last_heading_rate_sign_ = rate_sign;
        }
    }

    if (sample.collective_valid &&
        std::abs(sample.collective_rate) <= 0.02 &&
        std::abs(sample.heading_rate) <= 0.08 &&
        std::abs(sample.heading_error) <= 0.08 &&
        sample.collective >= 0.08) {
        const double feedback = sample.final_rudder - sample.collective_feedforward;
        static_collective_seconds_ += dt;
        static_feedback_seconds_ += feedback * dt;
        const double denominator = config_.collective_sign * sample.collective;
        if (std::abs(denominator) >= 0.05) {
            static_gain_delta_seconds_ += (feedback / denominator) * dt;
        }
    }

    if (sample.collective_valid && std::abs(sample.collective_rate) > 0.04) {
        collective_transient_seconds_ += dt;
        collective_transient_sq_seconds_ += sample.heading_rate * sample.heading_rate * dt;
        collective_transient_peak_ = std::max(collective_transient_peak_, std::abs(sample.heading_rate));
    }
}

TuneReport TuneSessionAnalyzer::report() const {
    TuneReport report;
    report.raw_samples = raw_samples_;
    report.usable_seconds = usable_seconds_;
    report.excluded_unstable_seconds = excluded_unstable_seconds_;
    report.normal_seconds = normal_seconds_;
    if (normal_seconds_ > 0.0) {
        report.heading_rate_rms = std::sqrt(heading_rate_sq_seconds_ / normal_seconds_);
        report.final_abs_mean = final_abs_seconds_ / normal_seconds_;
        report.saturation_ratio = saturation_seconds_ / normal_seconds_;
        report.oscillation_rate = static_cast<double>(heading_rate_sign_changes_) / normal_seconds_;
    }
    report.heading_rate_peak = heading_rate_peak_;
    report.static_collective_seconds = static_collective_seconds_;
    if (static_collective_seconds_ > 0.0) {
        report.static_feedback_mean = static_feedback_seconds_ / static_collective_seconds_;
    }
    report.collective_transient_seconds = collective_transient_seconds_;
    if (collective_transient_seconds_ > 0.0) {
        report.collective_transient_rms = std::sqrt(collective_transient_sq_seconds_ / collective_transient_seconds_);
    }
    report.collective_transient_peak = collective_transient_peak_;

    if (usable_seconds_ < 5.0) {
        report.recommendations.push_back("collect more centered-pedal AH-64D heading-hold data");
        return report;
    }

    if (static_collective_seconds_ >= 4.0) {
        const double avg_delta = static_gain_delta_seconds_ / static_collective_seconds_;
        if (std::abs(avg_delta) >= 0.03) {
            const double limited_delta = clamp(avg_delta, -0.15, 0.15);
            report.recommended_collective_gain = clamp(config_.collective_gain + limited_delta, 0.0, 1.50);
            report.recommendations.push_back(
                "collective_gain " + fixed2(config_.collective_gain) + " -> " +
                fixed2(*report.recommended_collective_gain) +
                " because steady feedback mean is " + fixed3(report.static_feedback_mean));
        } else {
            report.recommendations.push_back("collective_gain looks close in steady collective samples");
        }
    } else {
        report.recommendations.push_back("collective_gain needs more steady centered collective samples");
    }

    if (collective_transient_seconds_ >= 2.0) {
        if (report.collective_transient_rms >= 0.08 || report.collective_transient_peak >= 0.18) {
            const double step = report.collective_transient_peak >= 0.25 ? 0.10 : 0.05;
            report.recommended_collective_rate_gain = clamp(config_.collective_rate_gain + step, 0.0, 1.00);
            report.recommendations.push_back(
                "collective_rate_gain " + fixed2(config_.collective_rate_gain) + " -> " +
                fixed2(*report.recommended_collective_rate_gain) +
                " because collective transients still leave hRate rms=" +
                fixed3(report.collective_transient_rms));
        } else {
            report.recommendations.push_back("collective_rate_gain looks close in collective transients");
        }
    } else {
        report.recommendations.push_back("collective_rate_gain needs several centered collective up/down moves");
    }

    if (normal_seconds_ >= 4.0) {
        if (report.oscillation_rate >= 0.45 && report.heading_rate_rms >= 0.025) {
            report.recommended_kp = clamp(config_.kp * 0.90, 0.0, 10.0);
            report.recommendations.push_back(
                "kp " + fixed2(config_.kp) + " -> " + fixed2(*report.recommended_kp) +
                " because centered hold is oscillating");
        } else if (report.heading_rate_rms >= 0.07 && report.saturation_ratio < 0.10) {
            report.recommended_kp = clamp(config_.kp * 1.10, 0.0, 10.0);
            report.recommendations.push_back(
                "kp " + fixed2(config_.kp) + " -> " + fixed2(*report.recommended_kp) +
                " because centered yaw-rate damping is slow");
        } else if (report.heading_rate_rms >= 0.07 && report.saturation_ratio >= 0.10) {
            report.recommended_heading_hold_max_assist =
                clamp(config_.heading_hold_max_assist + 0.05, 0.0, 1.0);
            report.recommendations.push_back(
                "heading_hold_max_assist " + fixed2(config_.heading_hold_max_assist) + " -> " +
                fixed2(*report.recommended_heading_hold_max_assist) +
                " only if the segment was not VRS or loss of control");
        } else {
            report.recommendations.push_back("kp and heading_hold_max_assist look acceptable in normal hold");
        }
    } else {
        report.recommendations.push_back("kp needs more quiet centered heading-hold samples");
    }

    if (excluded_unstable_seconds_ >= 1.0) {
        report.recommendations.push_back(
            "excluded " + fixed2(excluded_unstable_seconds_) +
            "s of unstable data; do not tune normal gains from VRS/loss-of-control segments");
    }
    return report;
}

TuneUpdate choose_tune_update(const TuneConfig& current, const TuneReport& report) {
    TuneUpdate update;
    update.config = current;
    update.message = "no parameter change";

    const auto changed_value = [](double from, double to) {
        return std::abs(from - to) >= 0.005;
    };

    if (report.recommended_collective_gain &&
        changed_value(current.collective_gain, *report.recommended_collective_gain)) {
        update.config.collective_gain = *report.recommended_collective_gain;
        update.changed = true;
        update.message =
            "collective_gain " + fixed2(current.collective_gain) + " -> " +
            fixed2(update.config.collective_gain);
        return update;
    }

    if (report.recommended_collective_rate_gain &&
        changed_value(current.collective_rate_gain, *report.recommended_collective_rate_gain)) {
        update.config.collective_rate_gain = *report.recommended_collective_rate_gain;
        update.changed = true;
        update.message =
            "collective_rate_gain " + fixed2(current.collective_rate_gain) + " -> " +
            fixed2(update.config.collective_rate_gain);
        return update;
    }

    if (report.recommended_kp && changed_value(current.kp, *report.recommended_kp)) {
        update.config.kp = *report.recommended_kp;
        update.changed = true;
        update.message = "kp " + fixed2(current.kp) + " -> " + fixed2(update.config.kp);
        return update;
    }

    if (report.recommended_heading_hold_max_assist &&
        changed_value(current.heading_hold_max_assist, *report.recommended_heading_hold_max_assist)) {
        update.config.heading_hold_max_assist = *report.recommended_heading_hold_max_assist;
        update.changed = true;
        update.message =
            "heading_hold_max_assist " + fixed2(current.heading_hold_max_assist) + " -> " +
            fixed2(update.config.heading_hold_max_assist);
        return update;
    }

    return update;
}

}  // namespace autorudder
