#pragma once

#include <optional>
#include <string>
#include <vector>

namespace autorudder {

struct TuneConfig {
    double kp = 0.0;
    double heading_hold_max_assist = 0.0;
    double collective_gain = 0.0;
    double collective_rate_gain = 0.0;
    double collective_sign = -1.0;
};

struct TuneSample {
    double dt = 0.01;
    double pedal = 0.0;
    double heading_rate = 0.0;
    double heading_error = 0.0;
    double collective = 0.0;
    double collective_rate = 0.0;
    double collective_feedforward = 0.0;
    double final_rudder = 0.0;
    bool fresh = false;
    bool aircraft_is_ah64 = false;
    bool input_valid = false;
    bool heading_valid = false;
    bool collective_valid = false;
    bool heading_hold_mode = false;
};

struct TuneReport {
    int raw_samples = 0;
    double usable_seconds = 0.0;
    double excluded_unstable_seconds = 0.0;
    double normal_seconds = 0.0;
    double heading_rate_rms = 0.0;
    double heading_rate_peak = 0.0;
    double final_abs_mean = 0.0;
    double saturation_ratio = 0.0;
    double oscillation_rate = 0.0;
    double static_collective_seconds = 0.0;
    double static_feedback_mean = 0.0;
    double collective_transient_seconds = 0.0;
    double collective_transient_rms = 0.0;
    double collective_transient_peak = 0.0;
    std::optional<double> recommended_collective_gain;
    std::optional<double> recommended_collective_rate_gain;
    std::optional<double> recommended_kp;
    std::optional<double> recommended_heading_hold_max_assist;
    std::vector<std::string> recommendations;
};

struct TuneUpdate {
    TuneConfig config;
    bool changed = false;
    std::string message;
};

class TuneSessionAnalyzer {
public:
    explicit TuneSessionAnalyzer(TuneConfig config);

    void add(const TuneSample& sample);
    TuneReport report() const;
    void reset();

private:
    TuneConfig config_;
    int raw_samples_ = 0;
    double usable_seconds_ = 0.0;
    double excluded_unstable_seconds_ = 0.0;
    double normal_seconds_ = 0.0;
    double heading_rate_sq_seconds_ = 0.0;
    double heading_rate_peak_ = 0.0;
    double final_abs_seconds_ = 0.0;
    double saturation_seconds_ = 0.0;
    int heading_rate_sign_changes_ = 0;
    int last_heading_rate_sign_ = 0;
    double static_collective_seconds_ = 0.0;
    double static_feedback_seconds_ = 0.0;
    double static_gain_delta_seconds_ = 0.0;
    double collective_transient_seconds_ = 0.0;
    double collective_transient_sq_seconds_ = 0.0;
    double collective_transient_peak_ = 0.0;
};

TuneUpdate choose_tune_update(const TuneConfig& current, const TuneReport& report);

}  // namespace autorudder
