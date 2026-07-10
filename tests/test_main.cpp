#include "config.h"
#include "collective_drive.h"
#include "dcs_bios_protocol.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
#include "f14_roll_assist.h"
#include "power_feedforward.h"
#include "retro_toggle.h"
#include "retro_music_guard.h"
#include "retro_xm_player.h"
#include "rudder_input.h"
#include "tune_session.h"
#include "yaw_damper.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

int failures = 0;

void expect_true(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void expect_near(double actual, double expected, double epsilon, const std::string& message) {
    if (std::abs(actual - expected) > epsilon) {
        ++failures;
        std::cerr << "FAIL: " << message << " actual=" << actual << " expected=" << expected << '\n';
    }
}

void test_protocol_parser_applies_writes() {
    autorudder::DcsBiosState state(2048);
    int frames = 0;
    autorudder::DcsBiosProtocolParser parser;
    parser.set_frame_callback([&] { ++frames; });
    parser.set_write_callback([&](const autorudder::BiosWrite& write) { state.apply_write(write); });

    const std::vector<std::uint8_t> packet = {
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x04, 0x00, 'A', 'H', 0, 0,
        0x5e, 0x04, 0x02, 0x00, 0x34, 0x12,
    };
    parser.feed(packet.data(), 5);
    parser.feed(packet.data() + 5, packet.size() - 5);

    expect_true(frames == 1, "parser detects one frame");
    expect_true(state.read_string(0x0400, 4) == "AH", "string write is applied");
    expect_true(state.read_u16(0x045e).value_or(0) == 0x1234, "integer write is applied");
}

void test_protocol_parser_handles_multiple_frames() {
    autorudder::DcsBiosState state(2048);
    int frames = 0;
    autorudder::DcsBiosProtocolParser parser;
    parser.set_frame_callback([&] { ++frames; });
    parser.set_write_callback([&](const autorudder::BiosWrite& write) { state.apply_write(write); });

    const std::vector<std::uint8_t> packet = {
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x02, 0x00, 'A', 0,
        0x55, 0x55, 0x55, 0x55,
        0x00, 0x04, 0x02, 0x00, 'B', 0,
    };
    parser.feed(packet.data(), packet.size());

    expect_true(frames == 2, "parser detects repeated frame sync");
    expect_true(state.read_string(0x0400, 2) == "B", "second frame write is applied");
}

void test_ref_extraction() {
    const std::string json = R"JSON(
{
  "Speed": {
    "ANGULAR_VELOCITY_Z": {
      "identifier": "ANGULAR_VELOCITY_Z",
      "outputs": [{"address": 1118, "max_length": 6, "type": "string"}]
    }
  },
  "Gauge": {
    "PLT_SAI_BALL": {
      "identifier": "PLT_SAI_BALL",
      "outputs": [{"address": 32828, "mask": 65535, "shift_by": 0, "max_value": 65535, "type": "integer"}]
    }
  }
}
)JSON";
    const auto yaw = autorudder::extract_output_ref(json, "ANGULAR_VELOCITY_Z");
    const auto ball = autorudder::extract_output_ref(json, "PLT_SAI_BALL");
    expect_true(yaw.has_value(), "yaw ref found");
    expect_true(ball.has_value(), "ball ref found");
    expect_true(yaw->address == 1118, "yaw address parsed");
    expect_true(yaw->max_length == 6, "yaw max length parsed");
    expect_true(ball->address == 32828, "ball address parsed");
}

void test_config_loads_yaw_rate_hold_fields() {
    const auto path = std::filesystem::current_path() / "autorudder_test_config.ini";
    std::filesystem::remove(path);
    {
        std::ofstream out(path);
        out << "yaw_rate_error_exponent=0.75\n"
            << "yaw_rate_source=heading\n"
            << "yaw_rate_integral_gain=0.14\n"
            << "yaw_rate_integral_limit=0.12\n"
            << "yaw_rate_integral_leak=0.35\n"
            << "yaw_rate_integral_deadband=0.012\n";
    }

    const auto cfg = autorudder::load_config(path);
    std::filesystem::remove(path);

    expect_near(cfg.yaw_rate_error_exponent, 0.75, 0.001, "config loads yaw-rate error exponent");
    expect_true(cfg.yaw_rate_source == "heading", "config loads yaw-rate source");
    expect_near(cfg.yaw_rate_integral_gain, 0.14, 0.001, "config loads yaw-rate integral gain");
    expect_near(cfg.yaw_rate_integral_limit, 0.12, 0.001, "config loads yaw-rate integral limit");
    expect_near(cfg.yaw_rate_integral_leak, 0.35, 0.001, "config loads yaw-rate integral leak");
    expect_near(cfg.yaw_rate_integral_deadband, 0.012, 0.001, "config loads yaw-rate integral deadband");
}

void test_config_loads_ah64_roll_fields() {
    const auto path = std::filesystem::current_path() / "autorudder_test_config.ini";
    std::filesystem::remove(path);
    {
        std::ofstream out(path);
        out << "ah64_roll_enabled=1\n"
            << "ah64_roll_input_id=2\n"
            << "ah64_roll_device_name_contains=Warthog\n"
            << "ah64_roll_axis_name=RX\n"
            << "ah64_roll_output_axis_name=Y\n"
            << "ah64_roll_input_center=0.05\n"
            << "ah64_roll_input_deadzone=0.03\n"
            << "ah64_roll_input_scale=-1.20\n"
            << "ah64_roll_override_threshold=0.25\n"
            << "ah64_roll_counter_sign=-1\n"
            << "ah64_roll_counter_gain=0.22\n"
            << "ah64_roll_counter_max=0.09\n"
            << "ah64_roll_counter_deadband=0.04\n"
            << "ah64_roll_counter_fade_time=0.18\n";
    }

    const auto cfg = autorudder::load_config(path);
    std::filesystem::remove(path);

    expect_near(cfg.ah64_roll_enabled, 1.0, 0.001, "config loads AH-64D roll enabled");
    expect_true(cfg.ah64_roll_input_id == 2, "config loads AH-64D roll input id");
    expect_true(cfg.ah64_roll_device_name_contains == "Warthog", "config loads AH-64D roll device filter");
    expect_true(cfg.ah64_roll_axis_name == "RX", "config loads AH-64D roll input axis");
    expect_true(cfg.ah64_roll_output_axis_name == "Y", "config loads AH-64D roll output axis");
    expect_near(cfg.ah64_roll_input_center, 0.05, 0.001, "config loads AH-64D roll center");
    expect_near(cfg.ah64_roll_input_deadzone, 0.03, 0.001, "config loads AH-64D roll deadzone");
    expect_near(cfg.ah64_roll_input_scale, -1.20, 0.001, "config loads AH-64D roll scale");
    expect_near(cfg.ah64_roll_override_threshold, 0.25, 0.001, "config loads AH-64D roll override threshold");
    expect_near(cfg.ah64_roll_counter_sign, -1.0, 0.001, "config loads AH-64D roll counter sign");
    expect_near(cfg.ah64_roll_counter_gain, 0.22, 0.001, "config loads AH-64D roll counter gain");
    expect_near(cfg.ah64_roll_counter_max, 0.09, 0.001, "config loads AH-64D roll counter max");
    expect_near(cfg.ah64_roll_counter_deadband, 0.04, 0.001, "config loads AH-64D roll counter deadband");
    expect_near(cfg.ah64_roll_counter_fade_time, 0.18, 0.001, "config loads AH-64D roll counter fade time");
}

void test_config_loads_power_feedforward_fields() {
    const auto path = std::filesystem::current_path() / "autorudder_test_config.ini";
    std::filesystem::remove(path);
    {
        std::ofstream out(path);
        out << "power_feedforward_source=fuel_rpm\n"
            << "fuel_flow_min=0.050\n"
            << "fuel_flow_max=0.150\n"
            << "rpm_nominal=100.0\n"
            << "rpm_drop_full_scale=8.0\n"
            << "rpm_power_gain=0.40\n"
            << "power_proxy_rise_rate_limit=1.20\n"
            << "power_proxy_fall_rate_limit=0.025\n"
            << "power_collective_lead_gain=0.35\n"
            << "power_collective_lead_invert=1\n"
            << "power_collective_lead_deadband=0.02\n";
    }

    const auto cfg = autorudder::load_config(path);
    std::filesystem::remove(path);

    expect_true(cfg.power_feedforward_source == "fuel_rpm", "config loads power feedforward source");
    expect_near(cfg.fuel_flow_min, 0.050, 0.001, "config loads fuel flow min");
    expect_near(cfg.fuel_flow_max, 0.150, 0.001, "config loads fuel flow max");
    expect_near(cfg.rpm_nominal, 100.0, 0.001, "config loads rpm nominal");
    expect_near(cfg.rpm_drop_full_scale, 8.0, 0.001, "config loads rpm full scale");
    expect_near(cfg.rpm_power_gain, 0.40, 0.001, "config loads rpm power gain");
    expect_near(cfg.power_proxy_rise_rate_limit, 1.20, 0.001, "config loads power proxy rise limit");
    expect_near(cfg.power_proxy_fall_rate_limit, 0.025, 0.001, "config loads power proxy fall limit");
    expect_near(cfg.power_collective_lead_gain, 0.35, 0.001, "config loads collective lead gain");
    expect_near(cfg.power_collective_lead_invert, 1.0, 0.001, "config loads collective lead invert");
    expect_near(cfg.power_collective_lead_deadband, 0.02, 0.001, "config loads collective lead deadband");
}

void test_power_feedforward_collective_source_passthrough() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "collective";
    autorudder::PowerFeedforwardInput input;
    input.collective = 0.42;

    const auto output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "collective", "collective power source is reported");
    expect_true(output.value.has_value(), "collective power source produces a value");
    expect_near(*output.value, 0.42, 0.001, "collective power source passes through input");
}

void test_power_feedforward_fuel_flow_normalizes() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "fuel_flow";
    cfg.fuel_flow_min = 0.050;
    cfg.fuel_flow_max = 0.150;
    autorudder::PowerFeedforwardInput input;
    input.fuel_flow = 0.100;

    const auto output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_flow", "fuel flow power source is reported");
    expect_true(output.value.has_value(), "fuel flow power source produces a value");
    expect_near(*output.value, 0.50, 0.001, "fuel flow source normalizes to unit range");
}

void test_power_feedforward_fuel_rpm_adds_rpm_drop() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "fuel_rpm";
    cfg.fuel_flow_min = 0.050;
    cfg.fuel_flow_max = 0.150;
    cfg.rpm_nominal = 100.0;
    cfg.rpm_drop_full_scale = 10.0;
    cfg.rpm_power_gain = 0.20;
    autorudder::PowerFeedforwardInput input;
    input.fuel_flow = 0.100;
    input.rpm = 95.0;

    const auto output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_rpm", "fuel rpm power source is reported");
    expect_true(output.value.has_value(), "fuel rpm power source produces a value");
    expect_near(*output.value, 0.60, 0.001, "rpm drop increases power proxy");
}

void test_power_feedforward_fuel_rpm_requires_fuel() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "fuel_rpm";
    autorudder::PowerFeedforwardInput input;
    input.rpm = 95.0;

    const auto output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_rpm_missing", "missing fuel is reported");
    expect_true(!output.value.has_value(), "fuel rpm source does not invent a value without fuel");
}

void test_power_feedforward_collective_lead_only_increases_proxy() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "fuel_rpm";
    cfg.fuel_flow_min = 0.050;
    cfg.fuel_flow_max = 0.150;
    cfg.rpm_nominal = 100.0;
    cfg.rpm_drop_full_scale = 10.0;
    cfg.rpm_power_gain = 0.20;
    cfg.collective_lead_gain = 0.50;
    cfg.collective_lead_invert = 1.0;
    cfg.collective_lead_deadband = 0.02;

    autorudder::PowerFeedforwardInput input;
    input.fuel_flow = 0.090;
    input.rpm = 100.0;
    input.collective = 0.30;

    auto output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_rpm_collective_lead", "collective lead is reported when it raises fuel rpm proxy");
    expect_true(output.value.has_value(), "collective lead keeps a power value");
    expect_near(*output.value, 0.54, 0.001, "collective lead adds upward-only demand gap");

    input.collective = 0.90;
    output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_rpm", "collective lead is not reported when demand is below engine proxy");
    expect_true(output.value.has_value(), "engine proxy remains available when collective lead is inactive");
    expect_near(*output.value, 0.40, 0.001, "collective lead does not reduce engine proxy");
}

void test_power_feedforward_small_collective_lead_is_not_dropped_by_default() {
    autorudder::PowerFeedforwardConfig cfg;
    cfg.source = "fuel_rpm";
    cfg.fuel_flow_min = 0.050;
    cfg.fuel_flow_max = 0.150;
    cfg.rpm_power_gain = 0.0;
    cfg.collective_lead_gain = 0.35;
    cfg.collective_lead_invert = 1.0;

    autorudder::PowerFeedforwardInput input;
    input.fuel_flow = 0.100;
    input.rpm = 100.0;
    input.collective = 0.490;

    auto output = autorudder::compute_power_feedforward_input(cfg, input);

    expect_true(output.source == "fuel_rpm_collective_lead", "small inverted collective increase applies physical lead by default");
    expect_true(output.value.has_value(), "small collective lead keeps a power value");
    expect_near(*output.value, 0.5035, 0.0001, "small collective increase is preserved in the power proxy");

    input.collective = 0.510;
    output = autorudder::compute_power_feedforward_input(cfg, input);
    expect_true(output.source == "fuel_rpm", "small inverted collective decrease does not apply upward-only lead");
    expect_near(*output.value, 0.5000, 0.0001, "upward-only lead does not reduce the power proxy");
}

autorudder::AppConfig test_config() {
    autorudder::AppConfig cfg;
    cfg.control_mode = "yaw_damper";
    cfg.assist_sign = 1.0;
    cfg.kp = 0.5;
    cfg.ki = 0.0;
    cfg.integral_limit = 0.0;
    cfg.max_assist = 0.2;
    cfg.trim_capture_enabled = 0.0;
    cfg.yaw_rate_deadband = 0.0;
    cfg.pedal_override_threshold = 0.12;
    cfg.pedal_rate_override_threshold = 10.0;
    cfg.fade_in_time = 0.1;
    cfg.fade_out_time = 0.1;
    cfg.filter_time = 0.0;
    return cfg;
}

autorudder::AppConfig heading_config() {
    autorudder::AppConfig cfg;
    cfg.control_mode = "heading_hold";
    cfg.yaw_response_sign = 1.0;
    cfg.yaw_rate_sign = -1.0;
    cfg.yaw_rate_source = "auto";
    cfg.kp = 1.45;
    cfg.ki = 0.0;
    cfg.integral_limit = 0.0;
    cfg.max_assist = 0.85;
    cfg.heading_hold_max_assist = 0.35;
    cfg.release_brake_time = 1.8;
    cfg.release_brake_kp = 3.2;
    cfg.release_brake_max_assist = 0.85;
    cfg.yaw_rate_deadband = 0.0;
    cfg.heading_error_deadband = 0.0;
    cfg.heading_kp = 0.75;
    cfg.heading_rate_limit = 0.30;
    cfg.turn_rate_max = 0.45;
    cfg.pedal_command_sign = 1.0;
    cfg.pedal_command_deadzone = 0.06;
    cfg.pedal_command_exit_deadzone = 0.03;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    cfg.filter_time = 0.0;
    cfg.collective_gain = 0.0;
    cfg.collective_rate_gain = 0.0;
    return cfg;
}

autorudder::AppConfig f14_config() {
    autorudder::AppConfig cfg;
    cfg.control_mode = "f14_roll_assist";
    cfg.f14_aoa_onset_units = 13.0;
    cfg.f14_aoa_full_units = 17.0;
    cfg.f14_roll_deadzone = 0.03;
    cfg.f14_roll_washout = 0.50;
    cfg.f14_roll_to_rudder_gain = 0.50;
    cfg.f14_deep_roll_to_rudder_gain = 0.0;
    cfg.f14_low_aoa_roll_coordination_gain = 0.0;
    cfg.f14_roll_to_rudder_sign = 1.0;
    cfg.f14_yaw_rate_gain = 0.0;
    cfg.f14_slip_gain = 0.0;
    cfg.f14_rudder_max_assist = 0.60;
    cfg.f14_rudder_rate_limit = 0.0;
    cfg.f14_roll_rate_sign = 1.0;
    cfg.f14_reversal_guard_time = 0.15;
    cfg.f14_reversal_roll_threshold = 0.30;
    cfg.f14_reversal_roll_rate_threshold = 0.10;
    cfg.f14_reversal_guard_scale = 0.25;
    cfg.f14_ground_ias_threshold = 35.0;
    cfg.f14_ground_agl_threshold = 3.0;
    return cfg;
}

autorudder::F14RollAssistInput f14_valid_input() {
    autorudder::F14RollAssistInput input;
    input.dt = 0.05;
    input.physical_roll = 0.0;
    input.physical_rudder = 0.0;
    input.aoa_units = 17.0;
    input.aoa_valid = true;
    input.indicated_airspeed = 120.0;
    input.radar_altitude = 2000.0;
    input.gear_position = 0.0;
    input.flaps_position = 0.0;
    input.indicated_airspeed_valid = true;
    input.radar_altitude_valid = true;
    input.gear_valid = true;
    input.flaps_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_f14 = true;
    input.roll_input_valid = true;
    input.rudder_input_valid = true;
    input.assist_enabled = true;
    return input;
}

void test_yaw_damper_assists_centered_pedals() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.3;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.assist_offset < -0.14, "positive yaw rate commands negative assist");
    expect_true(output.final_rudder < -0.14, "final rudder includes assist");
    expect_true(output.assist_active, "assist is active");
}

void test_yaw_damper_user_override() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.3;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    for (int i = 0; i < 10; ++i) {
        damper.update(input);
    }

    input.physical_rudder = 0.5;
    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.user_override, "pedal displacement triggers override");
    expect_near(output.assist_offset, 0.0, 0.001, "assist fades to zero during override");
    expect_near(output.final_rudder, 0.5, 0.001, "override passes pedal through");
}

void test_yaw_damper_stale_or_wrong_aircraft_passes_through() {
    autorudder::YawDamper damper(test_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = -0.2;
    input.yaw_rate_z = 1.0;
    input.telemetry_fresh = false;
    input.aircraft_is_ah64 = true;

    const auto output = damper.update(input);
    expect_near(output.final_rudder, -0.2, 0.001, "stale telemetry passes through");
    expect_near(output.assist_offset, 0.0, 0.001, "stale telemetry has no assist");
}

void test_yaw_damper_integrates_centered_steady_rate() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.integral_limit = 0.3;
    cfg.max_assist = 0.5;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = -0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    autorudder::YawDamperOutput output;
    for (int i = 0; i < 50; ++i) {
        output = damper.update(input);
    }
    expect_true(output.integral_assist < -0.25, "steady yaw rate builds hold assist");
    expect_true(output.final_rudder < -0.25, "integral assist contributes to final rudder");
}

void test_yaw_damper_override_resets_integral() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.integral_limit = 0.3;
    cfg.max_assist = 0.5;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = -0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;
    for (int i = 0; i < 20; ++i) {
        damper.update(input);
    }

    input.physical_rudder = -0.5;
    autorudder::YawDamperOutput output;
    for (int i = 0; i < 10; ++i) {
        output = damper.update(input);
    }
    expect_true(output.user_override, "manual pedal still overrides with integral enabled");
    expect_near(output.integral_assist, 0.0, 0.001, "manual pedal clears hold integral");
    expect_near(output.final_rudder, -0.5, 0.001, "manual pedal passes through after integral reset");
}

void test_yaw_damper_captures_manual_trim_on_release() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.trim_capture_enabled = 1.0;
    cfg.trim_capture_min_pedal = 0.2;
    cfg.trim_capture_yaw_rate = 0.02;
    cfg.trim_capture_pedal_rate = 0.5;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.physical_rudder = -0.65;
    input.yaw_rate_z = 0.0;
    damper.update(input);
    damper.update(input);

    input.physical_rudder = 0.0;
    damper.update(input);
    const auto output = damper.update(input);

    expect_near(output.trim_bias, -0.65, 0.001, "stable manual pedal is captured as trim");
    expect_near(output.final_rudder, -0.65, 0.001, "captured trim drives centered-pedal output");
}

void test_yaw_damper_collective_feedforward() {
    autorudder::AppConfig cfg = test_config();
    cfg.assist_sign = -1.0;
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.0;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.0;
    input.collective = 0.60;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    const auto output = damper.update(input);
    expect_near(output.collective_feedforward, -0.42, 0.001, "collective feedforward commands left rudder");
    expect_near(output.final_rudder, -0.42, 0.001, "collective feedforward reaches final rudder");
}

void test_yaw_damper_collective_feedforward_visible_when_stale() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.0;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.physical_rudder = 0.10;
    input.collective = 0.60;
    input.collective_valid = true;
    input.telemetry_fresh = false;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    const auto output = damper.update(input);
    expect_near(output.collective_feedforward, -0.42, 0.001, "collective feedforward remains visible for diagnostics");
    expect_near(output.final_rudder, 0.10, 0.001, "stale telemetry still prevents feedforward from reaching final rudder");
    expect_true(output.reason == "stale telemetry", "stale telemetry gate is reported");
}

void test_yaw_damper_collective_power_feedforward() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 1.0;
    cfg.collective_sign = -1.0;
    cfg.collective_feedforward_mode = "power";
    cfg.collective_gain = 1.45;
    cfg.collective_zero_thrust = 0.0;
    cfg.collective_power_exponent = 1.5;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.01;
    input.collective = 0.50;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    const auto output = damper.update(input);
    expect_near(output.collective_feedforward, -0.513, 0.002, "power collective feedforward follows COL2YAW-style curve");
    expect_near(output.final_rudder, -0.513, 0.002, "power collective feedforward reaches rudder output");
}

void test_yaw_damper_power_proxy_fall_rate_limit_prevents_rudder_release() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 1.0;
    cfg.collective_sign = -1.0;
    cfg.collective_feedforward_mode = "linear";
    cfg.collective_gain = 1.0;
    cfg.collective_rate_gain = 0.0;
    cfg.power_proxy_fall_rate_limit = 0.05;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.collective = 0.50;
    auto output = damper.update(input);
    expect_near(output.collective, 0.50, 0.001, "initial power proxy is used directly");
    expect_near(output.final_rudder, -0.50, 0.001, "initial power proxy sets rudder feedforward");

    input.collective = 0.20;
    output = damper.update(input);
    expect_near(output.collective, 0.495, 0.001, "power proxy fall is slew-limited");
    expect_near(output.collective_feedforward, -0.495, 0.001, "fall limit keeps anti-torque feedforward");
    expect_near(output.final_rudder, -0.495, 0.001, "fall limit prevents sudden rudder release");
}

void test_yaw_damper_collective_transient_uses_fast_fade() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.45;
    cfg.collective_rate_limit = 0.40;
    cfg.collective_transient_rate_threshold = 0.20;
    cfg.collective_transient_fade_time = 0.01;
    cfg.fade_in_time = 1.0;
    cfg.fade_out_time = 1.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.01;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.collective = 0.0;
    damper.update(input);

    input.collective = 0.50;
    const auto output = damper.update(input);
    expect_true(output.collective_rate > 40.0, "collective step creates transient rate");
    expect_near(output.collective_feedforward, -0.75, 0.001, "collective step builds rate feedforward");
    expect_near(output.final_rudder, -0.75, 0.001, "transient feedforward bypasses slow normal fade");
}

void test_yaw_damper_collective_transient_holds_through_small_bounce() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.45;
    cfg.collective_rate_limit = 0.40;
    cfg.collective_transient_rate_threshold = 0.20;
    cfg.collective_rate_hold_time = 0.18;
    cfg.collective_rate_reverse_threshold = 0.35;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.01;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.collective = 0.0;
    damper.update(input);

    input.collective = 0.50;
    damper.update(input);

    input.collective = 0.497;
    auto output = damper.update(input);
    expect_near(output.collective_feedforward, -0.748, 0.002, "small collective bounce keeps the transient feedforward direction");

    input.collective = 0.40;
    output = damper.update(input);
    expect_true(output.collective_feedforward < -0.40, "large opposite collective motion does not snap to opposite rudder immediately");

    for (int i = 0; i < 12; ++i) {
        output = damper.update(input);
    }
    expect_true(output.collective_feedforward > 0.05, "sustained opposite collective motion still crosses over quickly");
}

void test_yaw_damper_collective_rate_deadband_ignores_small_motion() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.45;
    cfg.collective_rate_limit = 0.40;
    cfg.collective_transient_rate_threshold = 0.20;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.01;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.collective = 0.500;
    damper.update(input);

    input.collective = 0.501;
    const auto output = damper.update(input);

    expect_true(output.collective_rate < 0.20, "small collective motion stays below transient threshold");
    expect_near(output.collective_feedforward, -0.351, 0.001, "small collective motion does not add derivative feedforward");
}

void test_yaw_damper_trim_capture_subtracts_collective_feedforward() {
    autorudder::AppConfig cfg = test_config();
    cfg.max_assist = 0.85;
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.40;
    cfg.collective_rate_gain = 0.0;
    cfg.trim_capture_enabled = 1.0;
    cfg.trim_capture_min_pedal = 0.2;
    cfg.trim_capture_yaw_rate = 0.02;
    cfg.trim_capture_pedal_rate = 0.5;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.1;
    input.collective = 0.50;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    input.physical_rudder = -0.65;
    input.yaw_rate_z = 0.0;
    damper.update(input);
    damper.update(input);

    input.physical_rudder = 0.0;
    damper.update(input);
    const auto output = damper.update(input);

    expect_near(output.collective_feedforward, -0.20, 0.001, "collective feedforward remains active");
    expect_near(output.trim_bias, -0.45, 0.001, "captured trim stores only the remaining rudder");
    expect_near(output.final_rudder, -0.65, 0.001, "feedforward plus trim reproduces manual rudder");
}

void test_heading_hold_damps_heading_rate() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = -0.20;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.02;
    const auto output = damper.update(input);

    expect_true(output.heading_rate > 0.19, "angular velocity reports positive heading rate");
    expect_true(output.final_rudder < -0.20, "positive heading rate commands braking rudder");
    expect_true(output.yaw_rate_command < 0.0, "positive heading error commands return yaw rate");
    expect_true(output.reason == "heading hold", "centered heading mode reports heading hold");
}

void test_heading_hold_uses_angular_velocity_for_damping() {
    auto cfg = heading_config();
    cfg.yaw_rate_source = "angular";
    cfg.heading_rate_limit = 0.50;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.yaw_rate_z = -0.30;
    const auto output = damper.update(input);

    expect_near(output.heading_rate, 0.30, 0.001, "heading damping uses direct angular velocity");
    expect_true(output.final_rudder < -0.30, "direct angular velocity commands immediate braking rudder");
    expect_true(output.reason == "heading hold", "centered heading mode remains active");
}

void test_heading_hold_heading_source_ignores_body_angular_spike() {
    auto cfg = heading_config();
    cfg.yaw_rate_source = "heading";
    cfg.heading_kp = 0.0;
    cfg.yaw_rate_integral_gain = 0.0;
    cfg.yaw_accel_gain = 0.0;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.yaw_rate_z = -0.50;
    const auto output = damper.update(input);

    expect_near(output.heading_rate, 0.0, 0.001, "heading source ignores angular spike without heading change");
    expect_near(output.final_rudder, 0.0, 0.001, "ignored angular spike does not command rudder");
}

void test_heading_hold_heading_source_uses_heading_motion() {
    auto cfg = heading_config();
    cfg.yaw_rate_source = "heading";
    cfg.heading_kp = 0.0;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.03;
    const auto output = damper.update(input);

    expect_near(output.heading_rate, 0.30, 0.001, "heading source follows actual heading motion");
    expect_true(output.final_rudder < -0.25, "heading-source yaw rate commands braking rudder");
}

void test_heading_hold_rate_error_shaping_boosts_small_rates() {
    auto linear_cfg = heading_config();
    linear_cfg.kp = 1.0;
    linear_cfg.heading_hold_max_assist = 1.0;
    linear_cfg.yaw_rate_error_exponent = 1.0;

    auto shaped_cfg = linear_cfg;
    shaped_cfg.yaw_rate_error_exponent = 0.75;

    autorudder::YawDamper linear(linear_cfg);
    autorudder::YawDamper shaped(shaped_cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    linear.update(input);
    shaped.update(input);

    input.yaw_rate_z = -0.04;
    const auto linear_output = linear.update(input);
    const auto shaped_output = shaped.update(input);

    expect_true(linear_output.final_rudder < -0.035, "linear small yaw rate still damps");
    expect_true(shaped_output.final_rudder < linear_output.final_rudder - 0.02, "shaped small yaw rate gets stronger damping");
}

void test_heading_hold_rate_integral_reduces_steady_drift() {
    auto cfg = heading_config();
    cfg.kp = 0.0;
    cfg.heading_kp = 0.0;
    cfg.heading_hold_max_assist = 0.50;
    cfg.yaw_rate_integral_gain = 0.50;
    cfg.yaw_rate_integral_limit = 0.10;
    cfg.yaw_rate_integral_leak = 0.0;
    cfg.yaw_rate_integral_deadband = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = -0.05;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    autorudder::YawDamperOutput output;
    for (int i = 0; i < 50; ++i) {
        output = damper.update(input);
    }

    expect_true(output.integral_assist < -0.09, "steady heading drift builds yaw-rate hold integral");
    expect_true(output.final_rudder < -0.09, "yaw-rate hold integral reaches final rudder");
    expect_true(output.final_rudder >= -0.11, "yaw-rate hold integral remains limited");
}

void test_heading_hold_rate_integral_resets_on_turn_command() {
    auto cfg = heading_config();
    cfg.kp = 0.0;
    cfg.heading_kp = 0.0;
    cfg.heading_hold_max_assist = 0.50;
    cfg.yaw_rate_integral_gain = 0.50;
    cfg.yaw_rate_integral_limit = 0.10;
    cfg.yaw_rate_integral_leak = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = -0.05;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    for (int i = 0; i < 20; ++i) {
        damper.update(input);
    }

    input.physical_rudder = 0.20;
    input.yaw_rate_z = 0.0;
    const auto output = damper.update(input);

    expect_true(output.reason == "turn command", "manual pedal enters turn command");
    expect_near(output.integral_assist, 0.0, 0.001, "turn command clears yaw-rate hold integral");
    expect_near(output.final_rudder, 0.20, 0.001, "turn command remains direct pedal output");
}

void test_heading_hold_falls_back_when_angular_velocity_underreports() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.04;
    const auto output = damper.update(input);

    expect_true(output.heading_rate > 0.35, "heading derivative backs up an underreported angular rate");
    expect_true(output.final_rudder < -0.30, "heading-rate fallback still brakes self yaw");
}

void test_heading_hold_does_not_fallback_to_opposite_heading_derivative() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.012;
    input.yaw_rate_z = 0.03;
    const auto output = damper.update(input);

    expect_near(output.heading_rate, -0.03, 0.001, "opposite heading derivative does not replace nonzero angular rate");
    expect_true(output.final_rudder > 0.0, "controller follows the trusted angular-rate direction");
}

void test_heading_hold_rate_rescue_overrides_feedforward_bias() {
    auto cfg = heading_config();
    cfg.kp = 1.0;
    cfg.release_brake_kp = 2.0;
    cfg.heading_hold_max_assist = 0.35;
    cfg.release_brake_max_assist = 0.85;
    cfg.heading_rate_limit = 0.16;
    cfg.collective_feedforward_mode = "linear";
    cfg.collective_sign = -1.0;
    cfg.collective_gain = 0.60;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_rate_z = 0.0;
    input.collective = 0.80;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.yaw_rate_z = 0.30;
    const auto output = damper.update(input);

    expect_true(output.final_rudder > 0.05, "large self-yaw can overcome stale feedforward bias");
    expect_true(output.reason == "rate rescue", "large self-yaw enters rescue mode");
}

void test_heading_hold_rate_rescue_blends_near_threshold() {
    auto cfg = heading_config();
    cfg.yaw_rate_source = "angular";
    cfg.kp = 1.0;
    cfg.release_brake_kp = 3.0;
    cfg.heading_hold_max_assist = 0.20;
    cfg.release_brake_max_assist = 0.80;
    cfg.heading_rate_limit = 0.20;
    cfg.collective_gain = 0.0;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.yaw_rate_z = 0.205;
    const auto output = damper.update(input);

    expect_true(output.reason == "rate rescue", "near-threshold self-yaw reports rescue mode");
    expect_true(output.final_rudder < 0.25, "near-threshold rescue blends in instead of jumping to high authority");
}

void test_heading_hold_uses_yaw_acceleration_lead() {
    auto cfg = heading_config();
    cfg.yaw_accel_gain = 0.20;
    cfg.yaw_accel_filter_time = 0.0;
    cfg.yaw_accel_limit = 0.25;
    cfg.collective_gain = 0.0;
    cfg.collective_rate_gain = 0.0;
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.yaw_acceleration_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.yaw_acceleration_z = 1.0;
    const auto output = damper.update(input);

    expect_near(output.yaw_acceleration_assist, 0.20, 0.001, "yaw acceleration creates lead assist");
    expect_near(output.final_rudder, 0.20, 0.001, "lead assist reaches final rudder");
}

void test_trim_guard_clears_automatic_rudder_before_trim() {
    auto cfg = heading_config();
    cfg.fade_in_time = 0.0;
    cfg.fade_out_time = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.physical_rudder = 0.0;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.heading = 1.10;
    auto output = damper.update(input);
    expect_true(std::abs(output.final_rudder) > 0.05, "heading hold creates automatic rudder before trim");

    input.trim_guard_active = true;
    output = damper.update(input);

    expect_near(output.final_rudder, 0.0, 0.001, "trim guard outputs physical rudder only");
    expect_near(output.assist_offset, 0.0, 0.001, "trim guard clears automatic assist");
    expect_true(output.reason == "trim guard", "trim guard mode is reported");
}

void test_heading_hold_relocks_large_heading_error() {
    auto cfg = heading_config();
    cfg.heading_error_relock_threshold = 0.50;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.heading = 0.80;
    const auto output = damper.update(input);

    expect_near(output.heading_ref, 0.80, 0.001, "large heading error relocks current heading");
    expect_near(output.heading_error, 0.0, 0.001, "relock clears stale heading error");
    expect_near(output.yaw_rate_command, 0.0, 0.001, "relock stops old-heading return command");
    expect_true(output.reason == "heading relock", "large heading error reports relock mode");
}

void test_heading_hold_leaks_old_heading_error() {
    auto cfg = heading_config();
    cfg.heading_hold_leak_time = 1.0;
    cfg.heading_error_relock_threshold = 0.0;
    autorudder::YawDamper damper(cfg);

    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.heading = -0.4;
    const auto first = damper.update(input);
    const auto second = damper.update(input);

    expect_true(second.heading_error < first.heading_error, "leak reduces old-heading error");
    expect_true(second.yaw_rate_command < first.yaw_rate_command, "leak softens heading return command");
}

void test_heading_hold_recaptures_heading_after_turn_command() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.02;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = 0.5;
    input.heading = 0.4;
    auto output = damper.update(input);
    expect_true(output.yaw_rate_command > 0.10, "pedal deflection commands yaw rate");
    expect_true(output.reason == "turn command", "pedal deflection reports turn command");
    expect_near(output.final_rudder, 0.5, 0.001, "turn command maps pedal directly to output");

    input.heading = 0.4;
    input.physical_rudder = 0.0;
    input.yaw_rate_z = 0.0;
    output = damper.update(input);
    expect_near(output.heading_ref, 0.4, 0.001, "releasing pedal captures current heading");
    expect_near(output.heading_error, 0.0, 0.001, "new heading reference has no immediate error");
    expect_near(output.final_rudder, 0.0, 0.001, "release does not pull back toward old heading");
}

void test_heading_hold_uses_release_brake_after_turn_command() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = 0.1;
    input.heading = 0.1;
    damper.update(input);

    input.physical_rudder = 0.0;
    input.heading = 0.14;
    const auto output = damper.update(input);

    expect_true(output.reason == "release brake", "release from turn command enters brake mode");
    expect_true(output.final_rudder < -0.50, "release brake can exceed normal heading hold authority");
    expect_true(output.final_rudder >= -0.85, "release brake remains authority limited");
}

void test_heading_hold_release_brake_tracks_current_heading_until_settled() {
    auto cfg = heading_config();
    cfg.release_brake_time = 0.50;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = 0.4;
    input.heading = 1.2;
    damper.update(input);

    input.physical_rudder = 0.0;
    input.heading = 1.3;
    damper.update(input);

    input.heading = 1.4;
    const auto output = damper.update(input);

    expect_true(output.reason == "release brake", "release brake remains active after pedal release");
    expect_near(output.heading_ref, 1.4, 0.001, "release brake follows current heading while yaw is being arrested");
    expect_near(output.heading_error, 0.0, 0.001, "release brake does not build a spring-back heading error");
}

void test_heading_hold_release_brake_caps_total_output() {
    auto cfg = heading_config();
    cfg.collective_gain = 0.70;
    cfg.collective_sign = -1.0;
    cfg.release_brake_max_assist = 0.55;
    cfg.release_brake_kp = 3.0;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.collective = 1.0;
    input.collective_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    input.physical_rudder = 0.5;
    input.heading = 1.2;
    damper.update(input);

    input.physical_rudder = 0.0;
    input.heading = 1.28;
    const auto output = damper.update(input);

    expect_true(output.reason == "release brake", "pedal release enters release brake");
    expect_true(output.final_rudder >= -0.55 && output.final_rudder <= 0.55, "release brake caps total rudder output");
}

void test_heading_hold_allows_full_pedal_turn_output() {
    autorudder::YawDamper damper(heading_config());
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 0.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);

    input.physical_rudder = -1.0;
    input.heading = -0.1;
    const auto output = damper.update(input);

    expect_true(output.reason == "turn command", "full pedal enters turn command");
    expect_near(output.final_rudder, -1.0, 0.001, "full pedal turn is not yaw-rate limited");
}

autorudder::TuneConfig tune_config() {
    autorudder::TuneConfig cfg;
    cfg.kp = 1.45;
    cfg.heading_hold_max_assist = 0.35;
    cfg.collective_gain = 0.70;
    cfg.collective_rate_gain = 0.20;
    cfg.collective_sign = -1.0;
    return cfg;
}

void test_tune_session_recommends_collective_gain_increase() {
    const auto cfg = tune_config();
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.01;
    sample.heading_error = 0.01;
    sample.collective = 0.5;
    sample.collective_rate = 0.0;
    sample.collective_feedforward = -0.35;
    sample.final_rudder = -0.45;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.recommended_collective_gain.has_value(), "steady feedback recommends collective gain");
    expect_true(*report.recommended_collective_gain > cfg.collective_gain, "collective gain increases when feedback adds same-direction rudder");
}

void test_tune_session_uses_power_curve_for_collective_gain() {
    auto cfg = tune_config();
    cfg.collective_feedforward_mode = "power";
    cfg.collective_gain = 0.70;
    cfg.collective_zero_thrust = 0.0;
    cfg.collective_power_exponent = 1.5;
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.01;
    sample.heading_error = 0.01;
    sample.collective = 0.25;
    sample.collective_rate = 0.0;
    sample.collective_feedforward = -0.70 * std::pow(0.25, 1.5);
    sample.final_rudder = sample.collective_feedforward - 0.025;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_near(report.static_gain_fit_seconds, 6.0, 0.001, "power curve samples contribute to static gain fit");
    expect_true(report.recommended_collective_gain.has_value(), "power curve steady feedback recommends collective gain");
    expect_near(*report.recommended_collective_gain, 0.90, 0.001, "power curve gain estimate uses collective^1.5 denominator");
}

void test_tune_session_recommends_kp_increase_for_slow_damping() {
    const auto cfg = tune_config();
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.10;
    sample.heading_error = 0.02;
    sample.final_rudder = -0.10;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = false;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.recommended_kp.has_value(), "slow centered yaw damping recommends kp");
    expect_true(*report.recommended_kp > cfg.kp, "kp increases when hRate remains high without saturation");
}

void test_tune_session_ignores_scripted_collective_drive_for_kp_oscillation() {
    const auto cfg = tune_config();
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_error = 0.01;
    sample.collective = 0.45;
    sample.collective_rate = 0.0;
    sample.collective_feedforward = -0.18;
    sample.final_rudder = -0.18;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.collective_drive_active = true;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        sample.heading_rate = (i % 2 == 0) ? 0.06 : -0.06;
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.collective_drive_seconds > 5.0, "scripted collective drive time is tracked");
    expect_true(report.normal_seconds < 0.001, "scripted collective drive is not quiet normal hold");
    expect_true(!report.recommended_kp.has_value(), "scripted collective drive does not tune kp");
}

void test_tune_session_holds_collective_rate_gain_when_limited() {
    auto cfg = tune_config();
    cfg.collective_rate_gain = 0.30;
    cfg.collective_rate_limit = 0.06;
    autorudder::TuneSessionAnalyzer analyzer(cfg);

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 0.12;
    sample.heading_error = 0.01;
    sample.collective = 0.45;
    sample.collective_rate = 0.25;
    sample.collective_feedforward = -0.20;
    sample.final_rudder = -0.20;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.collective_drive_active = true;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.collective_rate_limited_ratio > 0.95, "collective rate limiter usage is tracked");
    expect_true(
        !report.recommended_collective_rate_gain.has_value(),
        "rate gain is not increased when the rate term is already limited");
}

void test_tune_session_excludes_unstable_segments() {
    autorudder::TuneSessionAnalyzer analyzer(tune_config());

    autorudder::TuneSample sample;
    sample.dt = 0.1;
    sample.pedal = 0.0;
    sample.heading_rate = 2.0;
    sample.heading_error = 1.2;
    sample.final_rudder = -0.35;
    sample.fresh = true;
    sample.aircraft_is_ah64 = true;
    sample.input_valid = true;
    sample.heading_valid = true;
    sample.collective_valid = true;
    sample.heading_hold_mode = true;
    sample.closed_loop_heading_hold = true;

    for (int i = 0; i < 60; ++i) {
        analyzer.add(sample);
    }

    const auto report = analyzer.report();
    expect_true(report.usable_seconds < 0.001, "unstable segments are not usable tune data");
    expect_true(report.excluded_unstable_seconds > 5.0, "unstable segment time is reported");
    expect_true(!report.recommended_heading_hold_max_assist.has_value(), "unstable saturation does not recommend more authority");
}

void test_tune_update_applies_all_recommendations() {
    const auto cfg = tune_config();
    autorudder::TuneReport report;
    report.recommended_collective_gain = 0.85;
    report.recommended_collective_rate_gain = 0.30;
    report.recommended_kp = 1.60;

    const auto update = autorudder::choose_tune_update(cfg, report);

    expect_true(update.changed, "tune update applies changes");
    expect_near(update.config.collective_gain, 0.85, 0.001, "collective gain changes");
    expect_near(update.config.collective_rate_gain, 0.30, 0.001, "collective rate gain changes");
    expect_near(update.config.kp, 1.60, 0.001, "kp changes");
}

void test_tune_update_applies_kp_when_feedforward_has_no_change() {
    const auto cfg = tune_config();
    autorudder::TuneReport report;
    report.recommended_kp = 1.60;

    const auto update = autorudder::choose_tune_update(cfg, report);

    expect_true(update.changed, "tune update applies kp change");
    expect_near(update.config.kp, 1.60, 0.001, "kp changes when higher-priority stages have no update");
    expect_near(update.config.collective_gain, cfg.collective_gain, 0.001, "collective gain is held");
}

void test_collective_drive_passthrough_then_scripted_steps() {
    autorudder::CollectiveDriveConfig cfg;
    cfg.amplitude = 0.10;
    cfg.period = 0.50;
    cfg.settle_time = 0.50;
    cfg.rate_limit = 0.20;
    autorudder::CollectiveDrive drive(cfg);

    auto output = drive.update(0.1, 0.50, false);
    expect_near(output.collective, 0.50, 0.001, "disabled collective drive passes physical collective through");
    expect_true(!output.driving, "disabled collective drive is not driving");

    output = drive.update(0.25, 0.50, true);
    expect_near(output.collective, 0.50, 0.001, "settle time holds physical collective");
    expect_true(!output.driving, "settle time is not driving");

    output = drive.update(0.25, 0.50, true);
    expect_true(output.driving, "collective drive starts after settle time");
    expect_near(output.collective, 0.55, 0.001, "collective drive rate-limits toward the positive step");

    output = drive.update(0.25, 0.50, true);
    expect_near(output.collective, 0.50, 0.001, "collective drive alternates toward the negative step");
}

void test_yaw_damper_set_config_preserves_heading_reference() {
    auto cfg = heading_config();
    cfg.kp = 1.0;
    autorudder::YawDamper damper(cfg);
    autorudder::YawDamperInput input;
    input.dt = 0.10;
    input.heading = 1.0;
    input.heading_valid = true;
    input.telemetry_fresh = true;
    input.aircraft_is_ah64 = true;
    input.input_valid = true;
    input.assist_enabled = true;

    damper.update(input);
    cfg.kp = 2.0;
    damper.set_config(cfg);
    input.heading = 1.10;
    const auto output = damper.update(input);

    expect_true(output.heading_error < -0.09, "heading reference survives config update");
    expect_true(output.final_rudder < -0.30, "updated kp is used without resetting damper state");
}

void test_f14_low_aoa_passes_roll_and_ignores_airborne_rudder() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.aoa_units = 10.0;
    input.physical_roll = 0.70;
    input.physical_rudder = -0.20;

    const auto output = assist.update(input);

    expect_near(output.aoa_weight, 0.0, 0.001, "low AoA has no high-AoA weight");
    expect_near(output.final_roll, 0.70, 0.001, "low AoA roll passes through");
    expect_near(output.final_rudder, 0.0, 0.001, "airborne physical rudder is ignored");
    expect_true(output.flight_mode == "NORMAL_FLIGHT", "low AoA reports normal flight mode");
}

void test_f14_high_aoa_mixes_roll_into_rudder_and_suppresses_roll() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.physical_roll = 0.80;
    input.physical_rudder = 0.10;

    const auto output = assist.update(input);

    expect_near(output.aoa_weight, 1.0, 0.001, "high AoA reaches full mix weight");
    expect_near(output.roll_scale, 0.50, 0.001, "high AoA reduces roll output");
    expect_near(output.final_roll, 0.40, 0.001, "roll output is scaled down");
    expect_near(output.rudder_from_roll, 0.40, 0.001, "roll demand creates rudder assist");
    expect_near(output.final_rudder, 0.40, 0.001, "airborne output uses automatic rudder only");
    expect_true(output.flight_mode == "HIGH_AOA_COMBAT", "high AoA reports combat mode");
}

void test_f14_ground_mode_passes_rudder_through() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.physical_roll = -0.40;
    input.physical_rudder = 0.35;
    input.indicated_airspeed = 5.0;
    input.radar_altitude = 0.5;

    const auto output = assist.update(input);

    expect_near(output.final_roll, -0.40, 0.001, "ground mode roll passes through");
    expect_near(output.final_rudder, 0.35, 0.001, "ground mode rudder passes through");
    expect_true(output.flight_mode == "GROUND", "ground mode is reported");
}

void test_f14_landing_mode_keeps_roll_and_ignores_physical_rudder() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.aoa_units = 17.0;
    input.physical_roll = 0.80;
    input.physical_rudder = 0.40;
    input.gear_position = 1.0;

    const auto output = assist.update(input);

    expect_near(output.final_roll, 0.80, 0.001, "landing mode does not suppress roll");
    expect_near(output.rudder_from_roll, 0.0, 0.001, "landing mode disables strong roll-to-rudder mix");
    expect_near(output.final_rudder, 0.0, 0.001, "landing mode ignores physical rudder in flight");
    expect_true(output.flight_mode == "LANDING", "landing mode is reported");
}

void test_f14_stale_telemetry_immediately_passes_through() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.physical_roll = 0.80;
    assist.update(input);

    input.telemetry_fresh = false;
    input.physical_roll = -0.30;
    input.physical_rudder = 0.25;
    const auto output = assist.update(input);

    expect_near(output.final_roll, -0.30, 0.001, "stale telemetry roll passes through");
    expect_near(output.final_rudder, 0.0, 0.001, "stale telemetry centers rudder without residual assist");
    expect_near(output.rudder_assist, 0.0, 0.001, "stale telemetry clears rudder assist state");
    expect_true(output.reason == "stale telemetry", "stale telemetry gate is reported");
}

void test_f14_reversal_guard_reduces_roll_to_rudder_gain() {
    autorudder::F14RollAssist assist(f14_config());
    auto input = f14_valid_input();
    input.physical_roll = 0.80;
    input.roll_rate_x = -0.20;
    input.roll_rate_valid = true;

    assist.update(input);
    assist.update(input);
    const auto output = assist.update(input);

    expect_near(output.reversal_guard, 0.25, 0.001, "sustained opposite roll rate engages guard");
    expect_near(output.rudder_from_roll, 0.10, 0.001, "guard reduces roll-to-rudder command");
    expect_near(output.final_rudder, 0.10, 0.001, "guarded command reaches rudder output");
}

void test_retro_toggle_debounces_stop_before_restart() {
    autorudder::RetroToggleController toggle(1000);

    auto action = toggle.click_profile(autorudder::RetroProfile::Ah64d, 0);
    expect_true(action.kind == autorudder::RetroToggleActionKind::Start, "first AH-64D click starts worker");
    expect_true(action.profile == autorudder::RetroProfile::Ah64d, "first click starts AH-64D");

    toggle.mark_running(autorudder::RetroProfile::Ah64d);
    action = toggle.click_profile(autorudder::RetroProfile::Ah64d, 10);
    expect_true(action.kind == autorudder::RetroToggleActionKind::Stop, "second AH-64D click requests stop");

    action = toggle.click_profile(autorudder::RetroProfile::Ah64d, 20);
    expect_true(action.kind == autorudder::RetroToggleActionKind::WaitForRelease, "repeated stop click while stopping is ignored");

    toggle.mark_stopped(30);
    action = toggle.click_profile(autorudder::RetroProfile::Ah64d, 500);
    expect_true(action.kind == autorudder::RetroToggleActionKind::WaitForRelease, "rapid click after stop does not restart vJoy");

    action = toggle.click_profile(autorudder::RetroProfile::Ah64d, 1031);
    expect_true(action.kind == autorudder::RetroToggleActionKind::Start, "click after release cooldown can restart");
}

void test_retro_music_guard_stops_for_dcs_and_profile_start() {
    autorudder::RetroMusicGuard guard;

    auto action = guard.update(true, false);
    expect_true(action == autorudder::RetroMusicAction::None, "music keeps playing while DCS is absent");

    action = guard.update(true, true);
    expect_true(action == autorudder::RetroMusicAction::StopForDcs, "music stops when DCS is detected");

    action = guard.update(false, true);
    expect_true(action == autorudder::RetroMusicAction::None, "stopped music does not repeatedly stop for DCS");

    autorudder::RetroMusicGuard start_guard;
    action = start_guard.profile_start(true);
    expect_true(action == autorudder::RetroMusicAction::StopForProfileStart, "music stops when a profile starts");

    action = start_guard.profile_start(false);
    expect_true(action == autorudder::RetroMusicAction::None, "profile start does nothing when music is already off");
}

void test_retro_sfx_click_toggles_and_blocks_restart() {
    auto action = autorudder::retro_sfx_click(true, false, false);
    expect_true(action == autorudder::RetroSfxAction::Stop, "SFX ON click stops music");

    action = autorudder::retro_sfx_click(false, false, false);
    expect_true(action == autorudder::RetroSfxAction::Start, "SFX OFF click starts music when idle");

    action = autorudder::retro_sfx_click(false, true, false);
    expect_true(action == autorudder::RetroSfxAction::None, "SFX OFF click stays off while DCS is running");

    action = autorudder::retro_sfx_click(false, false, true);
    expect_true(action == autorudder::RetroSfxAction::None, "SFX OFF click stays off while a profile is active");
}

std::vector<std::uint8_t> make_minimal_xm_module() {
    std::vector<std::uint8_t> data(60 + 276, 0);
    const auto put_text = [&](std::size_t offset, std::string_view text, std::size_t width) {
        for (std::size_t i = 0; i < text.size() && i < width; ++i) {
            data[offset + i] = static_cast<std::uint8_t>(text[i]);
        }
    };
    const auto put_u16 = [&](std::size_t offset, std::uint16_t value) {
        data[offset] = static_cast<std::uint8_t>(value & 0xFF);
        data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    };
    const auto put_u32 = [&](std::size_t offset, std::uint32_t value) {
        data[offset] = static_cast<std::uint8_t>(value & 0xFF);
        data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
        data[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
        data[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
    };

    put_text(0, "Extended Module: ", 17);
    put_text(17, "minimal test xm", 20);
    data[37] = 0x1A;
    put_text(38, "autorudder tests", 20);
    put_u16(58, 0x0104);
    put_u32(60, 276);
    put_u16(64, 1);
    put_u16(66, 0);
    put_u16(68, 1);
    put_u16(70, 1);
    put_u16(72, 0);
    put_u16(74, 1);
    put_u16(76, 6);
    put_u16(78, 125);

    const std::size_t pattern = data.size();
    data.resize(pattern + 9, 0);
    put_u32(pattern, 9);
    data[pattern + 4] = 0;
    put_u16(pattern + 5, 1);
    put_u16(pattern + 7, 0);
    return data;
}

void test_retro_xm_parser_loads_user_module() {
    const auto bytes = make_minimal_xm_module();

    const auto info = autorudder::parse_xm_module_info(bytes);
    expect_true(info.title == "minimal test xm", "XM parser reads module title");
    expect_true(info.channels == 1, "XM parser reads channel count");
    expect_true(info.patterns == 1, "XM parser reads pattern count");
    expect_true(info.instruments == 0, "XM parser reads instrument count");
    expect_true(info.default_tempo == 6, "XM parser reads default tempo");
    expect_true(info.default_bpm == 125, "XM parser reads default BPM");
}

void test_retro_xm_renderer_outputs_wav_memory() {
    const auto bytes = make_minimal_xm_module();

    const auto wav = autorudder::render_xm_to_wav(bytes, 8000, 1);
    expect_true(wav.size() > 44, "XM renderer produces WAV bytes");
    expect_true(std::string(reinterpret_cast<const char*>(wav.data()), 4) == "RIFF", "XM renderer writes RIFF header");
    expect_true(std::string(reinterpret_cast<const char*>(wav.data() + 8), 4) == "WAVE", "XM renderer writes WAVE header");
}

void test_rudder_input_calibration_recenters_offset_vjoy_axis() {
    autorudder::AppConfig cfg;
    cfg.rudder_input_center = -0.45;
    cfg.rudder_input_deadzone = 0.05;
    cfg.rudder_input_scale = 1.0;

    expect_near(autorudder::apply_rudder_input_calibration(-0.45, cfg), 0.0, 1e-9, "offset center maps to zero");
    expect_near(autorudder::apply_rudder_input_calibration(-0.43, cfg), 0.0, 1e-9, "deadzone suppresses small center drift");
    expect_near(autorudder::apply_rudder_input_calibration(-1.0, cfg), -1.0, 1e-9, "left endpoint remains full scale");
    expect_near(autorudder::apply_rudder_input_calibration(1.0, cfg), 1.0, 1e-9, "right endpoint remains full scale");
}

}  // namespace

int main() {
    test_protocol_parser_applies_writes();
    test_protocol_parser_handles_multiple_frames();
    test_ref_extraction();
    test_config_loads_yaw_rate_hold_fields();
    test_config_loads_ah64_roll_fields();
    test_config_loads_power_feedforward_fields();
    test_power_feedforward_collective_source_passthrough();
    test_power_feedforward_fuel_flow_normalizes();
    test_power_feedforward_fuel_rpm_adds_rpm_drop();
    test_power_feedforward_fuel_rpm_requires_fuel();
    test_power_feedforward_collective_lead_only_increases_proxy();
    test_power_feedforward_small_collective_lead_is_not_dropped_by_default();
    test_yaw_damper_assists_centered_pedals();
    test_yaw_damper_user_override();
    test_yaw_damper_stale_or_wrong_aircraft_passes_through();
    test_yaw_damper_integrates_centered_steady_rate();
    test_yaw_damper_override_resets_integral();
    test_yaw_damper_captures_manual_trim_on_release();
    test_yaw_damper_collective_feedforward();
    test_yaw_damper_collective_feedforward_visible_when_stale();
    test_yaw_damper_collective_power_feedforward();
    test_yaw_damper_power_proxy_fall_rate_limit_prevents_rudder_release();
    test_yaw_damper_collective_transient_uses_fast_fade();
    test_yaw_damper_collective_transient_holds_through_small_bounce();
    test_yaw_damper_collective_rate_deadband_ignores_small_motion();
    test_yaw_damper_trim_capture_subtracts_collective_feedforward();
    test_heading_hold_damps_heading_rate();
    test_heading_hold_uses_angular_velocity_for_damping();
    test_heading_hold_heading_source_ignores_body_angular_spike();
    test_heading_hold_heading_source_uses_heading_motion();
    test_heading_hold_rate_error_shaping_boosts_small_rates();
    test_heading_hold_rate_integral_reduces_steady_drift();
    test_heading_hold_rate_integral_resets_on_turn_command();
    test_heading_hold_falls_back_when_angular_velocity_underreports();
    test_heading_hold_does_not_fallback_to_opposite_heading_derivative();
    test_heading_hold_rate_rescue_overrides_feedforward_bias();
    test_heading_hold_rate_rescue_blends_near_threshold();
    test_heading_hold_uses_yaw_acceleration_lead();
    test_trim_guard_clears_automatic_rudder_before_trim();
    test_heading_hold_relocks_large_heading_error();
    test_heading_hold_leaks_old_heading_error();
    test_heading_hold_recaptures_heading_after_turn_command();
    test_heading_hold_uses_release_brake_after_turn_command();
    test_heading_hold_release_brake_tracks_current_heading_until_settled();
    test_heading_hold_release_brake_caps_total_output();
    test_heading_hold_allows_full_pedal_turn_output();
    test_tune_session_recommends_collective_gain_increase();
    test_tune_session_uses_power_curve_for_collective_gain();
    test_tune_session_recommends_kp_increase_for_slow_damping();
    test_tune_session_ignores_scripted_collective_drive_for_kp_oscillation();
    test_tune_session_holds_collective_rate_gain_when_limited();
    test_tune_session_excludes_unstable_segments();
    test_tune_update_applies_all_recommendations();
    test_tune_update_applies_kp_when_feedforward_has_no_change();
    test_collective_drive_passthrough_then_scripted_steps();
    test_yaw_damper_set_config_preserves_heading_reference();
    test_f14_low_aoa_passes_roll_and_ignores_airborne_rudder();
    test_f14_high_aoa_mixes_roll_into_rudder_and_suppresses_roll();
    test_f14_ground_mode_passes_rudder_through();
    test_f14_landing_mode_keeps_roll_and_ignores_physical_rudder();
    test_f14_stale_telemetry_immediately_passes_through();
    test_f14_reversal_guard_reduces_roll_to_rudder_gain();
    test_retro_toggle_debounces_stop_before_restart();
    test_retro_music_guard_stops_for_dcs_and_profile_start();
    test_retro_sfx_click_toggles_and_blocks_restart();
    test_retro_xm_parser_loads_user_module();
    test_retro_xm_renderer_outputs_wav_memory();
    test_rudder_input_calibration_recenters_offset_vjoy_axis();

    if (failures != 0) {
        std::cerr << failures << " test failure(s)\n";
        return EXIT_FAILURE;
    }
    std::cout << "All tests passed\n";
    return EXIT_SUCCESS;
}
