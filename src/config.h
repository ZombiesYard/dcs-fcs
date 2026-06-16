#pragma once

#include <filesystem>
#include <string>

namespace autorudder {

struct AppConfig {
    std::filesystem::path dcs_bios_path =
        R"(C:\Users\15423\Saved Games\DCS\Scripts\DCS-BIOS)";
    std::string multicast_address = "239.255.50.10";
    std::string multicast_interface = "127.0.0.1";
    int udp_port = 5010;
    std::string telemetry_source = "fast_export";
    std::string fast_export_bind_address = "127.0.0.1";
    int fast_export_port = 34380;

    int input_vjoy_id = 1;
    int output_vjoy_id = 2;
    std::string input_device_name_contains = "vJoy";
    std::string axis_name = "X";

    std::string control_mode = "heading_hold";
    double assist_sign = -1.0;
    double yaw_response_sign = 1.0;
    double yaw_rate_sign = -1.0;
    double kp = 1.45;
    double ki = 0.0;
    double integral_limit = 0.0;
    double max_assist = 0.85;
    double heading_hold_max_assist = 0.35;
    double release_brake_time = 1.80;
    double release_brake_kp = 3.20;
    double release_brake_max_assist = 0.85;
    double yaw_rate_deadband = 0.010;
    double heading_error_deadband = 0.015;
    double heading_kp = 0.75;
    double heading_rate_limit = 0.30;
    double turn_rate_max = 0.45;
    double pedal_command_sign = 1.0;
    double pedal_command_deadzone = 0.06;
    double pedal_command_exit_deadzone = 0.03;
    double pedal_override_threshold = 0.12;
    double pedal_rate_override_threshold = 1.0;
    double trim_capture_enabled = 0.0;
    double trim_capture_min_pedal = 0.20;
    double trim_capture_yaw_rate = 0.025;
    double trim_capture_pedal_rate = 0.50;
    std::string collective_source = "fast_export";
    int collective_input_id = 1;
    std::string collective_device_name_contains;
    std::string collective_axis_name = "Z";
    double collective_invert = 0.0;
    double collective_sign = -1.0;
    double collective_gain = 0.70;
    double collective_rate_gain = 0.20;
    double collective_rate_limit = 0.25;
    double fade_in_time = 0.06;
    double fade_out_time = 0.06;
    double filter_time = 0.05;
    double stale_timeout = 1.00;
    int loop_hz = 100;

    std::string hotkey = "PAUSE";
    std::filesystem::path log_path = "auto_rudder.log";
    double calibration_max_assist = 0.08;
};

AppConfig load_config(const std::filesystem::path& path);
void write_default_config(const std::filesystem::path& path);

}  // namespace autorudder
