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
    std::string telemetry_source = "dcs_bios";
    std::string fast_export_bind_address = "127.0.0.1";
    int fast_export_port = 34380;

    int input_vjoy_id = 1;
    int output_vjoy_id = 2;
    std::string input_device_name_contains = "vJoy";
    std::string axis_name = "RZ";

    double assist_sign = 1.0;
    double kp = 0.22;
    double max_assist = 0.25;
    double yaw_rate_deadband = 0.015;
    double pedal_override_threshold = 0.12;
    double pedal_rate_override_threshold = 1.0;
    double fade_in_time = 0.60;
    double fade_out_time = 0.12;
    double filter_time = 0.18;
    double stale_timeout = 0.50;
    int loop_hz = 100;

    std::string hotkey = "PAUSE";
    std::filesystem::path log_path = "auto_rudder.log";
    double calibration_max_assist = 0.08;
};

AppConfig load_config(const std::filesystem::path& path);
void write_default_config(const std::filesystem::path& path);

}  // namespace autorudder
