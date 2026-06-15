#include "config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace autorudder {
namespace {

std::string trim(std::string value) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool starts_with(std::string_view text, std::string_view prefix) {
    return text.size() >= prefix.size() && text.substr(0, prefix.size()) == prefix;
}

int parse_int(const std::string& value, const std::string& key) {
    try {
        return std::stoi(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid integer for key '" + key + "': " + value);
    }
}

double parse_double(const std::string& value, const std::string& key) {
    try {
        return std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid number for key '" + key + "': " + value);
    }
}

void apply_key(AppConfig& cfg, const std::string& key, const std::string& value) {
    if (key == "dcs_bios_path") cfg.dcs_bios_path = value;
    else if (key == "multicast_address") cfg.multicast_address = value;
    else if (key == "multicast_interface") cfg.multicast_interface = value;
    else if (key == "udp_port") cfg.udp_port = parse_int(value, key);
    else if (key == "telemetry_source") cfg.telemetry_source = value;
    else if (key == "fast_export_bind_address") cfg.fast_export_bind_address = value;
    else if (key == "fast_export_port") cfg.fast_export_port = parse_int(value, key);
    else if (key == "input_vjoy_id") cfg.input_vjoy_id = parse_int(value, key);
    else if (key == "output_vjoy_id") cfg.output_vjoy_id = parse_int(value, key);
    else if (key == "input_device_name_contains") cfg.input_device_name_contains = value;
    else if (key == "axis_name") cfg.axis_name = value;
    else if (key == "assist_sign") cfg.assist_sign = parse_double(value, key);
    else if (key == "kp") cfg.kp = parse_double(value, key);
    else if (key == "max_assist") cfg.max_assist = parse_double(value, key);
    else if (key == "yaw_rate_deadband") cfg.yaw_rate_deadband = parse_double(value, key);
    else if (key == "pedal_override_threshold") cfg.pedal_override_threshold = parse_double(value, key);
    else if (key == "pedal_rate_override_threshold") cfg.pedal_rate_override_threshold = parse_double(value, key);
    else if (key == "fade_in_time") cfg.fade_in_time = parse_double(value, key);
    else if (key == "fade_out_time") cfg.fade_out_time = parse_double(value, key);
    else if (key == "filter_time") cfg.filter_time = parse_double(value, key);
    else if (key == "stale_timeout") cfg.stale_timeout = parse_double(value, key);
    else if (key == "loop_hz") cfg.loop_hz = parse_int(value, key);
    else if (key == "hotkey") cfg.hotkey = value;
    else if (key == "log_path") cfg.log_path = value;
    else if (key == "calibration_max_assist") cfg.calibration_max_assist = parse_double(value, key);
}

void validate(const AppConfig& cfg) {
    if (cfg.udp_port <= 0 || cfg.udp_port > 65535) throw std::runtime_error("udp_port out of range");
    if (cfg.fast_export_port <= 0 || cfg.fast_export_port > 65535) throw std::runtime_error("fast_export_port out of range");
    if (cfg.telemetry_source != "dcs_bios" && cfg.telemetry_source != "fast_export") {
        throw std::runtime_error("telemetry_source must be dcs_bios or fast_export");
    }
    if (cfg.input_vjoy_id <= 0 || cfg.output_vjoy_id <= 0) throw std::runtime_error("vJoy IDs must be positive");
    if (cfg.loop_hz < 20 || cfg.loop_hz > 500) throw std::runtime_error("loop_hz must be between 20 and 500");
    if (cfg.max_assist < 0.0 || cfg.max_assist > 1.0) throw std::runtime_error("max_assist must be in [0, 1]");
    if (cfg.calibration_max_assist < 0.0 || cfg.calibration_max_assist > 0.25) {
        throw std::runtime_error("calibration_max_assist must be in [0, 0.25]");
    }
    if (cfg.filter_time < 0.0) throw std::runtime_error("filter_time must be non-negative");
    if (cfg.stale_timeout <= 0.0) throw std::runtime_error("stale_timeout must be positive");
}

}  // namespace

AppConfig load_config(const std::filesystem::path& path) {
    AppConfig cfg;
    std::ifstream in(path);
    if (!in) {
        validate(cfg);
        return cfg;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || starts_with(line, "#") || starts_with(line, ";") || starts_with(line, "[")) {
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if ((value.size() >= 2) &&
            ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.size() - 2);
        }
        apply_key(cfg, key, value);
    }

    validate(cfg);
    return cfg;
}

void write_default_config(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("Could not write default config: " + path.string());
    }

    const AppConfig cfg;
    out << "# AH-64D yaw-rate auto rudder\n"
        << "dcs_bios_path=" << cfg.dcs_bios_path.string() << "\n"
        << "multicast_address=" << cfg.multicast_address << "\n"
        << "multicast_interface=" << cfg.multicast_interface << "\n"
        << "udp_port=" << cfg.udp_port << "\n\n"
        << "telemetry_source=" << cfg.telemetry_source << "\n"
        << "fast_export_bind_address=" << cfg.fast_export_bind_address << "\n"
        << "fast_export_port=" << cfg.fast_export_port << "\n\n"
        << "input_vjoy_id=" << cfg.input_vjoy_id << "\n"
        << "output_vjoy_id=" << cfg.output_vjoy_id << "\n"
        << "input_device_name_contains=" << cfg.input_device_name_contains << "\n"
        << "axis_name=" << cfg.axis_name << "\n\n"
        << "assist_sign=" << cfg.assist_sign << "\n"
        << "kp=" << cfg.kp << "\n"
        << "max_assist=" << cfg.max_assist << "\n"
        << "yaw_rate_deadband=" << cfg.yaw_rate_deadband << "\n"
        << "pedal_override_threshold=" << cfg.pedal_override_threshold << "\n"
        << "pedal_rate_override_threshold=" << cfg.pedal_rate_override_threshold << "\n"
        << "fade_in_time=" << cfg.fade_in_time << "\n"
        << "fade_out_time=" << cfg.fade_out_time << "\n"
        << "filter_time=" << cfg.filter_time << "\n"
        << "stale_timeout=" << cfg.stale_timeout << "\n"
        << "loop_hz=" << cfg.loop_hz << "\n\n"
        << "hotkey=" << cfg.hotkey << "\n"
        << "log_path=" << cfg.log_path.string() << "\n"
        << "calibration_max_assist=" << cfg.calibration_max_assist << "\n";
}

}  // namespace autorudder
