#include "app.h"

#include "config.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
#include "logger.h"
#include "windows/dcs_bios_udp_client.h"
#include "windows/directinput_axis.h"
#include "windows/fast_export_udp_client.h"
#include "windows/hotkey.h"
#include "windows/vjoy_device.h"
#include "yaw_damper.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <windows.h>

namespace autorudder {
namespace {

std::atomic_bool g_running{true};

BOOL WINAPI console_handler(DWORD control_type) {
    if (control_type == CTRL_C_EVENT || control_type == CTRL_CLOSE_EVENT || control_type == CTRL_BREAK_EVENT) {
        g_running = false;
        return TRUE;
    }
    return FALSE;
}

struct CliOptions {
    std::filesystem::path config_path = "config.ini";
    bool dry_run = false;
    bool list_devices = false;
    bool calibrate_sign = false;
    bool test_output = false;
    bool help = false;
};

void print_help() {
    std::cout
        << "AH-64D external yaw FBW / auto rudder\n"
        << "Usage: ah64d_auto_rudder [--config PATH] [--dry-run] [--list-devices] [--calibrate-sign]\n\n"
        << "  --list-devices    Print DirectInput and vJoy devices, then exit.\n"
        << "  --dry-run         Decode DCS-BIOS and pedals without writing vJoy.\n"
        << "  --calibrate-sign  Run a low-authority sign comparison on vJoy output.\n"
        << "  --test-output     Sweep the configured output vJoy axis for binding diagnostics.\n"
        << "  --config PATH     Use another config.ini path.\n"
        << "  --help            Print this help.\n";
}

CliOptions parse_args(const std::vector<std::string>& args) {
    CliOptions options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else if (arg == "--list-devices") {
            options.list_devices = true;
        } else if (arg == "--calibrate-sign") {
            options.calibrate_sign = true;
        } else if (arg == "--test-output") {
            options.test_output = true;
        } else if (arg == "--config") {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("--config requires a path");
            }
            options.config_path = args[++i];
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return options;
}

std::string trim(std::string value) {
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool is_ah64(const std::string& aircraft_name) {
    return aircraft_name.find("AH-64D") != std::string::npos;
}

double parse_double_or_zero(const std::string& value) {
    try {
        return std::stod(trim(value));
    } catch (const std::exception&) {
        return 0.0;
    }
}

struct Telemetry {
    std::string aircraft_name;
    bool aircraft_is_ah64 = false;
    double yaw_rate_z = 0.0;
    std::optional<double> slip_ball;
    std::optional<double> collective;
    std::optional<double> heading;
};

struct TelemetrySample {
    Telemetry telemetry;
    bool fresh = false;
};

Telemetry read_telemetry(const DcsBiosState& state, const TelemetryRefs& refs) {
    Telemetry telemetry;
    telemetry.aircraft_name = state.read_string(refs.aircraft_name.address, refs.aircraft_name.max_length);
    telemetry.aircraft_is_ah64 = is_ah64(telemetry.aircraft_name);
    telemetry.yaw_rate_z = parse_double_or_zero(state.read_string(refs.yaw_rate_z.address, refs.yaw_rate_z.max_length));

    if (refs.slip_ball) {
        const auto raw = state.read_u16(refs.slip_ball->address);
        if (raw) {
            telemetry.slip_ball = (static_cast<double>(*raw) / 65535.0) * 2.0 - 1.0;
        }
    }
    return telemetry;
}

std::string fixed3(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

void print_device_lists() {
    std::cout << "DirectInput game controllers:\n";
    const auto devices = windows::DirectInputAxisInput::list_devices();
    if (devices.empty()) {
        std::cout << "  none\n";
    }
    for (const auto& device : devices) {
        std::cout << "  [" << device.index << "] " << device.product_name
                  << " / " << device.instance_name
                  << " instance=" << device.instance_guid
                  << " product=" << device.product_guid << '\n';
    }

    std::cout << "\nvJoy status:\n";
    for (const auto& status : windows::VJoyDevice::list_statuses()) {
        if (status.id == 0) {
            std::cout << "  " << status.status << '\n';
        } else {
            std::cout << "  vJoy #" << status.id << ": " << status.status << '\n';
        }
    }
}

struct Runtime {
    AppConfig cfg;
    Logger logger;
    TelemetryRefs refs;
    DcsBiosState bios_state;
    windows::DcsBiosUdpClient bios;
    std::optional<windows::FastExportUdpClient> fast_export;
    windows::DirectInputAxisInput pedals;
    std::optional<windows::DirectInputAxisInput> collective_input;

    Runtime(AppConfig config)
        : cfg(std::move(config)),
          logger(cfg.log_path),
          refs(load_telemetry_refs(cfg.dcs_bios_path)),
          bios_state(),
          bios(cfg.multicast_address, cfg.multicast_interface, cfg.udp_port, bios_state),
          pedals(cfg.input_vjoy_id, cfg.input_device_name_contains, cfg.axis_name) {
        if (cfg.telemetry_source == "fast_export") {
            fast_export.emplace(cfg.fast_export_bind_address, cfg.fast_export_port);
        }
        if (cfg.collective_source == "directinput") {
            collective_input.emplace(cfg.collective_input_id, cfg.collective_device_name_contains, cfg.collective_axis_name);
        }
    }
};

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double directinput_axis_to_collective(double raw_axis, const AppConfig& cfg) {
    double value = (raw_axis + 1.0) * 0.5;
    if (cfg.collective_invert > 0.5) {
        value = 1.0 - value;
    }
    return clamp01(value);
}

std::optional<double> read_collective(Runtime& runtime, const Telemetry& telemetry) {
    if (runtime.cfg.collective_source == "directinput") {
        if (!runtime.collective_input) {
            return std::nullopt;
        }
        if (const auto raw = runtime.collective_input->read()) {
            return directinput_axis_to_collective(*raw, runtime.cfg);
        }
        return std::nullopt;
    }
    if (runtime.cfg.collective_source == "fast_export") {
        return telemetry.collective;
    }
    return std::nullopt;
}

TelemetrySample sample_telemetry(Runtime& runtime) {
    runtime.bios.pump();
    if (runtime.fast_export) {
        runtime.fast_export->pump();
        TelemetrySample sample;
        if (const auto latest = runtime.fast_export->latest()) {
            sample.telemetry.aircraft_name = latest->aircraft_name;
            sample.telemetry.aircraft_is_ah64 = is_ah64(latest->aircraft_name);
            sample.telemetry.yaw_rate_z = latest->yaw_rate_z;
            sample.telemetry.slip_ball = latest->slip_ball;
            sample.telemetry.collective = latest->collective;
            sample.telemetry.heading = latest->heading;
        }
        sample.fresh = runtime.fast_export->has_recent_frame(runtime.cfg.stale_timeout);
        return sample;
    }

    TelemetrySample sample;
    sample.telemetry = read_telemetry(runtime.bios_state, runtime.refs);
    sample.fresh = runtime.bios.has_recent_frame(runtime.cfg.stale_timeout);
    return sample;
}

int run_normal(CliOptions options, AppConfig cfg) {
    Runtime runtime(std::move(cfg));
    windows::Hotkey hotkey(runtime.cfg.hotkey);
    std::optional<windows::VJoyDevice> output;
    if (!options.dry_run) {
        output.emplace(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);
    }

    runtime.logger.info("Selected pedal input: " + runtime.pedals.selected_name());
    if (runtime.collective_input) {
        runtime.logger.info("Selected collective input: " + runtime.collective_input->selected_name());
    } else {
        runtime.logger.info("Collective source: " + runtime.cfg.collective_source);
    }
    runtime.logger.info("Telemetry source: " + runtime.cfg.telemetry_source);
    runtime.logger.info(options.dry_run ? "Running in dry-run mode" : "Writing final rudder to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id));
    runtime.logger.info("Press " + hotkey.name() + " to toggle assist; Ctrl+C to exit.");

    YawDamper damper(runtime.cfg);
    bool assist_enabled = true;
    auto last = std::chrono::steady_clock::now();
    auto next_log = last;
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(runtime.cfg.loop_hz));

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        if (hotkey.pressed_edge()) {
            assist_enabled = !assist_enabled;
            runtime.logger.info(std::string("Assist ") + (assist_enabled ? "enabled" : "disabled"));
        }

        const auto pedal = runtime.pedals.read();
        const TelemetrySample sample = sample_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto collective = read_collective(runtime, telemetry);

        YawDamperInput damper_input;
        damper_input.dt = dt;
        damper_input.physical_rudder = pedal.value_or(0.0);
        damper_input.yaw_rate_z = telemetry.yaw_rate_z;
        damper_input.heading = telemetry.heading.value_or(0.0);
        damper_input.collective = collective.value_or(0.0);
        damper_input.heading_valid = telemetry.heading.has_value();
        damper_input.collective_valid = collective.has_value();
        damper_input.telemetry_fresh = sample.fresh;
        damper_input.aircraft_is_ah64 = telemetry.aircraft_is_ah64;
        damper_input.input_valid = pedal.has_value();
        damper_input.assist_enabled = assist_enabled;
        const auto result = damper.update(damper_input);

        if (output) {
            output->set_axis(result.final_rudder);
        }

        if (now >= next_log) {
            std::ostringstream line;
            line << "acft=" << (telemetry.aircraft_name.empty() ? "NONE" : telemetry.aircraft_name)
                 << " fresh=" << (sample.fresh ? "yes" : "no")
                 << " pedal=" << fixed3(pedal.value_or(0.0))
                 << " yawZ=" << fixed3(telemetry.yaw_rate_z)
                 << " rCmd=" << fixed3(result.yaw_rate_command)
                 << " hdg=" << (telemetry.heading ? fixed3(*telemetry.heading) : "NA")
                 << " href=" << fixed3(result.heading_ref)
                 << " hErr=" << fixed3(result.heading_error)
                 << " coll=" << (collective ? fixed3(*collective) : "NA")
                 << " cdot=" << fixed3(result.collective_rate)
                 << " cff=" << fixed3(result.collective_feedforward)
                 << " filt=" << fixed3(result.filtered_yaw_rate)
                 << " assist=" << fixed3(result.assist_offset)
                 << " hold=" << fixed3(result.integral_assist)
                 << " trim=" << fixed3(result.trim_bias)
                 << " final=" << fixed3(result.final_rudder)
                 << " mode=" << result.reason;
            if (telemetry.slip_ball) {
                line << " slip=" << fixed3(*telemetry.slip_ball);
            }
            runtime.logger.info(line.str());
            next_log = now + std::chrono::milliseconds(250);
        }

        std::this_thread::sleep_until(now + loop_period);
    }

    if (output) {
        output->set_axis(0.0);
    }
    runtime.logger.info("Stopped");
    return 0;
}

double run_calibration_phase(Runtime& runtime, windows::VJoyDevice& output, double sign, double seconds) {
    AppConfig cfg = runtime.cfg;
    cfg.assist_sign = sign;
    cfg.yaw_response_sign = sign;
    cfg.max_assist = cfg.calibration_max_assist;
    cfg.fade_in_time = 0.30;
    cfg.fade_out_time = 0.10;
    YawDamper damper(cfg);

    runtime.logger.info("Calibration phase assist_sign=" + fixed3(sign));
    const auto started = std::chrono::steady_clock::now();
    auto last = started;
    double sum_abs_yaw = 0.0;
    int samples = 0;
    const auto loop_period = std::chrono::duration<double>(1.0 / static_cast<double>(cfg.loop_hz));

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        if (elapsed >= seconds) {
            break;
        }
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        const auto pedal = runtime.pedals.read();
        const TelemetrySample sample = sample_telemetry(runtime);
        const Telemetry& telemetry = sample.telemetry;
        const auto collective = read_collective(runtime, telemetry);
        YawDamperInput damper_input;
        damper_input.dt = dt;
        damper_input.physical_rudder = pedal.value_or(0.0);
        damper_input.yaw_rate_z = telemetry.yaw_rate_z;
        damper_input.heading = telemetry.heading.value_or(0.0);
        damper_input.collective = collective.value_or(0.0);
        damper_input.heading_valid = telemetry.heading.has_value();
        damper_input.collective_valid = collective.has_value();
        damper_input.telemetry_fresh = sample.fresh;
        damper_input.aircraft_is_ah64 = telemetry.aircraft_is_ah64;
        damper_input.input_valid = pedal.has_value();
        damper_input.assist_enabled = true;
        const auto result = damper.update(damper_input);
        output.set_axis(result.final_rudder);

        if (elapsed > 2.0 && sample.fresh && telemetry.aircraft_is_ah64 && !result.user_override) {
            sum_abs_yaw += std::abs(telemetry.yaw_rate_z);
            ++samples;
        }
        std::this_thread::sleep_until(now + loop_period);
    }

    output.set_axis(0.0);
    if (samples == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return sum_abs_yaw / static_cast<double>(samples);
}

int run_calibration(AppConfig cfg) {
    Runtime runtime(std::move(cfg));
    windows::VJoyDevice output(runtime.cfg.output_vjoy_id, runtime.cfg.axis_name);
    runtime.logger.info("Calibration writes low-authority rudder to vJoy #" + std::to_string(runtime.cfg.output_vjoy_id));
    runtime.logger.info("Keep pedals centered. Move pedals to force override, Ctrl+C to abort.");

    const double score_positive = run_calibration_phase(runtime, output, 1.0, 8.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    const double score_negative = run_calibration_phase(runtime, output, -1.0, 8.0);

    output.set_axis(0.0);
    runtime.logger.info("Calibration score sign=+1 mean_abs_yaw=" + fixed3(score_positive));
    runtime.logger.info("Calibration score sign=-1 mean_abs_yaw=" + fixed3(score_negative));
    if (std::isfinite(score_positive) && std::isfinite(score_negative)) {
        const double recommended = score_positive <= score_negative ? 1.0 : -1.0;
        runtime.logger.info("Recommended config: assist_sign=" + fixed3(recommended));
        runtime.logger.info("Recommended config: yaw_response_sign=" + fixed3(recommended));
    } else {
        runtime.logger.warn("Calibration did not collect enough AH-64D fresh telemetry samples");
    }
    return 0;
}

int run_test_output(AppConfig cfg) {
    Logger logger(cfg.log_path);
    windows::VJoyDevice output(cfg.output_vjoy_id, cfg.axis_name);
    logger.info("Sweeping vJoy #" + std::to_string(cfg.output_vjoy_id) + " axis " + cfg.axis_name + " for diagnostics");
    logger.info("Open DCS axis controls or RCtrl+Enter. Stop with Ctrl+C.");

    const auto started = std::chrono::steady_clock::now();
    auto next_log = started;
    constexpr double pi = 3.14159265358979323846;
    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - started).count();
        const double value = 0.80 * std::sin(2.0 * pi * 0.20 * t);
        output.set_axis(value);

        if (now >= next_log) {
            logger.info("test_output final=" + fixed3(value));
            next_log = now + std::chrono::milliseconds(500);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    output.set_axis(0.0);
    logger.info("Stopped test output");
    return 0;
}

}  // namespace

int run_app(const std::vector<std::string>& args) {
    try {
        SetConsoleCtrlHandler(console_handler, TRUE);
        const CliOptions options = parse_args(args);
        if (options.help) {
            print_help();
            return 0;
        }
        if (options.list_devices) {
            print_device_lists();
            return 0;
        }

        if (!std::filesystem::exists(options.config_path)) {
            write_default_config(options.config_path);
            std::cout << "Wrote default config: " << options.config_path << '\n';
        }
        AppConfig cfg = load_config(options.config_path);

        if (options.test_output) {
            return run_test_output(std::move(cfg));
        }
        if (options.calibrate_sign) {
            return run_calibration(std::move(cfg));
        }
        return run_normal(options, std::move(cfg));
    } catch (const std::exception& ex) {
        std::cerr << "ERROR: " << ex.what() << '\n';
        return 1;
    }
}

}  // namespace autorudder
