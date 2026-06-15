#include "config.h"
#include "dcs_bios_protocol.h"
#include "dcs_bios_refs.h"
#include "dcs_bios_state.h"
#include "yaw_damper.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
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

autorudder::AppConfig test_config() {
    autorudder::AppConfig cfg;
    cfg.kp = 0.5;
    cfg.max_assist = 0.2;
    cfg.yaw_rate_deadband = 0.0;
    cfg.pedal_override_threshold = 0.12;
    cfg.pedal_rate_override_threshold = 10.0;
    cfg.fade_in_time = 0.1;
    cfg.fade_out_time = 0.1;
    cfg.filter_time = 0.0;
    return cfg;
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

}  // namespace

int main() {
    test_protocol_parser_applies_writes();
    test_protocol_parser_handles_multiple_frames();
    test_ref_extraction();
    test_yaw_damper_assists_centered_pedals();
    test_yaw_damper_user_override();
    test_yaw_damper_stale_or_wrong_aircraft_passes_through();

    if (failures != 0) {
        std::cerr << failures << " test failure(s)\n";
        return EXIT_FAILURE;
    }
    std::cout << "All tests passed\n";
    return EXIT_SUCCESS;
}
