#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace autorudder {

enum class BiosOutputType {
    String,
    Integer,
};

struct BiosOutputRef {
    std::string identifier;
    BiosOutputType type = BiosOutputType::Integer;
    std::uint16_t address = 0;
    std::uint16_t mask = 0xFFFF;
    int shift_by = 0;
    int max_value = 65535;
    std::size_t max_length = 0;
};

struct TelemetryRefs {
    BiosOutputRef aircraft_name;
    BiosOutputRef yaw_rate_z;
    std::optional<BiosOutputRef> slip_ball;
};

std::optional<BiosOutputRef> extract_output_ref(std::string_view json, std::string_view identifier);
TelemetryRefs load_telemetry_refs(const std::filesystem::path& dcs_bios_path);

}  // namespace autorudder
