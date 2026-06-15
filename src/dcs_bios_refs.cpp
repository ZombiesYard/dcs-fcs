#include "dcs_bios_refs.h"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace autorudder {
namespace {

std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Could not read DCS-BIOS JSON reference: " + path.string());
    }
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

std::string escape_regex(std::string_view text) {
    std::string escaped;
    for (const char ch : text) {
        switch (ch) {
        case '\\':
        case '.':
        case '^':
        case '$':
        case '|':
        case '(':
        case ')':
        case '[':
        case ']':
        case '*':
        case '+':
        case '?':
        case '{':
        case '}':
            escaped.push_back('\\');
            break;
        default:
            break;
        }
        escaped.push_back(ch);
    }
    return escaped;
}

std::optional<std::string> match_string(std::string_view text, const std::regex& rx) {
    std::cmatch match;
    if (!std::regex_search(text.data(), text.data() + text.size(), match, rx)) {
        return std::nullopt;
    }
    return match[1].str();
}

std::optional<int> match_int(std::string_view text, const char* key) {
    const std::regex rx(std::string("\"") + key + R"("\s*:\s*(\d+))");
    const auto value = match_string(text, rx);
    if (!value) {
        return std::nullopt;
    }
    return std::stoi(*value);
}

BiosOutputType parse_type(std::string_view text) {
    const std::regex type_rx(R"JSON("type"\s*:\s*"([^"]+)")JSON");
    const auto type = match_string(text, type_rx);
    if (type && *type == "string") {
        return BiosOutputType::String;
    }
    return BiosOutputType::Integer;
}

}  // namespace

std::optional<BiosOutputRef> extract_output_ref(std::string_view json, std::string_view identifier) {
    const std::string id_pattern =
        R"("identifier"\s*:\s*")" + escape_regex(identifier) + R"(")";
    const std::regex id_rx(id_pattern);
    std::cmatch id_match;
    if (!std::regex_search(json.data(), json.data() + json.size(), id_match, id_rx)) {
        return std::nullopt;
    }

    const char* start = id_match[0].second;
    const char* end = json.data() + json.size();
    const std::string_view tail(start, static_cast<std::size_t>(end - start));
    const auto outputs_pos = tail.find("\"outputs\"");
    if (outputs_pos == std::string_view::npos) {
        return std::nullopt;
    }

    const auto next_identifier_pos = tail.find("\"identifier\"", 1);
    const auto scan_size = next_identifier_pos == std::string_view::npos ? tail.size() : next_identifier_pos;
    const std::string_view control(tail.data(), scan_size);

    const auto address = match_int(control, "address");
    if (!address || *address < 0 || *address > 65535) {
        return std::nullopt;
    }

    BiosOutputRef ref;
    ref.identifier = std::string(identifier);
    ref.address = static_cast<std::uint16_t>(*address);
    ref.type = parse_type(control);
    ref.mask = static_cast<std::uint16_t>(match_int(control, "mask").value_or(65535));
    ref.shift_by = match_int(control, "shift_by").value_or(0);
    ref.max_value = match_int(control, "max_value").value_or(65535);
    ref.max_length = static_cast<std::size_t>(match_int(control, "max_length").value_or(0));

    if (ref.type == BiosOutputType::String && ref.max_length == 0) {
        return std::nullopt;
    }
    return ref;
}

TelemetryRefs load_telemetry_refs(const std::filesystem::path& dcs_bios_path) {
    const auto json_dir = dcs_bios_path / "doc" / "json";
    const auto metadata = read_file(json_dir / "MetadataStart.json");
    const auto common = read_file(json_dir / "CommonData.json");
    const auto ah64 = read_file(json_dir / "AH-64D.json");

    TelemetryRefs refs;
    auto acft = extract_output_ref(metadata, "_ACFT_NAME");
    auto yaw = extract_output_ref(common, "ANGULAR_VELOCITY_Z");
    if (!acft || !yaw) {
        throw std::runtime_error("Required DCS-BIOS outputs were not found in JSON references");
    }
    refs.aircraft_name = *acft;
    refs.yaw_rate_z = *yaw;
    refs.slip_ball = extract_output_ref(ah64, "PLT_SAI_BALL");
    return refs;
}

}  // namespace autorudder
