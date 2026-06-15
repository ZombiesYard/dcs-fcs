#pragma once

#include "dcs_bios_protocol.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autorudder {

class DcsBiosState {
public:
    explicit DcsBiosState(std::size_t bytes = 65536);

    void apply_write(const BiosWrite& write);
    std::string read_string(std::uint16_t address, std::size_t max_length) const;
    std::optional<std::uint16_t> read_u16(std::uint16_t address) const;

private:
    std::vector<std::uint8_t> memory_;
};

}  // namespace autorudder
