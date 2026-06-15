#include "dcs_bios_state.h"

#include <algorithm>

namespace autorudder {

DcsBiosState::DcsBiosState(std::size_t bytes) : memory_(bytes, 0) {}

void DcsBiosState::apply_write(const BiosWrite& write) {
    if (write.address >= memory_.size()) {
        return;
    }
    const std::size_t count = std::min<std::size_t>(write.data.size(), memory_.size() - write.address);
    std::copy_n(write.data.begin(), count, memory_.begin() + write.address);
}

std::string DcsBiosState::read_string(std::uint16_t address, std::size_t max_length) const {
    if (address >= memory_.size()) {
        return {};
    }

    const std::size_t count = std::min<std::size_t>(max_length, memory_.size() - address);
    std::string value;
    value.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const char ch = static_cast<char>(memory_[address + i]);
        if (ch == '\0') {
            break;
        }
        value.push_back(ch);
    }
    return value;
}

std::optional<std::uint16_t> DcsBiosState::read_u16(std::uint16_t address) const {
    if (static_cast<std::size_t>(address) + 1 >= memory_.size()) {
        return std::nullopt;
    }
    const auto lo = static_cast<std::uint16_t>(memory_[address]);
    const auto hi = static_cast<std::uint16_t>(memory_[address + 1]);
    return static_cast<std::uint16_t>(lo | (hi << 8));
}

}  // namespace autorudder
