#include "windows/hotkey.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <windows.h>

namespace autorudder::windows {
namespace {

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

int parse_vk(const std::string& name) {
    const std::string key = uppercase(name);
    if (key == "PAUSE") return VK_PAUSE;
    if (key == "ESC" || key == "ESCAPE") return VK_ESCAPE;
    if (key == "END") return VK_END;
    if (key == "HOME") return VK_HOME;
    if (key.size() >= 2 && key[0] == 'F') {
        const int n = std::stoi(key.substr(1));
        if (n >= 1 && n <= 24) {
            return VK_F1 + (n - 1);
        }
    }
    throw std::runtime_error("Unsupported hotkey: " + name);
}

}  // namespace

Hotkey::Hotkey(const std::string& name) : vk_(parse_vk(name)), name_(name) {}

bool Hotkey::pressed_edge() {
    const bool down = (GetAsyncKeyState(vk_) & 0x8000) != 0;
    const bool edge = down && !was_down_;
    was_down_ = down;
    return edge;
}

std::string Hotkey::name() const {
    return name_;
}

}  // namespace autorudder::windows
