#pragma once

#include <string>

namespace autorudder::windows {

class Hotkey {
public:
    explicit Hotkey(const std::string& name);

    bool pressed_edge();
    std::string name() const;

private:
    int vk_ = 0;
    bool was_down_ = false;
    std::string name_;
};

}  // namespace autorudder::windows
