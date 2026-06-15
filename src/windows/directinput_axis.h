#pragma once

#include <optional>
#include <string>
#include <vector>

#include <dinput.h>

namespace autorudder::windows {

struct DirectInputDeviceInfo {
    int index = 0;
    std::string instance_name;
    std::string product_name;
    std::string instance_guid;
    std::string product_guid;
};

class DirectInputAxisInput {
public:
    DirectInputAxisInput(int occurrence, const std::string& name_filter, const std::string& axis_name);
    ~DirectInputAxisInput();

    DirectInputAxisInput(const DirectInputAxisInput&) = delete;
    DirectInputAxisInput& operator=(const DirectInputAxisInput&) = delete;

    std::optional<double> read();
    std::string selected_name() const;

    static std::vector<DirectInputDeviceInfo> list_devices();

private:
    enum class Axis {
        X,
        Y,
        Z,
        RX,
        RY,
        RZ,
        Slider0,
        Slider1,
    };

    static Axis parse_axis(const std::string& axis_name);
    static double normalize_long(long value);

    IDirectInput8A* direct_input_ = nullptr;
    IDirectInputDevice8A* device_ = nullptr;
    Axis axis_ = Axis::RZ;
    std::string selected_name_;
};

}  // namespace autorudder::windows
