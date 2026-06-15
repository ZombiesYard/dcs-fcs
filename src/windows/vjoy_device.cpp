#include "windows/vjoy_device.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <vector>

namespace autorudder::windows {
namespace {

constexpr UINT HID_USAGE_X = 0x30;
constexpr UINT HID_USAGE_Y = 0x31;
constexpr UINT HID_USAGE_Z = 0x32;
constexpr UINT HID_USAGE_RX = 0x33;
constexpr UINT HID_USAGE_RY = 0x34;
constexpr UINT HID_USAGE_RZ = 0x35;
constexpr UINT HID_USAGE_SL0 = 0x36;
constexpr UINT HID_USAGE_SL1 = 0x37;

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

template <typename Fn>
Fn load_proc(HMODULE library, const char* name) {
    return reinterpret_cast<Fn>(VJoyDevice::require_proc(library, name));
}

HMODULE load_vjoy_library() {
    if (HMODULE library = LoadLibraryA("vJoyInterface.dll")) {
        return library;
    }

    const std::vector<std::string> candidates = {
        R"(C:\Program Files\vJoy\x64\vJoyInterface.dll)",
        R"(C:\Program Files\vJoy\vJoyInterface.dll)",
        R"(C:\Program Files (x86)\vJoy\x64\vJoyInterface.dll)",
        R"(C:\Program Files (x86)\vJoy\vJoyInterface.dll)",
    };

    for (const auto& path : candidates) {
        if (GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES) {
            if (HMODULE library = LoadLibraryA(path.c_str())) {
                return library;
            }
        }
    }
    return nullptr;
}

}  // namespace

VJoyDevice::VJoyDevice(int id, const std::string& axis_name)
    : id_(static_cast<UINT>(id)), axis_(axis_usage(axis_name)) {
    library_ = load_vjoy_library();
    if (!library_) {
        throw std::runtime_error("Could not load vJoyInterface.dll. Install vJoy and ensure the DLL is on PATH.");
    }

    const auto vjoy_enabled = load_proc<VJoyEnabledFn>(library_, "vJoyEnabled");
    const auto get_status = load_proc<GetVJDStatusFn>(library_, "GetVJDStatus");
    const auto acquire = load_proc<AcquireVJDFn>(library_, "AcquireVJD");
    relinquish_ = load_proc<RelinquishVJDFn>(library_, "RelinquishVJD");
    const auto reset = load_proc<ResetVJDFn>(library_, "ResetVJD");
    set_axis_ = load_proc<SetAxisFn>(library_, "SetAxis");

    if (!vjoy_enabled()) {
        throw std::runtime_error("vJoy driver is not enabled");
    }

    const int status = get_status(id_);
    if (status != 0 && status != 1) {
        throw std::runtime_error("vJoy device " + std::to_string(id_) + " is not available: " + status_name(status));
    }
    if (!acquire(id_)) {
        throw std::runtime_error("Could not acquire vJoy device " + std::to_string(id_));
    }
    reset(id_);

    const auto get_axis_min = reinterpret_cast<GetAxisRangeFn>(GetProcAddress(library_, "GetVJDAxisMin"));
    const auto get_axis_max = reinterpret_cast<GetAxisRangeFn>(GetProcAddress(library_, "GetVJDAxisMax"));
    if (get_axis_min) {
        get_axis_min(id_, axis_, &axis_min_);
    }
    if (get_axis_max) {
        get_axis_max(id_, axis_, &axis_max_);
    }
}

VJoyDevice::~VJoyDevice() {
    if (relinquish_) {
        relinquish_(id_);
    }
    if (library_) {
        FreeLibrary(library_);
    }
}

void VJoyDevice::set_axis(double value) {
    value = std::max(-1.0, std::min(1.0, value));
    const double normalized = (value + 1.0) * 0.5;
    const auto raw = static_cast<LONG>(axis_min_ + normalized * static_cast<double>(axis_max_ - axis_min_));
    if (!set_axis_(raw, id_, axis_)) {
        throw std::runtime_error("SetAxis failed for vJoy device " + std::to_string(id_));
    }
}

std::vector<VJoyDeviceStatus> VJoyDevice::list_statuses() {
    std::vector<VJoyDeviceStatus> statuses;
    HMODULE library = load_vjoy_library();
    if (!library) {
        statuses.push_back({0, "vJoyInterface.dll not found"});
        return statuses;
    }

    const auto vjoy_enabled = reinterpret_cast<VJoyEnabledFn>(GetProcAddress(library, "vJoyEnabled"));
    const auto get_status = reinterpret_cast<GetVJDStatusFn>(GetProcAddress(library, "GetVJDStatus"));
    if (!vjoy_enabled || !get_status || !vjoy_enabled()) {
        statuses.push_back({0, "vJoy driver not enabled"});
        FreeLibrary(library);
        return statuses;
    }

    for (int id = 1; id <= 16; ++id) {
        statuses.push_back({id, status_name(get_status(static_cast<UINT>(id)))});
    }
    FreeLibrary(library);
    return statuses;
}

UINT VJoyDevice::axis_usage(const std::string& axis_name) {
    const std::string axis = uppercase(axis_name);
    if (axis == "X") return HID_USAGE_X;
    if (axis == "Y") return HID_USAGE_Y;
    if (axis == "Z") return HID_USAGE_Z;
    if (axis == "RX") return HID_USAGE_RX;
    if (axis == "RY") return HID_USAGE_RY;
    if (axis == "RZ") return HID_USAGE_RZ;
    if (axis == "SLIDER0" || axis == "SL0") return HID_USAGE_SL0;
    if (axis == "SLIDER1" || axis == "SL1") return HID_USAGE_SL1;
    throw std::runtime_error("Unsupported vJoy axis_name: " + axis_name);
}

std::string VJoyDevice::status_name(int status) {
    switch (status) {
    case 0: return "owned by this feeder";
    case 1: return "free";
    case 2: return "busy";
    case 3: return "missing";
    default: return "unknown";
    }
}

FARPROC VJoyDevice::require_proc(HMODULE library, const char* name) {
    FARPROC proc = GetProcAddress(library, name);
    if (!proc) {
        throw std::runtime_error(std::string("vJoyInterface.dll is missing function ") + name);
    }
    return proc;
}

}  // namespace autorudder::windows
