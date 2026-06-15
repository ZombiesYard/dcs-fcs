#pragma once

#include <string>
#include <vector>

#include <windows.h>

namespace autorudder::windows {

struct VJoyDeviceStatus {
    int id = 0;
    std::string status;
};

class VJoyDevice {
public:
    VJoyDevice(int id, const std::string& axis_name);
    ~VJoyDevice();

    VJoyDevice(const VJoyDevice&) = delete;
    VJoyDevice& operator=(const VJoyDevice&) = delete;

    void set_axis(double value);
    static std::vector<VJoyDeviceStatus> list_statuses();
    static FARPROC require_proc(HMODULE library, const char* name);

private:
    using VJoyEnabledFn = BOOL(__cdecl*)();
    using GetVJDStatusFn = int(__cdecl*)(UINT);
    using AcquireVJDFn = BOOL(__cdecl*)(UINT);
    using RelinquishVJDFn = void(__cdecl*)(UINT);
    using ResetVJDFn = BOOL(__cdecl*)(UINT);
    using SetAxisFn = BOOL(__cdecl*)(LONG, UINT, UINT);
    using GetAxisRangeFn = BOOL(__cdecl*)(UINT, UINT, LONG*);

    static UINT axis_usage(const std::string& axis_name);
    static std::string status_name(int status);

    HMODULE library_ = nullptr;
    UINT id_ = 0;
    UINT axis_ = 0;
    LONG axis_min_ = 0;
    LONG axis_max_ = 32768;
    RelinquishVJDFn relinquish_ = nullptr;
    SetAxisFn set_axis_ = nullptr;
};

}  // namespace autorudder::windows
