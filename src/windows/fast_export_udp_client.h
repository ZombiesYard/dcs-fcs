#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <winsock2.h>

namespace autorudder::windows {

struct FastExportTelemetry {
    std::string aircraft_name;
    double yaw_rate_z = 0.0;
    std::optional<double> slip_ball;
    std::optional<double> collective;
    std::optional<double> heading;
};

class FastExportUdpClient {
public:
    FastExportUdpClient(const std::string& bind_address, int port);
    ~FastExportUdpClient();

    FastExportUdpClient(const FastExportUdpClient&) = delete;
    FastExportUdpClient& operator=(const FastExportUdpClient&) = delete;

    int pump();
    bool has_recent_frame(double stale_timeout_seconds) const;
    std::optional<FastExportTelemetry> latest() const;

private:
    SOCKET socket_ = INVALID_SOCKET;
    std::optional<FastExportTelemetry> latest_;
    std::chrono::steady_clock::time_point last_frame_time_{};
};

}  // namespace autorudder::windows
