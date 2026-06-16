#include "windows/fast_export_udp_client.h"

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ws2tcpip.h>

namespace autorudder::windows {
namespace {

std::string socket_error(const char* operation) {
    return std::string(operation) + " failed, WSA error " + std::to_string(WSAGetLastError());
}

void ensure_winsock_started() {
    static bool started = [] {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
        return true;
    }();
    (void)started;
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        if (!field.empty() && (field.back() == '\n' || field.back() == '\r')) {
            field.pop_back();
        }
        fields.push_back(field);
    }
    return fields;
}

std::optional<FastExportTelemetry> parse_line(const std::string& line) {
    const auto fields = split_csv(line);
    if (fields.size() < 5 || fields[0] != "AR1") {
        return std::nullopt;
    }

    FastExportTelemetry telemetry;
    telemetry.aircraft_name = fields[2];
    try {
        telemetry.yaw_rate_z = std::stod(fields[3]);
        telemetry.slip_ball = std::stod(fields[4]);
        if (fields.size() >= 6 && !fields[5].empty()) {
            telemetry.collective = std::stod(fields[5]);
        }
        if (fields.size() >= 7 && !fields[6].empty()) {
            telemetry.heading = std::stod(fields[6]);
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
    return telemetry;
}

}  // namespace

FastExportUdpClient::FastExportUdpClient(const std::string& bind_address, int port) {
    ensure_winsock_started();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        throw std::runtime_error(socket_error("socket"));
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = inet_addr(bind_address.c_str());
    local.sin_port = htons(static_cast<u_short>(port));
    if (bind(socket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        throw std::runtime_error(socket_error("bind fast_export"));
    }

    u_long non_blocking = 1;
    if (ioctlsocket(socket_, FIONBIO, &non_blocking) != 0) {
        throw std::runtime_error(socket_error("ioctlsocket(FIONBIO)"));
    }
}

FastExportUdpClient::~FastExportUdpClient() {
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
    }
}

int FastExportUdpClient::pump() {
    std::array<char, 1024> buffer{};
    int packets = 0;
    for (;;) {
        const int received = recv(socket_, buffer.data(), static_cast<int>(buffer.size() - 1), 0);
        if (received > 0) {
            buffer[static_cast<std::size_t>(received)] = '\0';
            if (auto parsed = parse_line(std::string(buffer.data(), static_cast<std::size_t>(received)))) {
                latest_ = *parsed;
                last_frame_time_ = std::chrono::steady_clock::now();
            }
            ++packets;
            continue;
        }

        const int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return packets;
        }
        throw std::runtime_error(socket_error("recv fast_export"));
    }
}

bool FastExportUdpClient::has_recent_frame(double stale_timeout_seconds) const {
    if (last_frame_time_ == std::chrono::steady_clock::time_point{}) {
        return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_time_).count();
    return age <= stale_timeout_seconds;
}

std::optional<FastExportTelemetry> FastExportUdpClient::latest() const {
    return latest_;
}

}  // namespace autorudder::windows
