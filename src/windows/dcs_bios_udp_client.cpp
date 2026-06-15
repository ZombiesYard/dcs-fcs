#include "windows/dcs_bios_udp_client.h"

#include <array>
#include <stdexcept>

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

}  // namespace

DcsBiosUdpClient::DcsBiosUdpClient(
    const std::string& multicast_address,
    const std::string& multicast_interface,
    int port,
    DcsBiosState& state) {
    ensure_winsock_started();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        throw std::runtime_error(socket_error("socket"));
    }

    BOOL reuse = TRUE;
    if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse)) != 0) {
        throw std::runtime_error(socket_error("setsockopt(SO_REUSEADDR)"));
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(static_cast<u_short>(port));
    if (bind(socket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        throw std::runtime_error(socket_error("bind"));
    }

    ip_mreq request{};
    request.imr_multiaddr.s_addr = inet_addr(multicast_address.c_str());
    request.imr_interface.s_addr = inet_addr(multicast_interface.c_str());
    if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char*>(&request), sizeof(request)) != 0) {
        throw std::runtime_error(socket_error("setsockopt(IP_ADD_MEMBERSHIP)"));
    }

    u_long non_blocking = 1;
    if (ioctlsocket(socket_, FIONBIO, &non_blocking) != 0) {
        throw std::runtime_error(socket_error("ioctlsocket(FIONBIO)"));
    }

    parser_.set_write_callback([&state](const BiosWrite& write) {
        state.apply_write(write);
    });
    parser_.set_frame_callback([this] {
        last_frame_time_ = std::chrono::steady_clock::now();
    });
}

DcsBiosUdpClient::~DcsBiosUdpClient() {
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
    }
}

int DcsBiosUdpClient::pump() {
    std::array<std::uint8_t, 8192> buffer{};
    int packets = 0;

    for (;;) {
        const int received = recv(socket_, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0);
        if (received > 0) {
            parser_.feed(buffer.data(), static_cast<std::size_t>(received));
            ++packets;
            continue;
        }

        const int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return packets;
        }
        throw std::runtime_error(socket_error("recv"));
    }
}

bool DcsBiosUdpClient::has_recent_frame(double stale_timeout_seconds) const {
    if (last_frame_time_ == std::chrono::steady_clock::time_point{}) {
        return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_time_).count();
    return age <= stale_timeout_seconds;
}

}  // namespace autorudder::windows
