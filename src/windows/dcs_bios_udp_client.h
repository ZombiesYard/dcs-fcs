#pragma once

#include "dcs_bios_protocol.h"
#include "dcs_bios_state.h"

#include <chrono>
#include <cstdint>
#include <string>

#include <winsock2.h>

namespace autorudder::windows {

class DcsBiosUdpClient {
public:
    DcsBiosUdpClient(
        const std::string& multicast_address,
        const std::string& multicast_interface,
        int port,
        DcsBiosState& state);
    ~DcsBiosUdpClient();

    DcsBiosUdpClient(const DcsBiosUdpClient&) = delete;
    DcsBiosUdpClient& operator=(const DcsBiosUdpClient&) = delete;

    int pump();
    bool has_recent_frame(double stale_timeout_seconds) const;

private:
    SOCKET socket_ = INVALID_SOCKET;
    DcsBiosProtocolParser parser_;
    std::chrono::steady_clock::time_point last_frame_time_{};
};

}  // namespace autorudder::windows
