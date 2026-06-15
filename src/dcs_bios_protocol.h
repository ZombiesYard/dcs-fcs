#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace autorudder {

struct BiosWrite {
    std::uint16_t address = 0;
    std::vector<std::uint8_t> data;
};

class DcsBiosProtocolParser {
public:
    using WriteCallback = std::function<void(const BiosWrite&)>;
    using FrameCallback = std::function<void()>;

    void set_write_callback(WriteCallback callback);
    void set_frame_callback(FrameCallback callback);
    void reset();
    void feed(const std::uint8_t* data, std::size_t size);

private:
    enum class State {
        SeekingSync,
        AddressLow,
        AddressHigh,
        LengthLow,
        LengthHigh,
        Data,
    };

    void consume(std::uint8_t byte);

    State state_ = State::SeekingSync;
    int sync_count_ = 0;
    std::uint16_t address_ = 0;
    std::uint16_t length_ = 0;
    std::vector<std::uint8_t> data_;
    WriteCallback write_callback_;
    FrameCallback frame_callback_;
};

}  // namespace autorudder
