#include "dcs_bios_protocol.h"

namespace autorudder {

void DcsBiosProtocolParser::set_write_callback(WriteCallback callback) {
    write_callback_ = std::move(callback);
}

void DcsBiosProtocolParser::set_frame_callback(FrameCallback callback) {
    frame_callback_ = std::move(callback);
}

void DcsBiosProtocolParser::reset() {
    state_ = State::SeekingSync;
    sync_count_ = 0;
    address_ = 0;
    length_ = 0;
    data_.clear();
}

void DcsBiosProtocolParser::feed(const std::uint8_t* data, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) {
        consume(data[i]);
    }
}

void DcsBiosProtocolParser::consume(std::uint8_t byte) {
    if (state_ == State::SeekingSync) {
        if (byte == 0x55) {
            ++sync_count_;
            if (sync_count_ == 4) {
                sync_count_ = 0;
                state_ = State::AddressLow;
                if (frame_callback_) {
                    frame_callback_();
                }
            }
        } else {
            sync_count_ = 0;
        }
        return;
    }

    switch (state_) {
    case State::AddressLow:
        if (byte == 0x55) {
            state_ = State::SeekingSync;
            sync_count_ = 1;
            return;
        }
        address_ = byte;
        state_ = State::AddressHigh;
        break;
    case State::AddressHigh:
        address_ |= static_cast<std::uint16_t>(byte) << 8;
        state_ = State::LengthLow;
        break;
    case State::LengthLow:
        length_ = byte;
        state_ = State::LengthHigh;
        break;
    case State::LengthHigh:
        length_ |= static_cast<std::uint16_t>(byte) << 8;
        data_.clear();
        data_.reserve(length_);
        state_ = length_ == 0 ? State::AddressLow : State::Data;
        if (length_ == 0 && write_callback_) {
            write_callback_(BiosWrite{address_, {}});
        }
        break;
    case State::Data:
        data_.push_back(byte);
        if (data_.size() == length_) {
            if (write_callback_) {
                write_callback_(BiosWrite{address_, data_});
            }
            state_ = State::AddressLow;
        }
        break;
    case State::SeekingSync:
        break;
    }
}

}  // namespace autorudder
