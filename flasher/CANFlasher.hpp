#include <cstddef>
#include <cstdint>

#include "Flasher.hpp"

namespace can_flasher {
    namespace {
        flasher::Result handle_start(uint8_t *data, size_t length) {
            uintptr_t address;
            if (length != sizeof(address))
                return flasher::Result::InvalidParameter;

            memcpy(&address, data, sizeof(address));
            return flasher::start(address);
        }

        flasher::Result handle_write(uint8_t *data, size_t length) {
            return flasher::write(data, length);
        }
    }

    void handle(uint8_t *data, size_t length) {
        using enum flasher::State;

        if (length < 1) {
            flasher::flasher_state_callback(flasher::get_state(), flasher::Result::InvalidParameter, 0);
            return;
        }

        flasher::State target_state = (flasher::State)*data;
        data += 1;
        length -= 1;

        flasher::Result result;
        switch (target_state) {
        case Idle:
            flasher::init();
            result = flasher::Result::Success;
            break;
        case Erasing:
            result = handle_start(data, length);
            break;
        case Writing:
            result = handle_write(data, length);
            break;
        default:
            result = flasher::Result::InvalidParameter;
        }

        flasher::flasher_state_callback(flasher::get_state(), result, 0);
    }

    constexpr size_t FeedbackSize =
        sizeof(flasher::State) + sizeof(flasher::Result) + sizeof(uint32_t);
    static_assert(FeedbackSize <= 8);

    struct {
        uint32_t last_sent;
        std::atomic<bool> updated;
        bool valid;
        uint8_t data[FeedbackSize];
    } feedback;

    void generate_feedback(flasher::State state, flasher::Result result, uint32_t arg) {
        uint8_t *ptr = feedback.data;
        std::memcpy(ptr, &state, sizeof(state));
        ptr += sizeof(state);
        std::memcpy(ptr, &result, sizeof(result));
        ptr += sizeof(result);
        std::memcpy(ptr, &arg, sizeof(arg));
        // this works because poll_feedback can't interrupt us
        feedback.updated.store(true);
    }

    bool poll_feedback(uint32_t now, uint8_t *out) {
        if (feedback.updated.load() ||
            (feedback.valid && now - feedback.last_sent >= 500)) {
            feedback.valid = true;
            do {
                feedback.updated.store(false);
                std::memcpy(out, feedback.data, sizeof(feedback.data));
                // this works because we cannot interrupt generate_feedback
            } while (feedback.updated.load());
            feedback.last_sent = now;
            return true;
        }

        return false;
    }
}

namespace flasher {
void flasher_state_callback(flasher::State state, flasher::Result result, uint32_t arg) {
    can_flasher::generate_feedback(state, result, arg);
}
}
