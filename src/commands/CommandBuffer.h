#pragma once

#include <cstdint>

// TODO: multiple commands in buffer
class CommandBuffer
{
public:
    static constexpr int BUFFER_SIZE = 1024;

public:
    CommandBuffer();

    bool writeByte(uint8_t byte);

    [[nodiscard]] const char *getCurrentCommand() const;

    void flushCurrentCommand();

private:
    uint8_t buffer_[BUFFER_SIZE]{};
    uint8_t *buf_end_{};
    uint8_t *cur_pos_{};
    uint8_t *cur_command_start_{};
};
