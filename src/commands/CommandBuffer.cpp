#include "CommandBuffer.h"

CommandBuffer::CommandBuffer()
{
    cur_pos_ = buffer_;
    buf_end_ = cur_pos_ + BUFFER_SIZE;
}

bool CommandBuffer::writeByte(uint8_t byte)
{
    if (cur_command_start_ != nullptr)
    {
        return false;
    }
    if (cur_pos_ == buf_end_)
    {
        return false;
    }
    *cur_pos_ = byte;
    if (byte == '\n' || byte == '\r')
    {
        cur_command_start_ = buffer_;
        *cur_pos_ = '\0';
    }
    ++cur_pos_;
    return true;
}

const char *CommandBuffer::getCurrentCommand() const
{
    return reinterpret_cast<const char *>(cur_command_start_);
}

void CommandBuffer::flushCurrentCommand()
{
    cur_pos_ = buffer_;
    cur_command_start_ = nullptr;
}
