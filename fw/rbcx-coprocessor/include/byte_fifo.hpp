#pragma once

#include <array>
#include <algorithm>
#include <stdint.h>

class ByteFifo
{
    std::array<uint8_t, 16> m_fifo;
    int m_head, m_tail;

public:
    ByteFifo(): m_head(0), m_tail(0) {}

    uint8_t *data() const { return (uint8_t*)m_fifo.data(); }
    size_t size() const { return m_fifo.size(); }

    std::pair<uint8_t *, size_t> readable_range() const
    {
        return m_head >= m_tail
            ? std::make_pair(data() + m_tail, m_head - m_tail)
            : std::make_pair(data() + m_tail, int(size()) - m_tail);
    }
    
    std::pair<uint8_t *, size_t> writeable_range() const
    {
        return m_head >= m_tail
            ? std::make_pair(data() + m_head, int(size()) - m_head)
            : std::make_pair(data() + m_head, m_tail - m_head);
    }

    void set_head(int newHead) { m_head = newHead; }

    void notify_written(size_t len)
    {
        m_head += len;
        if (m_head >= int(size()))
        {
            m_head -= size();
        }
    }

    void notify_read(size_t len)
    {
        m_tail += len;
        if (m_tail >= int(size()))
        {
            m_tail -= size();
        }
    }

    void write_range(uint8_t *data, size_t len)
    {
        std::copy_n(data, len, this->data() + m_head);
        notify_written(len);
    }

    void read_range(uint8_t *data, size_t len)
    {
        std::copy_n(this->data() + m_tail, len, data);
        notify_read(len);
    }
};
