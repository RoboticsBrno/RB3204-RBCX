#pragma once

#include <algorithm>
#include <array>
#include <stdint.h>

template <int Size>
class ByteFifo {
    std::array<uint8_t, Size> m_fifo;
    int m_head, m_tail;

    int static adjust(int index, int delta) {
        index += delta;
        if (index >= Size) {
            index -= Size;
        }
        if (index < 0) {
            index += Size;
        }
        return index;
    }

public:
    ByteFifo()
        : m_head(0)
        , m_tail(0) {}

    uint8_t* data() const { return (uint8_t*)m_fifo.data(); }
    size_t size() const { return m_fifo.size(); }

    void setHead(int newHead) {
        m_head = newHead;
    }

    std::pair<uint8_t*, size_t> readableRange() const {
        return m_head >= m_tail
            ? std::make_pair(data() + m_tail, m_head - m_tail)
            : std::make_pair(data() + m_tail, int(size()) - m_tail);
    }

    std::pair<uint8_t*, size_t> writeableRange() const {
        int preTail = adjust(m_tail, -1);
        return m_head >= preTail
            ? std::make_pair(data() + m_head, std::max(0, int(size()) - m_head))
            : std::make_pair(data() + m_head, std::max(0, preTail - m_head));
    }

    void notifyWritten(size_t len) {
        m_head = adjust(m_head, len);
    }

    void notifyRead(size_t len) {
        m_tail = adjust(m_tail, len);
    }

    void writeRange(uint8_t* data, size_t len) {
        std::copy_n(data, len, this->data() + m_head);
        notifyWritten(len);
    }

    void readRange(uint8_t* data, size_t len) {
        std::copy_n(this->data() + m_tail, len, data);
        notifyRead(len);
    }
};
