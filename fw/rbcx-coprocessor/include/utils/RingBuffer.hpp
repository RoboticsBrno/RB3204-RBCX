#pragma once

#include <array>
#include <stddef.h>

template <typename T, size_t Size> class RingBuffer {
    static_assert(Size > 0);
    static_assert(std::is_trivial<T>::value);

public:
    RingBuffer()
        : m_data {}
        , m_head(0) {}

    size_t size() const { return Size; }

    void push(const T& val) {
        m_data[m_head] = val;
        if (++m_head == Size) {
            m_head = 0;
        }
    }

    const T& at(size_t idx) const { return m_data.at(convertIdx(idx)); }
    T& at(size_t idx) { return m_data.at(convertIdx(idx)); }

    const T& operator[](size_t idx) const { return at(idx); }
    T& operator[](size_t idx) { return at(idx); }

private:
    size_t convertIdx(size_t idx) const { return (idx + m_head) % Size; }

    std::array<T, Size> m_data;
    size_t m_head;
};
