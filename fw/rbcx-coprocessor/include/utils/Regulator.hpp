#pragma once

#include <stdint.h>

template <class value_type> struct Regulator {
    typedef uint32_t coef_type;
    typedef int32_t int_type;

    constexpr Regulator<value_type>& operator=(Regulator<value_type>&& other)
        = default;

    Regulator() = default;
    Regulator(const value_type& max_output, const coef_type& p,
        const coef_type& i, const coef_type& d)
        : m_w(0)
        , m_stop(false)
        , m_p(p)
        , m_i(i)
        , m_d(d)
        , m_e(0)
        , m_le(0)
        , m_de(0)
        , m_x(0)
        , m_integrator(0)
        , m_max_output(max_output) {}

    void operator()(value_type v) { m_w = v; }
    value_type operator()() { return m_w; }

    void P(const coef_type& v) { m_p = v; }
    coef_type P() const { return m_p; }
    void I(const coef_type& v) { m_i = v; }
    coef_type I() const { return m_i; }
    void D(const coef_type& v) { m_d = v; }
    coef_type D() const { return m_d; }

    void stop(bool s = true) {
        m_stop = s;
        m_w = 0;
        m_e = 0;
        m_le = 0;
        m_de = 0;
        m_integrator = 0;
    }

    value_type process(value_type y) {
        if (m_stop)
            return 0;
        m_e = m_w - y; //m_w - required  value, y - encoder position/distance

        int_type integrator = m_integrator;

        if (m_i != 0)
            integrator += m_e;

        m_de = m_e - m_le;

        int_type x = ((int_type(m_p) * m_e) >> 8)
            + ((int_type(m_d) * m_de) >> 8)
            + ((int_type(m_i) * integrator) >> 8);

        if (x > m_max_output) {
            m_x = m_max_output;
            if (integrator < 0)
                m_integrator = integrator;
        } else if (x < -m_max_output) {
            m_x = -m_max_output;
            if (integrator > 0)
                m_integrator = integrator;
        } else {
            m_x = x;
            m_integrator = integrator;
        }
        m_le = m_e;
        return m_x;
    }

    void clear() {
        m_integrator = 0;
        m_le = m_e;
        m_w = 0;
    }

    value_type e() const { return m_e; }
    value_type de() const { return m_de; }
    value_type x() const { return m_x; }
    int_type integrator() const { return m_integrator; }

private:
    value_type m_w;
    bool m_stop;
    coef_type m_p;
    coef_type m_i;
    coef_type m_d;
    value_type m_e;
    value_type m_le;
    value_type m_de;
    value_type m_x;
    int_type m_integrator;
    value_type m_max_output; //crop output, absolute value
};
