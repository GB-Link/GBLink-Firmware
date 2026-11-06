#include <algorithm>
#include <array>
#include <concepts>
#include <cstdint>
#include <span>
#include <stdint.h>

#pragma once

class TxValues
{
public:

    TxValues() = default;

    template<std::convertible_to<uint16_t>... Ts>
        requires (sizeof...(Ts) <= 3)
    TxValues(Ts... values)
        : m_count(sizeof...(Ts)), m_values{static_cast<uint16_t>(values)...}
    {}

    uint8_t size() { return m_count; }

    void push(uint16_t value) 
    { 
        m_values[m_count] = value;
        m_count++;    
    }

    const uint16_t& operator[](size_t index) {return m_values[index]; }

    std::span<const uint16_t> span() { return std::span(m_values).first(m_count); }

    auto begin() { return m_values.begin(); }

    auto end() { return m_values.begin() + m_count; }

    auto begin() const { return m_values.cbegin(); }

    auto end() const { return m_values.cbegin() + m_count; } 

private:

    uint8_t m_count = 0;
    std::array<uint16_t, 3> m_values = {};
};
