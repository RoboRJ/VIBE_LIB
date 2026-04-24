#pragma once

#include <cstdint>

namespace studentlib {

using timestamp_ms_t = std::uint32_t;

enum class AxisType {
    LONGITUDINAL,
    LATERAL
};

}  // namespace studentlib