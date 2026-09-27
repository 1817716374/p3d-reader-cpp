#pragma once
#include "internal.hpp"

namespace p3d::interpolation_detail {
// Only confirmed native preparation guards use this exception. Numerical
// failures remain ordinary errors and do not prove a missing native cache.
struct FitRejected : std::runtime_error {
    FitRejected(const char *message, const char *guard)
        : std::runtime_error(message), reason(guard) {}
    const char *reason;
};
} // namespace p3d::interpolation_detail
