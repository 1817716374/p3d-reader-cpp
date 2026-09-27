#pragma once
#include "internal.hpp"

namespace p3d::spiral_detail {
// Confirmed native buffer/count guards, distinct from numerical failure or a
// caller's integration budget. The enclosing curve wrapper survives these.
struct FitRejected : std::runtime_error {
    FitRejected(const char *message, const char *guard)
        : std::runtime_error(message), reason(guard) {}
    const char *reason;
};
} // namespace p3d::spiral_detail
