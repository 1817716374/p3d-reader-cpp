#pragma once
#include <functional>
#include <p3d/reader.hpp>
#include <stdexcept>

namespace p3d {
// Nested decoders must not swallow shared resource limits as local diagnostics.
struct DecodeBudgetExceeded : std::runtime_error {
    using std::runtime_error::runtime_error;
};
Json decode_bgfb(const Bytes &, const std::function<void(std::size_t)> &);
Json decode_csg_bytes_budgeted(const Bytes &, std::size_t, std::size_t,
                               const std::function<void(std::size_t)> &);
} // namespace p3d
