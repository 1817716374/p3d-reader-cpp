#include <p3d/view_frame.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
namespace p3d {
NativeViewFrameResult project_native_view_frame(const NativeViewFrameInput &in) {
    NativeViewFrameResult out;
    const std::array<double, 9> identity{1,0,0,0,1,0,0,0,1};
    for (double value : in.range)
        if (!std::isfinite(value) || value == std::numeric_limits<double>::max()) {
            out.reason = "nonfinite_or_disconnect_view_range"; return out;
        }
    out.orientation = in.orientation.value_or(identity);
    bool near_identity = true;
    for (std::size_t i = 0; i < 9; ++i) {
        if (!std::isfinite(out.orientation[i])) { out.reason = "nonfinite_view_orientation"; return out; }
        near_identity &= std::abs(out.orientation[i] - identity[i]) <= 1e-12;
    }
    if (in.orientation && !near_identity) {
        if (!in.preserve_spatial_orientation) { out.reason = "model_directory_orientation_predicate_not_established"; return out; }
        if (!*in.preserve_spatial_orientation) {
            const double x = out.orientation[0], y = out.orientation[3];
            const double angle = (x == 0 && y == 0) ? 0 : std::atan2(y, x);
            const double c = std::cos(angle), s = std::sin(angle);
            out.orientation = {c,-s,0,s,c,0,0,0,1}; out.orientation_reduced = true;
        }
    }
    const auto &m = out.orientation;
    std::array<double, 3> low{}, high{};
    for (unsigned corner = 0; corner < 8; ++corner) {
        std::array<double, 3> point{};
        for (unsigned j = 0; j < 3; ++j) point[j] = in.range[j + ((corner & (1u << j)) ? 3 : 0)];
        for (unsigned j = 0; j < 3; ++j) {
            const double value = (m[3*j] * point[0] + m[3*j+1] * point[1]) + m[3*j+2] * point[2];
            if (!std::isfinite(value)) { out.reason = "nonfinite_projected_view_range"; return out; }
            if (!corner) low[j] = high[j] = value;
            else { low[j] = std::min(low[j], value); high[j] = std::max(high[j], value); }
        }
    }
    for (unsigned j = 0; j < 3; ++j) out.delta[j] = high[j] - low[j];
    if (in.viewport) {
        const auto &v = *in.viewport;
        auto dimension = [](std::int32_t first, std::int32_t last) {
            if (last <= first) return 1.0;
            const auto bits = static_cast<std::uint32_t>(last) - static_cast<std::uint32_t>(first);
            const auto signed_value = bits <= INT32_MAX ? static_cast<std::int64_t>(bits) :
                                      static_cast<std::int64_t>(bits) - INT64_C(0x100000000);
            return static_cast<double>(signed_value);
        };
        const double width = dimension(v[0], v[2]), height = dimension(v[1], v[3]);
        const double x_times_height = out.delta[0] * height, y_times_width = out.delta[1] * width;
        if (x_times_height > y_times_width) {
            const double y = x_times_height / width;
            low[1] -= (y - out.delta[1]) * 0.5; out.delta[1] = y;
        } else {
            const double x = y_times_width / height;
            low[0] -= (x - out.delta[0]) * 0.5; out.delta[0] = x;
        }
        for (unsigned j = 0; j < 2; ++j) {
            const double margin = out.delta[j] * 0.025;
            low[j] -= margin; out.delta[j] += margin + margin;
        }
    }
    // Original GeomBase ordinal 4627 multiplies by the transpose, not a
    // general inverse. Preserve that behavior for scale/shear matrices too.
    for (unsigned j = 0; j < 3; ++j)
        out.origin[j] = (m[j] * low[0] + m[3+j] * low[1]) + m[6+j] * low[2];
    out.half_depth = out.delta[2] * 0.5;
    out.slot_index = static_cast<std::uint32_t>(in.slot) < 8 ? static_cast<std::uint32_t>(in.slot) : 0;
    for (unsigned j = 0; j < 3; ++j)
        if (!std::isfinite(out.origin[j]) || !std::isfinite(out.delta[j])) {
            out.reason = "nonfinite_view_frame_result"; return out;
        }
    out.resolved = true; return out;
}
} // namespace p3d
