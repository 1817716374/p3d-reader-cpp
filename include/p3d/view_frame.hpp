#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
namespace p3d {
struct NativeViewFrameInput {
    // Two opposite range corners: low XYZ, high XYZ. No units conversion.
    std::array<double, 6> range{};
    // Row-major matrix. No matrix selects identity.
    std::optional<std::array<double, 9>> orientation;
    // Result of the model-directory predicate in 4b7550. True preserves a
    // spatial matrix; false reduces nonidentity input to a rotation about Z.
    std::optional<bool> preserve_spatial_orientation;
    // left, top, right, bottom in native signed 32-bit viewport coordinates.
    std::optional<std::array<std::int32_t, 4>> viewport;
    std::int32_t slot = 0;
};
struct NativeViewFrameResult {
    bool resolved = false;
    std::string reason;
    std::array<double, 3> origin{};
    std::array<double, 3> delta{};
    std::array<double, 9> orientation{};
    double half_depth = 0;
    std::uint32_t slot_index = 0;
    bool orientation_reduced = false;
};
// Numeric framing stage of R1.18 4b7550 for finite, non-disconnect inputs.
// Does not obtain the model range, bind a model, choose flags, or apply a GUI.
// Consume numeric fields only when resolved is true; otherwise they may be partial.
NativeViewFrameResult project_native_view_frame(const NativeViewFrameInput &input);
} // namespace p3d
