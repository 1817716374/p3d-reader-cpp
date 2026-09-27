#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct SweptBodyOptions {
    std::size_t max_control_points = 1000000;
    std::size_t max_work = 10000000;
    std::size_t max_faces = 1000000;
};
struct SweptBodySurface {
    // Derived BsplineSurface table. Its null BGFB boundaries field does NOT
    // mean untrimmed: generated runtime trims are in the two arrays below.
    Json geometry;
    std::vector<std::vector<Point2>> boundary_points;
    // Parallel to boundary_points when nonempty; empty means all null lists.
    // These are parameter-space curves, not texture coordinates.
    std::vector<std::vector<BsplineCurve>> boundary_curves;
    std::array<std::size_t, 3> working_location{}; // group, member, working surface
    std::size_t source_member = 0;                 // partitioned member before orientation reversal
};
enum class SweptBodyFaceKind { side, cap };
struct SweptBodyFace {
    std::array<std::int64_t, 3> indices{}; // Native GeFaceIndices, not material IDs.
    SweptBodyFaceKind kind = SweptBodyFaceKind::side;
    std::size_t index = 0; // surfaces[index] or caps[index], according to kind
    // Side position in groups; absent for cap regions.
    std::optional<std::array<std::size_t, 3>> location;
};
struct SweptBodyResult {
    Json source;
    // reconstructed / native_failure / not_reconstructed. Native success is
    // not a proof of closed, manifold or self-intersection-free geometry.
    std::string status = "not_reconstructed";
    std::vector<SweptBodySurface> surfaces;
    // [profile group][member][patch] -> surfaces. Native identities only;
    // geometrically equal independent surfaces are never deduplicated.
    std::vector<std::vector<std::vector<std::size_t>>> groups;
    std::vector<Json> caps;           // Ordered start/end CurveVector regions, not meshes.
    std::vector<SweptBodyFace> faces; // Caps first, then native global side order.
    Json report = Json::object();
    // Exact three-component lookup. Returns nullptr for missing IDs or failed
    // reconstruction. Pointer lifetime follows this result's faces vector.
    const SweptBodyFace *find_face(const std::array<std::int64_t, 3> &) const noexcept;
};
// Reconstruct the supported native grouped-facet route from a decoded
// P3DSweptBody table (profile, path, optional capped=false). Owns all output;
// source stays unchanged and separate calls share no mutable state.
// Native rejection retains completed side groups but publishes no face IDs or
// caps. Unsupported layouts, malformed data and exhausted budgets return
// not_reconstructed with a reason and no derived output. Memory allocation
// failure propagates. Runtime trim arrays must accompany their surface table.
// No triangulation, material assignment or native planar-face conversion.
SweptBodyResult reconstruct_bgfb_swept_body(const Json &, const SweptBodyOptions & = {});
} // namespace p3d
