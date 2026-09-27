#pragma once
#include "native_polyface_copy.hpp"

namespace p3d::swept_detail {
struct NativePolyfacePreparationOptions {
    bool normals_required = true, parameters_required = true;
    bool edge_chains_required = false, convex_facets_required = false;
    bool hide_smooth_edges = false;
    // Original 32-bit setting: comparison is signed, and the triangulator
    // receives a sign-extended size_t. Zero does not mean unlimited.
    std::uint32_t max_edges_per_face = 3;
    std::int32_t parameter_mode = 0;
    std::size_t draw_method_index = 0;
};
struct NativePolyfacePreparation {
    NativePolyfaceMesh output;
    bool copied = false, complete = false;
    Json report;
};
struct NativePreparedPolyfaceAssembly {
    NativeBuilderPolyfaceOutput assembled;
    std::vector<NativePolyfacePreparation> sources;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Original outer preparation on styles 1/3/4/5/6 through their own visitors.
// No silent preconversion: only the original conditional triangulation converts.
// Native failures in a preparation step do not suppress subsequent steps.
NativePolyfacePreparation
prepare_native_polyface_for_builder(const NativePolyfaceMesh &,
                                    const NativePolyfacePreparationOptions &, TubeBudget &);
// Sources already have placement applied. Coordinate map settings belong to
// the builder; the per-source coordinate batch retains its caller-owned scope
// and normal transform controls even when native query-copy is needed.
// Prepared sources are returned separately. Untransferred raw layouts or color/extensions
// prevent complete; values omitted by native query-copy remain in the unchanged
// caller-owned inputs and are diagnosed by that preparation step.
NativePreparedPolyfaceAssembly
assemble_native_prepared_polyfaces(const std::vector<NativePolyfaceMesh> &,
                                   const NativePolyfacePreparationOptions &,
                                   const NativeBuilderCoordinateOptions &, TubeBudget &);
} // namespace p3d::swept_detail
