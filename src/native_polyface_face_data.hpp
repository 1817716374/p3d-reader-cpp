#pragma once
#include "native_builder_polyface.hpp"

namespace p3d::swept_detail {
struct NativePolyfaceFaceDataState {
    // Indexed-face query data. num_per_face > 1 selects fixed-size blocks;
    // 0 and 1 use zero-delimited faces in the original visitor.
    NativeBuilderPolyface mesh;
    bool parameter_pool_active = false, face_data_pool_active = false;
    bool normal_pool_active = false;
};
struct NativePolyfaceFaceDataResult {
    NativePolyfaceFaceDataState output;
    bool complete = false;
    Json report;
};
NativeBuilderFaceData native_null_face_data();
// Native SetNewFace on indexed query data; end_index=0 means the full point
// index array. The optional record supplies metadata and existing UV ranges;
// XYZ and normal ranges are recomputed. Input remains immutable on all paths.
NativePolyfaceFaceDataResult
set_native_polyface_face_data(const NativePolyfaceFaceDataState &,
                              const std::optional<NativeBuilderFaceData> &, std::size_t end_index,
                              TubeBudget &);
// Native BuildPerFaceFaceData. UV-bearing input groups all remaining facets;
// input without UV gets a record per visited face. No raw mesh-style conversion,
// UV synthesis, normal generation or solid-completeness claim is implied.
NativePolyfaceFaceDataResult build_native_polyface_face_data(const NativePolyfaceFaceDataState &,
                                                             TubeBudget &);
} // namespace p3d::swept_detail
