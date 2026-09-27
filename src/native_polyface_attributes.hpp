#pragma once
#include "native_polyface_face_data.hpp"

namespace p3d::swept_detail {
struct NativePolyfaceAttributes {
    NativePolyfaceFaceDataState output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Indexed query route. Replaces the selected pool and index array. No raw
// mesh-style conversion or approximate/smoothed normal generation is implied.
// Failed face frames leave normal indices zero; UV generation writes distinct
// zero-valued UV entries for every corner of a failed frame, as in the original.
NativePolyfaceAttributes build_native_polyface_normals(const NativePolyfaceFaceDataState &,
                                                       TubeBudget &);
NativePolyfaceAttributes build_native_polyface_parameters(const NativePolyfaceFaceDataState &,
                                                          int coordinate_selector, TubeBudget &);
} // namespace p3d::swept_detail
