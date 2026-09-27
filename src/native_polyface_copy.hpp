#pragma once
#include "native_polyface_visitor.hpp"
namespace p3d::swept_detail {
struct NativePolyfaceCopy {
    NativePolyfaceMesh output;
    bool complete = false;
    Json report;
};
// Zero-initialized handle, native constructor and ClearTags defaults.
NativePolyfaceMesh make_native_polyface_mesh(std::uint32_t num_per_face = 0,
                                             std::uint32_t mesh_style = 1);
// Native query-copy into an existing destination snapshot. Source vector tags,
// row widths and activity are not assigned wholesale. Empty copied buffers do
// not deactivate a previously active destination. Texture ID stays destination's.
// complete diagnoses omitted source values/layout metadata; it is not mesh validity.
NativePolyfaceCopy copy_native_polyface_query(const NativePolyfaceMesh &source,
                                              const NativePolyfaceMesh &destination, TubeBudget &);
} // namespace p3d::swept_detail
