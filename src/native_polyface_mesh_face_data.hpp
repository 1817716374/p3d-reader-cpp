#pragma once
#include "native_polyface_face_data.hpp"
#include "native_polyface_visitor.hpp"
namespace p3d::swept_detail {
struct NativePolyfaceMeshFaceData {
    NativePolyfaceMesh output;
    bool complete = false;
    Json report;
};
// Typed original SetNewFace/BuildPerFaceFaceData. Read positions follow the
// source style, without conversion. Preserve metadata and native flags; reject
// unsafe array reads. Native build success does not imply full face coverage.
NativePolyfaceMeshFaceData
set_native_polyface_mesh_face_data(const NativePolyfaceMesh &,
                                   const std::optional<NativeBuilderFaceData> &,
                                   std::size_t end_index, TubeBudget &);
NativePolyfaceMeshFaceData build_native_polyface_mesh_face_data(const NativePolyfaceMesh &,
                                                                TubeBudget &);
} // namespace p3d::swept_detail
