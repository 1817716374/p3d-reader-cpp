#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct CsgMeshBooleanOptions {
    // Additional positional tolerance for derived mesh operations, in the input
    // coordinates' units. This is NOT the P3D angular tessellation tolerance.
    double tolerance = 0;
    std::size_t max_output_triangles = 3000000;
};
struct CsgTriangleSource {
    unsigned operand = 0; // 0: left, 1: right; no source identity deduplication.
    std::size_t face_index = 0;
    bool backside = false;
    // Dominant-axis projection weights in the original source triangle plane.
    // Use these for linear corner attributes; projection_distance records the
    // positional discrepancy caused by rounding or kernel tolerance.
    std::array<std::array<double, 3>, 3> corner_barycentric{};
    std::array<double, 3> corner_projection_distance{};
};
struct CsgMeshBooleanResult {
    std::string status = "not_evaluated";
    std::vector<Point3> vertices;
    std::vector<Triangle> faces;
    std::vector<CsgTriangleSource> face_sources;
    Json diagnostics = Json::object();
};
// Boolean operation on two already constructed, outward-oriented closed
// triangle meshes: native operation codes 0=union, 1=intersection, 2=difference.
// This derived mesh kernel does not perform native CSG tree update, solid
// tessellation, topology repair, or final material selection. The input Geometry
// objects remain unchanged; every output face identifies its source operand
// and face, so attributes and material associations can be recovered explicitly.
CsgMeshBooleanResult evaluate_csg_mesh_boolean(const Geometry &left, const Geometry &right,
                                               std::int32_t operation,
                                               const CsgMeshBooleanOptions &options = {});
} // namespace p3d
