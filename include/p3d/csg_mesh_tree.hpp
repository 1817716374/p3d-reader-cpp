#pragma once
#include <p3d/csg_mesh.hpp>
#include <p3d/polyface.hpp>

namespace p3d {
struct CsgMeshTreeOptions {
    CsgMeshListOptions boolean;
    std::size_t max_nodes = 100000;
    std::size_t max_cached_triangles = 3000000;
    std::size_t max_cached_meshes = 100000;
    std::size_t max_vertex_transforms = 10000000;
    std::size_t max_archive_depth = 32; // Root is depth zero; hard ceiling 128.
    std::size_t max_node_updates = 1000000;
    std::size_t max_geometry_visits = 1000000;
};
struct CsgTreeTriangleSource {
    std::size_t geometry_index = 0, face_index = 0;
    Matrix4 source_to_result{};
    bool backside = false; // Boolean reversal, excluding placement handedness.
    std::array<std::array<double, 3>, 3> corner_barycentric{};
    std::array<double, 3> corner_projection_distance{};
};
struct CsgTreeMesh {
    std::vector<Point3> vertices;
    std::vector<Triangle> faces;
    std::vector<CsgTreeTriangleSource> face_sources;
};
struct CsgMeshTreeResult {
    std::string status = "not_evaluated";
    std::vector<CsgTreeMesh> meshes;
    Json updated_nodes = Json::array();
    Json diagnostics = Json::object();
};
// Executes the decoded tree's update and root conversion for an archive whose
// original geometry entries are all native Polyface objects. Supply one already
// triangulated mesh per archive.geometries entry, in that exact index order.
// Source arrays and the archive stay unchanged. Attribute pools remain in the
// supplied Geometry objects; result faces link to their original triangles.
// Original cache objects are discarded by update. Shared source/cache references
// and successive in-place transforms are preserved in a private runtime state.
// Nested CSG, solids needing tessellation, and other source kinds are explicitly
// unsupported here; a successful derived result does not prove native kernel
// equivalence, cache cardinality, or final material selection.
CsgMeshTreeResult evaluate_csg_polyface_tree(const Json &archive,
                                             const std::vector<Geometry> &source_meshes,
                                             const CsgMeshTreeOptions &options = {});
struct CsgPolyfaceArchiveResult {
    CsgMeshTreeResult result;
    // Source tables, attribute pools and polygon/corner links for interpreting
    // result face geometry_index/face_index. Binding status is independent of
    // geometry evaluation status; no final material selection is implied.
    std::vector<PolyfaceMeshResult> sources;
    // One path per source: [{"list":"geometries"|"node_caches","index":N},...].
    // Intermediate steps enter nested archives; the last step names the Polyface.
    // Result geometry_index addresses the flattened sources vector, not the
    // outer archive's geometry list. Without nesting these indices are unchanged.
    std::vector<Json> source_paths;
};
// Reads supported BGFB Polyface sources and nested CSG archives. Nested objects
// are placed, updated and expanded each time the native conversion visits them;
// they are not precomputed once. Budgets are shared across the whole hierarchy.
CsgPolyfaceArchiveResult
evaluate_csg_polyface_archive(const Json &archive, const CsgMeshTreeOptions &tree_options = {},
                              const PolyfaceMeshOptions &mesh_options = {});
} // namespace p3d
