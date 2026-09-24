#include <p3d/csg_mesh_tree.hpp>
#include "geometry.hpp"

namespace {
using namespace p3d;
Geometry cube(double x) {
    Geometry g;
    g.vertices = {{x, 0, 0}, {x + 2, 0, 0}, {x + 2, 2, 0}, {x, 2, 0},
                  {x, 0, 2}, {x + 2, 0, 2}, {x + 2, 2, 2}, {x, 2, 2}};
    g.faces = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
               {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    return g;
}
Json node(int operation, Json geometry = Json::array(), Json left = nullptr, Json right = nullptr,
          Json matrices = Json::array()) {
    return {{"status", "decoded"},          {"operation", operation},
            {"geometry_indices", geometry}, {"cache_indices", Json::array()},
            {"matrix_indices", matrices},   {"is_old_value", 0},
            {"left_index", left},           {"right_index", right}};
}
Json archive(Json nodes, std::size_t count) {
    Json out = {{"tree", {{"nodes", nodes}, {"root_index", nodes.empty() ? Json() : Json(0)}}},
                {"geometries", Json::array()},
                {"node_caches", Json::array()},
                {"transforms", Json::array()},
                {"angle_tolerance", 0}};
    for (std::size_t i = 0; i < count; ++i)
        out["geometries"].push_back({{"encoding", "bgfb"},
                                     {"status", "decoded"},
                                     {"geometry", {{"geometry", {{"_type", "Polyface"}}}}}});
    for (double x : {1., 2., 100.})
        out["transforms"].push_back(
            {{"matrix_3x4_rows", {{1., 0., 0., x}, {0., 1., 0., 0.}, {0., 0., 1., 0.}}}});
    return out;
}
double volume(const CsgTreeMesh &g) {
    double v = 0;
    for (const auto &t : g.faces) {
        const auto &a = g.vertices[t[0]], &b = g.vertices[t[1]], &c = g.vertices[t[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return v / 6;
}
} // namespace
unsigned csg_mesh_tree_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto validate = [&](const CsgMeshTreeResult &result, const std::vector<Geometry> &sources,
                        const std::vector<double> &volumes) {
        check(result.status == "evaluated", result.diagnostics.dump().c_str());
        check(result.meshes.size() == volumes.size(), "CSG tree root output count");
        for (std::size_t i = 0; i < volumes.size(); ++i) {
            const auto &g = result.meshes[i];
            check(std::abs(volume(g) - volumes[i]) < 1e-8,
                  "CSG tree independently computed volume");
            check(g.faces.size() == g.face_sources.size(), "CSG tree source face correspondence");
            for (std::size_t f = 0; f < g.faces.size(); ++f) {
                const auto &s = g.face_sources[f];
                check(s.geometry_index < sources.size(), "CSG original geometry identity");
                const auto &original = sources.at(s.geometry_index);
                check(s.face_index < original.faces.size(), "CSG original triangle identity");
                for (unsigned k = 0; k < 3; ++k) {
                    Point3 p{};
                    double sum = 0;
                    for (unsigned j = 0; j < 3; ++j) {
                        sum += s.corner_barycentric[k][j];
                        for (unsigned axis = 0; axis < 3; ++axis)
                            p[axis] += s.corner_barycentric[k][j] *
                                       original.vertices[original.faces[s.face_index][j]][axis];
                    }
                    p = transform(s.source_to_result, p);
                    check(std::abs(sum - 1) < 1e-9, "CSG tree original weights partition unity");
                    check(s.corner_projection_distance[k] < 1e-8,
                          "CSG tree source projection bounded");
                    for (unsigned axis = 0; axis < 3; ++axis)
                        check(std::abs(p[axis] - g.vertices[g.faces[f][k]][axis]) < 1e-8,
                              "CSG source weights and placement recover world corner");
                }
            }
        }
    };
    const auto a = cube(0), b = cube(1);
    auto tree = archive({node(0, {0}, nullptr, nullptr, {0})}, 1);
    auto result = evaluate_csg_polyface_tree(tree, {a});
    validate(result, {a}, {8});
    check(result.meshes[0].vertices[0][0] == 1, "leaf transform applied once");
    check(result.updated_nodes[0]["is_old_value"] == 0 &&
              result.updated_nodes[0]["cache_indices"] == Json({0}),
          "leaf rebuild replaces caches and clears isOld");
    for (int op = 0; op < 3; ++op) {
        tree = archive({node(op, Json::array(), 1, 2), node(4, {0}), node(4, {1})}, 2);
        validate(evaluate_csg_polyface_tree(tree, {a, b}), {a, b}, {op == 0 ? 12. : 4.});
    }
    // Same source becomes the left leaf's cache, then both parent operands.
    // Native Polyface pointers alias: +1 at leaf, +1 at left conversion,
    // +2 at right conversion. Both operands finally occupy x=[4,6].
    tree = archive({node(0, Json::array(), 1, 2, {2}), node(0, {0}, nullptr, nullptr, {0}),
                    node(4, {0}, nullptr, nullptr, {1})},
                   1);
    const auto before = tree;
    result = evaluate_csg_polyface_tree(tree, {a});
    validate(result, {a}, {8});
    for (const auto &p : result.meshes[0].vertices)
        check(p[0] >= 4 && p[0] <= 6, "CSG source/cache alias sees both parent operand transforms");
    check(tree == before, "tree evaluation never rewrites the source archive");
    check(result.meshes[0].face_sources[0].source_to_result[0][3] == 4,
          "root's own transform is not an extra scene placement");
    // Duplicate source indices trigger repeated in-place placement.
    tree = archive({node(0, {0, 0}, nullptr, nullptr, {0})}, 1);
    result = evaluate_csg_polyface_tree(tree, {a});
    validate(result, {a}, {8});
    check(result.meshes[0].face_sources[0].source_to_result[0][3] == 2,
          "duplicate native references are not transformed as independent copies");
    // Compound collects original indices preferentially, not child caches.
    tree = archive({node(3, Json::array(), 1, 2), node(0, {0}, nullptr, nullptr, {0}),
                    node(4, {1}, nullptr, nullptr, {1})},
                   2);
    result = evaluate_csg_polyface_tree(tree, {a, cube(10)});
    validate(result, {a, cube(10)}, {8, 8});
    check(result.meshes[0].vertices[0][0] == 1 && result.meshes[1].vertices[0][0] == 10,
          "Compound does not apply child matrices as scene placements");
    check(result.updated_nodes[0]["geometry_indices"] == Json({0, 1}) &&
              result.updated_nodes[0]["cache_indices"].empty(),
          "Compound source priority");
    // Original cache indices can name objects generated by an earlier sibling.
    tree = archive({node(0, Json::array(), 1, 2), node(0, {0}), node(1, {1})}, 2);
    tree["node_caches"] = {{{"opaque_old_cache", true}}};
    tree["tree"]["nodes"][2]["cache_indices"] = {0};
    result = evaluate_csg_polyface_tree(tree, {a, b});
    validate(result, {a, b}, {8});
    check(result.diagnostics["boolean_operations"] == 2,
          "leaf selection uses rebuilt current cache pool");
    // A reflection reverses triangle order, with corresponding source weights.
    auto reflected = archive({node(0, {0}, nullptr, nullptr, {0})}, 1);
    reflected["transforms"][0]["matrix_3x4_rows"] = {
        {-1., 0., 0., 0.}, {0., 2., 0., 0.}, {0., 0., 1., 0.}};
    result = evaluate_csg_polyface_tree(reflected, {a});
    validate(result, {a}, {16});
    check(result.meshes[0].faces[0] == Triangle({0, 1, 2}), "reflection corrects winding");
    // Reconstruct provenance through a Boolean child, its subsequent placement,
    // and a second Boolean at the parent. Cutter and child faces have different
    // source-to-result matrices and must never be assigned one global placement.
    auto hierarchy = archive({node(2, Json::array(), 1, 2), node(0, Json::array(), 3, 4, {0}),
                              node(4, {2}), node(4, {0}), node(4, {1})},
                             3);
    const std::vector<Geometry> pool{a, b, cube(2.5)};
    const auto vertices_before = pool[0].vertices;
    result = evaluate_csg_polyface_tree(hierarchy, pool);
    validate(result, pool, {6});
    bool has_cutter = false, has_placed_child = false;
    for (const auto &face : result.meshes[0].face_sources) {
        if (face.geometry_index == 2) {
            has_cutter = true;
            check(face.backside && face.source_to_result[0][3] == 0,
                  "parent cutter keeps its own original coordinate frame");
        } else {
            has_placed_child = true;
            check(face.source_to_result[0][3] == 1,
                  "placed Boolean child retains original source transform");
        }
    }
    check(has_cutter && has_placed_child && pool[0].vertices == vertices_before,
          "multilevel Boolean preserves separate sources and immutable caller geometry");
    auto unsupported = tree;
    unsupported["geometries"][0]["geometry"]["geometry"]["_type"] = "DgnBox";
    check(evaluate_csg_polyface_tree(unsupported, {a, b}).status != "evaluated",
          "pretriangulated meshes cannot disguise unresolved solid transformation semantics");
    auto cyclic = tree;
    cyclic["tree"]["nodes"][0]["left_index"] = 0;
    check(evaluate_csg_polyface_tree(cyclic, {a, b}).meshes.empty(),
          "cyclic tree rejected without recursion");
    auto shared = tree;
    shared["tree"]["nodes"][0]["right_index"] = 1;
    check(evaluate_csg_polyface_tree(shared, {a, b}).status != "evaluated",
          "persisted node graph is a tree");
    CsgMeshTreeOptions options;
    options.max_nodes = 1;
    check(evaluate_csg_polyface_tree(tree, {a, b}, options).status == "work_limit",
          "tree node budget");
    options = {};
    options.boolean.max_boolean_operations = 1;
    check(evaluate_csg_polyface_tree(tree, {a, b}, options).meshes.empty(),
          "whole-tree Boolean budget");
    options = {};
    options.max_cached_meshes = 1;
    check(evaluate_csg_polyface_tree(tree, {a, b}, options).status == "work_limit",
          "whole-tree cache count budget");
    options = {};
    options.max_vertex_transforms = 0;
    check(evaluate_csg_polyface_tree(tree, {a, b}, options).status == "work_limit",
          "transformed vertex budget");
    options = {};
    options.max_cached_triangles = 0;
    check(evaluate_csg_polyface_tree(tree, {a, b}, options).status == "work_limit",
          "cache triangle budget");
    validate(evaluate_csg_polyface_tree(archive(Json::array(), 0), {}), {}, {});
    return checks;
}
