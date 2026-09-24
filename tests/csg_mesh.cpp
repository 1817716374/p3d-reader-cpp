#include <p3d/csg_mesh.hpp>
#include "internal.hpp"
#include <future>

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
double volume(const CsgMeshBooleanResult &g) {
    double v = 0;
    for (const auto &t : g.faces) {
        const auto &a = g.vertices[t[0]], &b = g.vertices[t[1]], &c = g.vertices[t[2]];
        v += a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) +
             a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    return v / 6;
}
} // namespace
unsigned csg_mesh_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    const auto a = cube(0), b = cube(1);
    auto validate = [&](const CsgMeshBooleanResult &r, const Geometry &left, const Geometry &right,
                        double expected) {
        check(r.status == "evaluated", r.diagnostics.dump().c_str());
        check(std::abs(volume(r) - expected) < 1e-9, "CSG independently computed signed volume");
        check(r.face_sources.size() == r.faces.size(), "every Boolean output face has a source");
        for (std::size_t i = 0; i < r.faces.size(); ++i) {
            const auto &source = r.face_sources[i];
            const auto &g = source.operand ? right : left;
            check(source.operand < 2 && source.face_index < g.faces.size(),
                  "Boolean face identity references supplied source mesh");
            for (unsigned k = 0; k < 3; ++k) {
                const auto &w = source.corner_barycentric[k];
                check(std::abs(w[0] + w[1] + w[2] - 1) < 1e-12,
                      "Boolean source weights sum to one");
                check(source.corner_projection_distance[k] < 1e-9,
                      "Boolean source projection reports bounded geometric displacement");
                for (unsigned axis = 0; axis < 3; ++axis) {
                    double p = 0;
                    for (unsigned j = 0; j < 3; ++j)
                        p += w[j] * g.vertices[g.faces[source.face_index][j]][axis];
                    check(std::abs(p - r.vertices[r.faces[i][k]][axis]) < 1e-9,
                          "Boolean source weights recover the output point and linear attributes");
                }
            }
        }
    };
    for (int op = 0; op < 3; ++op)
        validate(evaluate_csg_mesh_boolean(a, b, op), a, b, op == 0 ? 12 : 4);
    const auto asymmetric = cube(.5);
    validate(evaluate_csg_mesh_boolean(a, asymmetric, 1), a, asymmetric, 6);
    validate(evaluate_csg_mesh_boolean(a, asymmetric, 2), a, asymmetric, 2);
    auto rotate = [](Geometry g) {
        for (auto &p : g.vertices) {
            const auto y = std::cos(.4) * p[1] - std::sin(.4) * p[2];
            const auto z = std::sin(.4) * p[1] + std::cos(.4) * p[2];
            p = {std::cos(.3) * p[0] - std::sin(.3) * y + 3,
                 std::sin(.3) * p[0] + std::cos(.3) * y - 2, z + 4};
        }
        return g;
    };
    const auto rotated_a = rotate(a), rotated_b = rotate(b);
    validate(evaluate_csg_mesh_boolean(rotated_a, rotated_b, 1), rotated_a, rotated_b, 4);
    auto difference = evaluate_csg_mesh_boolean(a, b, 2);
    bool cut = false;
    for (const auto &s : difference.face_sources)
        if (s.operand == 1) {
            cut = true;
            check(s.backside, "difference cutter faces preserve reversed-source orientation");
        }
    check(cut, "difference carries cutter provenance for new boundary faces");
    const auto disjoint = cube(4);
    validate(evaluate_csg_mesh_boolean(a, disjoint, 0), a, disjoint, 16);
    validate(evaluate_csg_mesh_boolean(a, disjoint, 1), a, disjoint, 0);
    validate(evaluate_csg_mesh_boolean(a, disjoint, 2), a, disjoint, 8);
    const auto touching = cube(2);
    validate(evaluate_csg_mesh_boolean(a, touching, 0), a, touching, 16);
    validate(evaluate_csg_mesh_boolean(a, touching, 1), a, touching, 0);
    validate(evaluate_csg_mesh_boolean(a, a, 2), a, a, 0);
    Geometry empty;
    for (int op = 0; op < 3; ++op) {
        validate(evaluate_csg_mesh_boolean(a, empty, op), a, empty, op == 1 ? 0 : 8);
        validate(evaluate_csg_mesh_boolean(empty, a, op), empty, a, op == 0 ? 8 : 0);
    }
    auto open = a;
    open.faces.pop_back();
    auto bad = evaluate_csg_mesh_boolean(open, b, 0);
    check(bad.status == "invalid_input" && bad.faces.empty(),
          "open mesh is rejected without topology repair");
    Geometry unshared;
    for (const auto &face : a.faces) {
        Triangle triangle{};
        for (unsigned k = 0; k < 3; ++k) {
            triangle[k] = std::uint32_t(unshared.vertices.size());
            unshared.vertices.push_back(a.vertices[face[k]]);
        }
        unshared.faces.push_back(triangle);
    }
    check(evaluate_csg_mesh_boolean(unshared, b, 0).status == "invalid_input",
          "coincident positions do not invent missing source vertex identities");
    auto unresolved = a;
    unresolved.unknown.push_back({{"reason", "unknown source"}});
    check(evaluate_csg_mesh_boolean(unresolved, b, 0).status != "evaluated",
          "unknown input geometry cannot silently disappear");
    auto invalid = a;
    invalid.faces[0][0] = 99;
    check(evaluate_csg_mesh_boolean(invalid, b, 0).status != "evaluated",
          "invalid source index does not reach mesh kernel");
    invalid = a;
    invalid.vertices[0][0] = std::numeric_limits<double>::infinity();
    check(evaluate_csg_mesh_boolean(invalid, b, 0).status != "evaluated",
          "nonfinite geometry rejected");
    CsgMeshBooleanOptions options;
    options.max_output_triangles = 1;
    check(evaluate_csg_mesh_boolean(a, b, 0, options).status == "output_limit",
          "oversized result is not emitted as a partial mesh");
    check(evaluate_csg_mesh_boolean(a, b, 3).status == "invalid_input",
          "Compound is not silently treated as a Boolean union");
    const std::vector<Geometry> list_left{cube(0), cube(.5)};
    const std::vector<Geometry> list_right{cube(1), cube(1.5)};
    auto validate_list = [&](const CsgMeshListResult &r, const std::vector<Geometry> &left,
                             const std::vector<Geometry> &right, const std::vector<double> &volumes,
                             std::size_t operations) {
        check(r.status == "evaluated", r.diagnostics.dump().c_str());
        check(r.groups.size() == volumes.size(), "CSG grouped output cardinality");
        check(r.diagnostics.at("boolean_operations") == operations,
              "CSG grouped operation count follows native order");
        for (std::size_t n = 0; n < r.groups.size(); ++n) {
            const auto &group = r.groups[n];
            const auto &mesh = group.mesh;
            check(mesh.status == "evaluated" && std::abs(volume(mesh) - volumes[n]) < 1e-9,
                  "CSG group independently computed signed volume");
            check(mesh.face_sources.size() == mesh.faces.size(), "CSG group source cardinality");
            for (std::size_t f = 0; f < mesh.faces.size(); ++f) {
                const auto &s = mesh.face_sources[f];
                check(s.operand < 2, "CSG source operand range");
                const auto &inputs = s.operand ? right : left;
                const auto &members = s.operand ? group.right_indices : group.left_indices;
                check(s.mesh_index < inputs.size() &&
                          std::find(members.begin(), members.end(), s.mesh_index) != members.end(),
                      "CSG source identifies original list entry within its output group");
                const auto &g = inputs.at(s.mesh_index);
                check(s.face_index < g.faces.size(), "CSG group source face range");
                for (unsigned k = 0; k < 3; ++k) {
                    check(s.corner_projection_distance[k] < 1e-9,
                          "CSG grouped source displacement bounded");
                    for (unsigned axis = 0; axis < 3; ++axis) {
                        double coordinate = 0;
                        for (unsigned j = 0; j < 3; ++j)
                            coordinate += s.corner_barycentric[k][j] *
                                          g.vertices[g.faces[s.face_index][j]][axis];
                        check(std::abs(coordinate - mesh.vertices[mesh.faces[f][k]][axis]) < 1e-9,
                              "CSG repeated operations retain original corner provenance");
                    }
                }
            }
        }
    };
    auto intersections = evaluate_csg_mesh_lists(list_left, list_right, 1);
    validate_list(intersections, list_left, list_right, {4, 2, 6, 4}, 4);
    for (std::size_t i = 0; i < 4; ++i)
        check(intersections.groups[i].left_indices == std::vector<std::size_t>{i / 2} &&
                  intersections.groups[i].right_indices == std::vector<std::size_t>{i % 2},
              "CSG intersections preserve left-major pair order, including overlap");
    validate_list(evaluate_csg_mesh_lists(list_left, list_right, 0), list_left, list_right, {14},
                  3);
    auto differences = evaluate_csg_mesh_lists(list_left, list_right, 2);
    validate_list(differences, list_left, list_right, {4, 2}, 4);
    for (const auto &group : differences.groups)
        for (const auto &s : group.mesh.face_sources)
            if (s.operand == 1)
                check(s.backside,
                      "list difference preserves cutter orientation through later cuts");
    const std::vector<Geometry> series{cube(0), cube(.5), cube(1)};
    validate_list(reduce_csg_mesh_list(series, 0), series, {}, {12}, 2);
    validate_list(reduce_csg_mesh_list(series, 1), series, {}, {4}, 2);
    validate_list(reduce_csg_mesh_list({}, 1), {}, {}, {}, 0);
    const std::vector<Geometry> duplicates{a, a};
    auto passed = evaluate_csg_mesh_lists({}, duplicates, 0);
    validate_list(passed, {}, duplicates, {8, 8}, 0);
    for (std::size_t i = 0; i < 2; ++i) {
        check(passed.groups[i].mesh.vertices == a.vertices &&
                  passed.groups[i].mesh.faces == a.faces,
              "empty-list union preserves each duplicate mesh's exact topology");
        check(passed.groups[i].mesh.face_sources[0].mesh_index == i,
              "identical list entries retain separate identities");
    }
    validate_list(evaluate_csg_mesh_lists(duplicates, {}, 2), duplicates, {}, {8, 8}, 0);
    validate_list(evaluate_csg_mesh_lists(duplicates, {}, 1), duplicates, {}, {}, 0);
    validate_list(evaluate_csg_mesh_lists({}, duplicates, 2), {}, duplicates, {}, 0);
    validate_list(evaluate_csg_mesh_lists(duplicates, {disjoint}, 1), duplicates, {disjoint},
                  {0, 0}, 2);
    validate_list(reduce_csg_mesh_list({a}, 1), {a}, {}, {8}, 0);
    check(reduce_csg_mesh_list({a}, 2).status == "invalid_input",
          "leaf difference is not invented");
    CsgMeshListOptions list_options;
    list_options.max_boolean_operations = 3;
    auto limited = evaluate_csg_mesh_lists(list_left, list_right, 1, list_options);
    check(limited.status == "work_limit" && limited.groups.empty() &&
              limited.diagnostics.at("boolean_operations") == 0,
          "Cartesian operation budget checked before any work");
    list_options = {};
    list_options.max_output_groups = 1;
    limited = evaluate_csg_mesh_lists(duplicates, {}, 0, list_options);
    check(limited.status == "work_limit" && limited.groups.empty(), "passthrough group budget");
    list_options = {};
    list_options.mesh.max_output_triangles = a.faces.size();
    limited = evaluate_csg_mesh_lists(duplicates, {}, 0, list_options);
    check(limited.status == "output_limit" && limited.groups.empty(),
          "aggregate output triangle limit discards earlier successful groups");
    list_options.mesh.max_output_triangles = 0;
    limited = reduce_csg_mesh_list(series, 0, list_options);
    check(limited.status == "output_limit" && limited.groups.empty(),
          "intermediate triangle limit is enforced");
    auto bad_list = list_left;
    bad_list.back().unknown.push_back({{"reason", "unresolved"}});
    limited = evaluate_csg_mesh_lists(bad_list, list_right, 1);
    check(limited.status == "invalid_input" && limited.groups.empty() &&
              limited.diagnostics.at("input_mesh_index") == 1 &&
              limited.diagnostics.at("boolean_operations") == 0,
          "late invalid list entry fails atomically with original input identity");
    std::vector<std::future<CsgMeshBooleanResult>> jobs;
    for (int i = 0; i < 4; ++i)
        jobs.push_back(
            std::async(std::launch::async, [&] { return evaluate_csg_mesh_boolean(a, b, 2); }));
    for (auto &job : jobs)
        validate(job.get(), a, b, 4);
    check(a.vertices == cube(0).vertices && a.faces == cube(0).faces,
          "Boolean evaluation does not mutate source topology");
    return checks;
}
