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
