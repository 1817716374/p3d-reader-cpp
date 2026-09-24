#include <p3d/csg_mesh.hpp>
#include "internal.hpp"
#include <manifold/manifold.h>

namespace p3d {
namespace {
const char *error_name(manifold::Manifold::Error error) {
    static const char *names[] = {"NoError",
                                  "NonFiniteVertex",
                                  "NotManifold",
                                  "VertexOutOfBounds",
                                  "PropertiesWrongLength",
                                  "MissingPositionProperties",
                                  "MergeVectorsDifferentLengths",
                                  "MergeIndexOutOfBounds",
                                  "TransformWrongLength",
                                  "RunIndexWrongLength",
                                  "FaceIDWrongLength",
                                  "InvalidConstruction",
                                  "ResultTooLarge",
                                  "InvalidTangents",
                                  "Cancelled"};
    const auto index = unsigned(error);
    return index < sizeof(names) / sizeof(*names) ? names[index] : "UnknownKernelError";
}
manifold::Manifold solid(const Geometry &g, std::uint32_t id, double tolerance) {
    require(g.lines.empty() && g.texts.empty() && g.unknown.empty(),
            "CSG mesh operand contains nonmesh or unresolved geometry");
    require(g.vertices.size() <= std::size_t(INT32_MAX) &&
                g.faces.size() <= std::size_t(INT32_MAX / 3),
            "CSG mesh operand too large");
    manifold::MeshGL64 mesh;
    mesh.tolerance = tolerance;
    for (const auto &p : g.vertices)
        for (double x : p) {
            require(std::isfinite(x), "CSG nonfinite input position");
            mesh.vertProperties.push_back(x);
        }
    for (std::size_t i = 0; i < g.faces.size(); ++i) {
        for (auto index : g.faces[i]) {
            require(index < g.vertices.size(), "CSG input triangle index out of range");
            mesh.triVerts.push_back(index);
        }
        mesh.faceID.push_back(i);
    }
    if (g.faces.empty())
        return {};
    mesh.runOriginalID = {id};
    mesh.runIndex = {0, mesh.triVerts.size()};
    // Do not invoke Merge(): matching coordinates are not proof that the
    // original indexed topology identifies these vertices.
    return manifold::Manifold(mesh);
}
std::array<double, 3> barycentric(const Geometry &g, std::size_t face, Point3 point,
                                  double &distance) {
    const auto &t = g.faces.at(face);
    const auto &a = g.vertices.at(t[0]);
    Point3 u{}, v{}, w{};
    double scale = 0;
    for (unsigned k = 0; k < 3; ++k) {
        u[k] = g.vertices.at(t[1])[k] - a[k];
        v[k] = g.vertices.at(t[2])[k] - a[k];
        w[k] = point[k] - a[k];
        require(std::isfinite(u[k]) && std::isfinite(v[k]) && std::isfinite(w[k]),
                "CSG source correspondence arithmetic overflow");
        scale = std::max(scale, std::max(std::abs(u[k]), std::abs(v[k])));
    }
    require(scale > 0, "CSG degenerate source triangle");
    for (unsigned k = 0; k < 3; ++k) {
        u[k] /= scale;
        v[k] /= scale;
        w[k] /= scale;
    }
    const Point3 normal{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0]};
    unsigned axis = 0;
    for (unsigned k = 1; k < 3; ++k)
        if (std::abs(normal[k]) > std::abs(normal[axis]))
            axis = k;
    const auto x = (axis + 1) % 3, y = (axis + 2) % 3;
    const double det = u[x] * v[y] - u[y] * v[x];
    require(det != 0 && std::isfinite(det), "CSG degenerate source triangle");
    const double b = (w[x] * v[y] - w[y] * v[x]) / det;
    const double c = (u[x] * w[y] - u[y] * w[x]) / det;
    const std::array<double, 3> weights{1 - b - c, b, c};
    for (auto q : weights)
        require(std::isfinite(q), "CSG nonfinite source weights");
    Point3 delta{};
    for (unsigned k = 0; k < 3; ++k)
        delta[k] = a[k] + (b * u[k] + c * v[k]) * scale - point[k];
    distance = std::hypot(delta[0], delta[1], delta[2]);
    require(std::isfinite(distance), "CSG nonfinite source projection distance");
    return weights;
}
struct MeshSource {
    const Geometry *geometry;
    unsigned operand;
    std::size_t mesh_index;
};
CsgMeshBooleanResult extract_mesh(const manifold::Manifold &value, std::uint32_t first_id,
                                  const std::vector<MeshSource> &sources, const Json &diagnostics) {
    const auto mesh = value.GetMeshGL64();
    require(mesh.numProp == 3 && mesh.faceID.size() == mesh.NumTri(),
            "CSG incomplete output provenance");
    CsgMeshBooleanResult output;
    output.status = "evaluated";
    output.diagnostics = diagnostics;
    output.diagnostics["effective_tolerance"] = mesh.tolerance;
    for (std::size_t i = 0; i < mesh.NumVert(); ++i)
        output.vertices.push_back({mesh.vertProperties[i * 3], mesh.vertProperties[i * 3 + 1],
                                   mesh.vertProperties[i * 3 + 2]});
    std::size_t run = 0;
    for (std::size_t i = 0; i < mesh.NumTri(); ++i) {
        while (run + 1 < mesh.runIndex.size() && i * 3 >= mesh.runIndex[run + 1])
            ++run;
        require(run < mesh.runOriginalID.size() && mesh.runIndex.at(run) <= i * 3 &&
                    mesh.runIndex.at(run + 1) >= i * 3 + 3,
                "CSG output triangle run range");
        const auto id = mesh.runOriginalID[run];
        require(id >= first_id && std::size_t(id - first_id) < sources.size(),
                "CSG output source identity");
        const auto &ref = sources.at(id - first_id);
        CsgTriangleSource source;
        source.operand = ref.operand;
        source.mesh_index = ref.mesh_index;
        source.face_index = mesh.faceID[i];
        source.backside = mesh.Backside(run);
        const auto &g = *ref.geometry;
        require(source.face_index < g.faces.size(), "CSG output source face range");
        Triangle triangle{};
        for (unsigned k = 0; k < 3; ++k) {
            const auto index = mesh.triVerts[i * 3 + k];
            require(index < output.vertices.size() && index <= UINT32_MAX,
                    "CSG output vertex range");
            triangle[k] = std::uint32_t(index);
            source.corner_barycentric[k] = barycentric(g, source.face_index, output.vertices[index],
                                                       source.corner_projection_distance[k]);
        }
        output.faces.push_back(triangle);
        output.face_sources.push_back(source);
    }
    return output;
}
} // namespace
CsgMeshBooleanResult evaluate_csg_mesh_boolean(const Geometry &left, const Geometry &right,
                                               std::int32_t operation,
                                               const CsgMeshBooleanOptions &options) {
    CsgMeshBooleanResult result;
    result.diagnostics = {{"kernel", "manifold"},
                          {"scope", "derived_triangle_mesh_boolean"},
                          {"native_tree_update", "not_evaluated"},
                          {"native_kernel_equivalence", "not_established"}};
    if (operation < 0 || operation > 2 || !std::isfinite(options.tolerance) ||
        options.tolerance < 0) {
        result.status = "invalid_input";
        result.diagnostics["reason"] = "invalid_operation_or_tolerance";
        return result;
    }
    try {
        result.status = "invalid_input";
        const auto first_id = manifold::Manifold::ReserveIDs(2);
        const auto a = solid(left, first_id, options.tolerance);
        const auto b = solid(right, first_id + 1, options.tolerance);
        if (a.Status() != manifold::Manifold::Error::NoError ||
            b.Status() != manifold::Manifold::Error::NoError) {
            result.status = "invalid_input";
            result.diagnostics["operand_error_codes"] = {int(a.Status()), int(b.Status())};
            result.diagnostics["operand_errors"] = {error_name(a.Status()), error_name(b.Status())};
            return result;
        }
        result.status = "not_evaluated";
        const auto op = operation == 0   ? manifold::OpType::Add
                        : operation == 1 ? manifold::OpType::Intersect
                                         : manifold::OpType::Subtract;
        const auto value = a.Boolean(b, op);
        if (value.Status() != manifold::Manifold::Error::NoError) {
            result.diagnostics["kernel_error_code"] = int(value.Status());
            result.diagnostics["kernel_error"] = error_name(value.Status());
            return result;
        }
        if (value.NumTri() > options.max_output_triangles || value.NumVert() > UINT32_MAX) {
            result.status = "output_limit";
            return result;
        }
        return extract_mesh(value, first_id, {{&left, 0, 0}, {&right, 1, 0}}, result.diagnostics);
    } catch (const std::exception &e) {
        result.diagnostics["reason"] = e.what();
        return result;
    }
}
namespace {
CsgMeshListResult evaluate_lists(const std::vector<Geometry> &left,
                                 const std::vector<Geometry> &right, std::int32_t operation,
                                 bool leaf, const CsgMeshListOptions &options) {
    CsgMeshListResult result;
    result.diagnostics = {{"kernel", "manifold"},
                          {"scope", "derived_triangle_mesh_list_boolean"},
                          {"native_tree_update", "not_evaluated"},
                          {"native_kernel_equivalence", "not_established"},
                          {"leaf_reduction", leaf},
                          {"boolean_operations", 0},
                          {"empty_groups_retained", true}};
    if (operation < 0 || operation > (leaf ? 1 : 2) || !std::isfinite(options.mesh.tolerance) ||
        options.mesh.tolerance < 0) {
        result.status = "invalid_input";
        result.diagnostics["reason"] = "invalid_operation_or_tolerance";
        return result;
    }
    try {
        result.status = "invalid_input";
        require(left.size() <= UINT32_MAX && right.size() <= UINT32_MAX - left.size(),
                "CSG source identity count exceeds kernel range");
        const auto count = left.size() + right.size();
        std::size_t operations = 0, groups = 0;
        if (leaf) {
            groups = left.empty() ? 0 : 1;
            operations = left.empty() ? 0 : left.size() - 1;
        } else if (operation == 0) {
            groups = left.empty() || right.empty() ? count : 1;
            operations = left.empty() || right.empty() ? 0 : count - 1;
        } else {
            result.status = "work_limit";
            require(right.empty() || left.size() <= options.max_boolean_operations / right.size(),
                    "CSG Boolean operation budget exceeded");
            operations = left.size() * right.size();
            groups = operation == 1 ? operations : left.size();
        }
        result.status = "work_limit";
        require(operations <= options.max_boolean_operations,
                "CSG Boolean operation budget exceeded");
        require(groups <= options.max_output_groups, "CSG output group budget exceeded");
        result.diagnostics["planned_boolean_operations"] = operations;
        result.diagnostics["planned_groups"] = groups;
        result.status = "invalid_input";
        std::vector<MeshSource> sources;
        std::vector<manifold::Manifold> solids;
        const auto first_id = count ? manifold::Manifold::ReserveIDs(std::uint32_t(count)) : 0;
        auto add = [&](const std::vector<Geometry> &input, unsigned side) {
            for (std::size_t i = 0; i < input.size(); ++i) {
                result.diagnostics["input_operand"] = side;
                result.diagnostics["input_mesh_index"] = i;
                const auto &g = input[i];
                auto value =
                    solid(g, first_id + std::uint32_t(sources.size()), options.mesh.tolerance);
                require(value.Status() == manifold::Manifold::Error::NoError,
                        error_name(value.Status()));
                sources.push_back({&g, side, i});
                solids.push_back(std::move(value));
            }
        };
        add(left, 0);
        add(right, 1);
        result.diagnostics.erase("input_operand");
        result.diagnostics.erase("input_mesh_index");
        result.status = "not_evaluated";
        std::size_t performed = 0, triangles = 0;
        auto combine = [&](const manifold::Manifold &a, const manifold::Manifold &b, int op) {
            const auto type = op == 0   ? manifold::OpType::Add
                              : op == 1 ? manifold::OpType::Intersect
                                        : manifold::OpType::Subtract;
            auto value = a.Boolean(b, type);
            ++performed;
            result.diagnostics["boolean_operations"] = performed;
            require(value.Status() == manifold::Manifold::Error::NoError,
                    error_name(value.Status()));
            // Force each step before the next: do not replace native grouping
            // with a backend's reordered multi-input batch Boolean.
            if (value.NumTri() > options.mesh.max_output_triangles) {
                result.status = "output_limit";
                require(false, "CSG intermediate triangle budget exceeded");
            }
            return value;
        };
        auto indices = [](std::size_t n) {
            std::vector<std::size_t> out(n);
            for (std::size_t i = 0; i < n; ++i)
                out[i] = i;
            return out;
        };
        auto emit = [&](CsgMeshGroup group) {
            if (group.mesh.faces.size() > options.mesh.max_output_triangles - triangles) {
                result.status = "output_limit";
                require(false, "CSG total output triangle budget exceeded");
            }
            triangles += group.mesh.faces.size();
            result.groups.push_back(std::move(group));
        };
        auto output = [&](const manifold::Manifold &value, std::vector<std::size_t> li,
                          std::vector<std::size_t> ri) {
            if (value.NumTri() > options.mesh.max_output_triangles - triangles ||
                value.NumVert() > UINT32_MAX) {
                result.status = "output_limit";
                require(false, "CSG output size budget exceeded");
            }
            emit({std::move(li), std::move(ri),
                  extract_mesh(value, first_id, sources, result.diagnostics)});
        };
        auto passthrough = [&](std::size_t index) {
            const auto &ref = sources.at(index);
            const auto &g = *ref.geometry;
            if (g.faces.size() > options.mesh.max_output_triangles - triangles) {
                result.status = "output_limit";
                require(false, "CSG total output triangle budget exceeded");
            }
            CsgMeshBooleanResult mesh;
            mesh.status = "evaluated";
            mesh.diagnostics = result.diagnostics;
            mesh.diagnostics["passthrough"] = true;
            mesh.vertices = g.vertices;
            mesh.faces = g.faces;
            for (std::size_t face = 0; face < g.faces.size(); ++face) {
                CsgTriangleSource source;
                source.operand = ref.operand;
                source.mesh_index = ref.mesh_index;
                source.face_index = face;
                for (unsigned k = 0; k < 3; ++k)
                    source.corner_barycentric[k][k] = 1;
                mesh.face_sources.push_back(source);
            }
            emit({ref.operand == 0 ? std::vector<std::size_t>{ref.mesh_index}
                                   : std::vector<std::size_t>{},
                  ref.operand == 1 ? std::vector<std::size_t>{ref.mesh_index}
                                   : std::vector<std::size_t>{},
                  std::move(mesh)});
        };
        auto fold = [&](std::size_t begin, std::size_t end, int op) {
            auto value = solids.at(begin);
            for (auto i = begin + 1; i < end; ++i)
                value = combine(value, solids[i], op);
            return value;
        };
        if (leaf) {
            if (left.size() == 1)
                passthrough(0);
            else if (!left.empty())
                output(fold(0, left.size(), operation), indices(left.size()), {});
        } else if (operation == 0) {
            if (left.empty() || right.empty()) {
                for (std::size_t i = 0; i < count; ++i)
                    passthrough(i);
            } else {
                const auto a = fold(0, left.size(), 0);
                const auto b = fold(left.size(), count, 0);
                output(combine(a, b, 0), indices(left.size()), indices(right.size()));
            }
        } else if (operation == 1) {
            for (std::size_t i = 0; i < left.size(); ++i)
                for (std::size_t j = 0; j < right.size(); ++j)
                    output(combine(solids[i], solids[left.size() + j], 1), {i}, {j});
        } else {
            for (std::size_t i = 0; i < left.size(); ++i) {
                if (right.empty()) {
                    passthrough(i);
                    continue;
                }
                auto value = solids[i];
                for (std::size_t j = 0; j < right.size(); ++j)
                    value = combine(value, solids[left.size() + j], 2);
                output(value, {i}, indices(right.size()));
            }
        }
        result.status = "evaluated";
        result.diagnostics["output_triangles"] = triangles;
        return result;
    } catch (const std::exception &e) {
        result.groups.clear();
        result.diagnostics["reason"] = e.what();
        return result;
    }
}
} // namespace
CsgMeshListResult evaluate_csg_mesh_lists(const std::vector<Geometry> &left,
                                          const std::vector<Geometry> &right,
                                          std::int32_t operation,
                                          const CsgMeshListOptions &options) {
    return evaluate_lists(left, right, operation, false, options);
}
CsgMeshListResult reduce_csg_mesh_list(const std::vector<Geometry> &input, std::int32_t operation,
                                       const CsgMeshListOptions &options) {
    return evaluate_lists(input, {}, operation, true, options);
}
} // namespace p3d
