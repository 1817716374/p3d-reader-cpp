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
        const auto mesh = value.GetMeshGL64();
        require(mesh.numProp == 3 && mesh.faceID.size() == mesh.NumTri(),
                "CSG incomplete output provenance");
        CsgMeshBooleanResult output;
        output.status = "evaluated";
        output.diagnostics = result.diagnostics;
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
            require(id == first_id || id == first_id + 1, "CSG output source identity");
            CsgTriangleSource source;
            source.operand = id - first_id;
            source.face_index = mesh.faceID[i];
            source.backside = mesh.Backside(run);
            const auto &g = source.operand ? right : left;
            require(source.face_index < g.faces.size(), "CSG output source face range");
            Triangle triangle{};
            for (unsigned k = 0; k < 3; ++k) {
                const auto index = mesh.triVerts[i * 3 + k];
                require(index < output.vertices.size() && index <= UINT32_MAX,
                        "CSG output vertex range");
                triangle[k] = std::uint32_t(index);
                source.corner_barycentric[k] =
                    barycentric(g, source.face_index, output.vertices[index],
                                source.corner_projection_distance[k]);
            }
            output.faces.push_back(triangle);
            output.face_sources.push_back(source);
        }
        return output;
    } catch (const std::exception &e) {
        result.diagnostics["reason"] = e.what();
        return result;
    }
}
} // namespace p3d
