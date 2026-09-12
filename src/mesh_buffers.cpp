#include "geometry.hpp"
#include "mesh_registry.hpp"

namespace p3d {
namespace {
template <std::size_t N> std::array<float, N> floats(const std::array<double, N> &value) {
    std::array<float, N> out;
    for (std::size_t k = 0; k < N; ++k) {
        out[k] = float(value[k]);
        require(std::isfinite(out[k]), "native mesh float export overflow");
    }
    return out;
}
std::int32_t index(std::size_t id) {
    require(id && id - 1 <= std::size_t(INT32_MAX), "native mesh signed index overflow");
    return std::int32_t(id - 1);
}
} // namespace

Json assemble_native_mesh_buffers(const Json &triangles, bool smoothing) {
    using mesh_detail::face_normal;
    using mesh_detail::Registry;
    using mesh_detail::unit;
    Json out = {{"coordinate_space", "command_local"},
                {"component_type", "float32"},
                {"index_type", "int32"},
                {"normal_mode", smoothing ? "smoothing_groups" : "flat_triangles"},
                {"points", Json::array()},
                {"normals", Json::array()},
                {"uvs", Json::array()},
                {"faces", Json::array()},
                {"face_material_ids", Json::array()}};
    if (!smoothing) {
        Registry<float, 8> vertices(1e-6f);
        for (const auto &triangle : triangles) {
            const auto &corners = triangle.at("corners");
            std::array<Point3, 3> points;
            for (unsigned j = 0; j < 3; ++j)
                points[j] = corners.at(j).at("point").get<Point3>();
            // Flat normals use original double corners, before any registration.
            const auto n = floats(face_normal(points[0], points[1], points[2]));
            std::array<std::int32_t, 3> face;
            for (unsigned j = 0; j < 3; ++j) {
                const auto p = floats(points[j]);
                const auto uv = floats(corners[j].at("uv").get<Point2>());
                face[j] =
                    index(vertices.insert({p[0], p[1], p[2], n[0], n[1], n[2], uv[0], uv[1]}));
            }
            out["faces"].push_back(face);
            out["face_material_ids"].push_back(triangle.at("material_id"));
        }
        for (std::size_t i = 1; i <= vertices.size(); ++i) {
            const auto v = vertices.at(i);
            out["points"].push_back({v[0], v[1], v[2]});
            out["normals"].push_back({v[3], v[4], v[5]});
            out["uvs"].push_back({v[6], v[7]});
        }
        out["vertex_key"] = "float32_position_normal_uv";
        out["vertex_tolerance"] = 1e-6f;
        return out;
    }

    struct TriangleKeys {
        std::array<std::size_t, 3> position, uv;
        std::int32_t group;
        Point3 normal;
    };
    Registry<double, 3> positions(1e-7), normals(1e-7);
    Registry<double, 2> uvs(1e-6);
    std::vector<TriangleKeys> keys;
    std::map<std::pair<std::int32_t, std::size_t>, Point3> sums;
    for (const auto &triangle : triangles) {
        TriangleKeys key;
        key.group = triangle.at("normal_group").get<std::int32_t>();
        std::array<Point3, 3> p;
        for (unsigned j = 0; j < 3; ++j) {
            const auto &corner = triangle.at("corners").at(j);
            key.position[j] = positions.insert(corner.at("point").get<Point3>());
            key.uv[j] = uvs.insert(corner.at("uv").get<Point2>());
            p[j] = positions.at(key.position[j]);
        }
        key.normal = face_normal(p[0], p[1], p[2]);
        if (key.group)
            for (const auto id : key.position)
                for (unsigned k = 0; k < 3; ++k)
                    sums[{key.group, id}][k] += key.normal[k];
        keys.push_back(key);
    }
    for (auto &sum : sums)
        sum.second = unit(sum.second);
    // Exact integer triples are a strict order; output stays in insertion order.
    using Triple = std::array<std::int32_t, 3>;
    std::map<Triple, std::int32_t> registry;
    std::vector<Triple> vertices;
    for (std::size_t i = 0; i < keys.size(); ++i) {
        const auto &key = keys[i];
        Triple face;
        for (unsigned j = 0; j < 3; ++j) {
            const auto n = key.group ? sums.at({key.group, key.position[j]}) : key.normal;
            const Triple vertex{index(key.position[j]), index(normals.insert(n)), index(key.uv[j])};
            const auto inserted = registry.emplace(vertex, index(vertices.size() + 1));
            if (inserted.second)
                vertices.push_back(vertex);
            face[j] = inserted.first->second;
        }
        out["faces"].push_back(face);
        out["face_material_ids"].push_back(triangles[i].at("material_id"));
    }
    for (const auto &vertex : vertices) {
        // Convert only after the registry keys are fixed. Equal exported floats
        // do not imply equal native position/normal/UV registry indices.
        out["points"].push_back(floats(positions.at(std::size_t(vertex[0]) + 1)));
        out["normals"].push_back(floats(normals.at(std::size_t(vertex[1]) + 1)));
        out["uvs"].push_back(floats(uvs.at(std::size_t(vertex[2]) + 1)));
    }
    out["vertex_key"] = "position_normal_uv_registry_indices";
    out["registry_counts"] = {
        {"positions", positions.size()}, {"normals", normals.size()}, {"uvs", uvs.size()}};
    out["registry_tolerances"] = {{"positions", 1e-7}, {"normals", 1e-7}, {"uvs", 1e-6}};
    return out;
}
} // namespace p3d
