#include "geometry.hpp"

namespace p3d {
namespace {
// The native tolerance comparison is not a strict weak ordering. An STL map
// would have undefined requirements here; reproduce its explicit tree walk and
// red/black insertion instead. This registry never changes the source mesh.
class Positions {
    struct Node {
        Point3 value;
        std::size_t child[2]{0, 0}, parent = 0;
        bool red = true;
    };
    std::vector<Node> nodes{{}};
    std::size_t root = 0;
    static int compare(Point3 a, Point3 b) {
        for (unsigned k = 0; k < 3; ++k) {
            if (a[k] - 1e-7 > b[k])
                return 1;
            if (b[k] > a[k] + 1e-7)
                return -1;
        }
        return 0;
    }
    void rotate(std::size_t x, unsigned side) {
        const auto y = nodes[x].child[1 - side];
        nodes[x].child[1 - side] = nodes[y].child[side];
        if (nodes[y].child[side])
            nodes[nodes[y].child[side]].parent = x;
        const auto p = nodes[x].parent;
        nodes[y].parent = p;
        if (!p)
            root = y;
        else
            nodes[p].child[nodes[p].child[1] == x] = y;
        nodes[y].child[side] = x;
        nodes[x].parent = y;
    }

  public:
    Positions() {
        nodes[0].red = false;
    }
    std::size_t insert(Point3 value) {
        std::size_t p = 0, candidate = 0, x = root;
        unsigned side = 0;
        while (x) {
            p = x;
            side = compare(value, nodes[x].value) > 0;
            if (!side)
                candidate = x;
            x = nodes[x].child[side];
        }
        if (candidate && compare(nodes[candidate].value, value) <= 0)
            return candidate;
        const auto id = nodes.size();
        nodes.push_back(Node{value, {0, 0}, p, true});
        if (!p)
            root = id;
        else
            nodes[p].child[side] = id;
        x = id;
        while (nodes[nodes[x].parent].red) {
            p = nodes[x].parent;
            auto g = nodes[p].parent;
            side = nodes[g].child[1] == p;
            const auto uncle = nodes[g].child[1 - side];
            if (nodes[uncle].red) {
                nodes[p].red = nodes[uncle].red = false;
                nodes[g].red = true;
                x = g;
            } else {
                if (x == nodes[p].child[1 - side]) {
                    x = p;
                    rotate(x, side);
                    p = nodes[x].parent;
                    g = nodes[p].parent;
                }
                nodes[p].red = false;
                nodes[g].red = true;
                rotate(g, 1 - side);
            }
        }
        nodes[root].red = false;
        return id;
    }
    Point3 at(std::size_t i) const {
        return nodes.at(i).value;
    }
    std::size_t size() const {
        return nodes.size() - 1;
    }
};
Point3 unit(Point3 p) {
    const double length = std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]);
    require(std::isfinite(length), "mesh normal magnitude overflow");
    // GeVec3d::normalize assigns +X when the magnitude is zero.
    if (!(length > 0))
        return {1, 0, 0};
    const double factor = 1 / length;
    for (auto &v : p) {
        v *= factor;
        require(std::isfinite(v), "mesh normal overflow");
    }
    return p;
}
Point3 face_normal(Point3 a, Point3 b, Point3 c) {
    for (unsigned k = 0; k < 3; ++k) {
        b[k] -= a[k];
        c[k] -= a[k];
    }
    return unit({b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]});
}
} // namespace

void evaluate_mesh_normals(Json &channels, const std::vector<Point3> &points,
                           const std::vector<Triangle> &faces) {
    const auto &source = channels.at("source");
    if (!source.contains("native_triangulation") || !source["native_triangulation"].is_object())
        return;
    const auto &routing = source["native_triangulation"];
    if (routing.value("status", "") != "mapped" ||
        !routing.value("source_polygon_layout_matches", false))
        return;
    Json report = {{"status", "invalid"},
                   {"triangle_input", "library_triangulation"},
                   {"native_glu_triangulation", "not_evaluated"},
                   {"zero_magnitude_normal", Point3{1, 0, 0}},
                   {"placement", "inverse_transpose_without_renormalization"}};
    try {
        require(channels["triangles"].size() == faces.size(), "mesh normal triangle mapping");
        const bool smoothing = routing["normal_mode"] == "smoothing_groups";
        Positions positions;
        std::vector<std::array<std::size_t, 3>> keys;
        std::vector<Point3> normals;
        std::vector<std::int32_t> groups;
        std::map<std::pair<std::int32_t, std::size_t>, Point3> sums;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            std::array<Point3, 3> p;
            std::array<std::size_t, 3> ids{};
            for (unsigned j = 0; j < 3; ++j) {
                p[j] = points.at(faces[i][j]);
                for (double v : p[j])
                    require(std::isfinite(v), "nonfinite mesh normal position");
                if (smoothing) {
                    ids[j] = positions.insert(p[j]);
                    p[j] = positions.at(ids[j]);
                }
            }
            const auto &g =
                channels["triangles"].at(i).at("native_triangulation").at("normal_group");
            const auto group = smoothing && !g.is_null() ? g.get<std::int32_t>() : 0;
            keys.push_back(ids);
            groups.push_back(group);
            normals.push_back(face_normal(p[0], p[1], p[2]));
            if (group)
                for (auto id : ids)
                    for (unsigned k = 0; k < 3; ++k)
                        sums[{group, id}][k] += normals.back()[k];
        }
        for (auto &sum : sums)
            sum.second = unit(sum.second);
        Json values = Json::array();
        for (std::size_t i = 0; i < faces.size(); ++i) {
            std::array<Point3, 3> n;
            for (unsigned j = 0; j < 3; ++j)
                n[j] = groups[i] ? sums.at({groups[i], keys[i][j]}) : normals[i];
            values.push_back(n);
        }
        for (std::size_t i = 0; i < faces.size(); ++i) {
            auto &triangle = channels["triangles"][i]["native_triangulation"];
            triangle["source_corner_normals"] = values[i];
            triangle["corner_normals"] = values[i];
        }
        report["status"] = "computed";
        report["position_registry_count"] = smoothing ? Json(positions.size()) : Json();
    } catch (const std::exception &e) {
        report["error"] = e.what();
    }
    channels["normal_evaluation"] = std::move(report);
}
} // namespace p3d
