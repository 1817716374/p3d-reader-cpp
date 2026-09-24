#include <p3d/polyface.hpp>
#include "geometry.hpp"

namespace p3d {
namespace {
bool present(const Json &table, const char *name) {
    return table.contains(name) && !table.at(name).is_null();
}
const Json &array(const Json &table, const char *name) {
    static const Json empty = Json::array();
    if (!present(table, name))
        return empty;
    const auto &out = table.at(name);
    require(out.is_array(), "BGFB Polyface channel is not an array");
    return out;
}
template <std::size_t N>
std::vector<std::array<double, N>> pool(const Json &table, const char *name, std::size_t limit,
                                        Json &report) {
    const auto &values = array(table, name);
    require(values.size() / N <= limit, "BGFB Polyface channel element budget");
    std::vector<std::array<double, N>> out;
    for (std::size_t i = 0; i < values.size() / N; ++i) {
        std::array<double, N> p{};
        for (std::size_t k = 0; k < N; ++k) {
            p[k] = values[i * N + k].get<double>();
            require(std::isfinite(p[k]), "BGFB Polyface nonfinite copied channel value");
        }
        out.push_back(p);
    }
    report[name] = {{"source_scalar_count", values.size()},
                    {"copied_elements", out.size()},
                    {"ignored_tail_scalars", values.size() % N},
                    {"present", present(table, name)}};
    return out;
}
std::uint32_t index(const Json &value, std::size_t count) {
    const auto signed_index = value.get<std::int64_t>();
    require(signed_index >= INT32_MIN && signed_index <= INT32_MAX && signed_index != 0,
            "BGFB Polyface signed index range");
    const auto absolute = std::uint64_t(signed_index < 0 ? -signed_index : signed_index);
    require(absolute <= count, "BGFB Polyface index outside source pool");
    return std::uint32_t(absolute - 1);
}
} // namespace
PolyfaceMeshResult mesh_bgfb_polyface(const Json &table, const PolyfaceMeshOptions &options) {
    PolyfaceMeshResult out;
    out.source = table;
    out.report = {{"scope", "derived_bgfb_polyface_mesh"},
                  {"native_triangulation_equivalence", "not_established"},
                  {"polygon_edge_tests", 0},
                  {"attribute_bindings", Json::object()},
                  {"channels", Json::object()},
                  {"polygons", Json::array()}};
    try {
        require(table.at("_type") == "Polyface", "expected BGFB Polyface table");
        const auto style = table.value("meshStyle", 0);
        const auto width = table.value("numPerFace", 0);
        require(style == 1 || style == 3 || style == 4,
                "BGFB Polyface mesh style not supported by derived triangulation");
        require(width >= 0, "BGFB Polyface negative face block width");
        out.report["mesh_style"] = style;
        out.report["num_per_face"] = width;
        auto &g = out.geometry;
        g.vertices = pool<3>(table, "point", options.max_points, out.report["channels"]);
        require(g.vertices.size() <= UINT32_MAX, "BGFB Polyface point index capacity");
        g.source_normals = pool<3>(table, "normal", options.max_points, out.report["channels"]);
        for (const auto &v : g.source_normals)
            g.normals.push_back(v);
        g.uvs = pool<2>(table, "param", options.max_points, out.report["channels"]);
        const auto colors =
            pool<3>(table, "doubleColor", options.max_points, out.report["channels"]);
        const auto &integer_colors = array(table, "intColor");
        const auto &color_table = array(table, "colorTable");
        require(integer_colors.size() <= options.max_points &&
                    color_table.size() <= options.max_points,
                "BGFB Polyface color pool budget");
        const auto &point_indices = array(table, "pointIndex");
        require(point_indices.size() <= options.max_corners, "BGFB Polyface index budget");
        std::vector<std::vector<std::size_t>> polygons;
        if (style == 1) {
            const auto block = width > 1 ? std::size_t(width) : 0;
            if (block) {
                require(point_indices.size() % block == 0,
                        "BGFB Polyface incomplete fixed face block");
                for (std::size_t begin = 0; begin < point_indices.size(); begin += block) {
                    std::vector<std::size_t> face;
                    for (auto i = begin; i < begin + block && point_indices[i] != 0; ++i)
                        face.push_back(i);
                    for (auto i = begin + face.size(); i < begin + block; ++i)
                        require(point_indices[i] == 0,
                                "BGFB Polyface nonzero corner after fixed-block padding");
                    polygons.push_back(std::move(face));
                }
            } else {
                std::vector<std::size_t> face;
                for (std::size_t i = 0; i < point_indices.size(); ++i) {
                    if (point_indices[i] == 0) {
                        polygons.push_back(std::move(face));
                        face.clear();
                    } else
                        face.push_back(i);
                }
                require(face.empty(), "BGFB Polyface unterminated indexed face");
            }
        } else {
            const std::size_t block = style == 3 ? 3 : 4;
            require(g.vertices.size() <= options.max_corners,
                    "BGFB Polyface implicit corner budget");
            // Native conversion uses complete consecutive point blocks.
            out.report["unused_trailing_points"] = g.vertices.size() % block;
            for (std::size_t begin = 0; begin + block <= g.vertices.size(); begin += block) {
                std::vector<std::size_t> face;
                for (std::size_t i = 0; i < block; ++i)
                    face.push_back(begin + i);
                polygons.push_back(std::move(face));
            }
        }
        out.report["source_corner_count"] = style == 1 ? point_indices.size() : g.vertices.size();
        auto binding = [&](const char *name, const char *report_name, std::size_t count,
                           bool active,
                           const std::array<std::size_t, 3> &corners) -> std::optional<Triangle> {
            auto &report = out.report["attribute_bindings"][report_name];
            if (report.is_null())
                report = {{"status", "mapped"}, {"invalid_triangles", 0}};
            const auto &indices = array(table, name);
            if (!active || (style == 1 && indices.empty())) {
                report["status"] = active ? "unbound_pool" : "not_present";
                return std::nullopt;
            }
            try {
                Triangle result{};
                for (unsigned k = 0; k < 3; ++k)
                    result[k] = style == 1 ? index(indices.at(corners[k]), count)
                                           : index(Json(corners[k] + 1), count);
                return result;
            } catch (const std::exception &) {
                report["status"] = "invalid_indices";
                report["invalid_triangles"] = report.at("invalid_triangles").get<std::size_t>() + 1;
                return std::nullopt;
            }
        };
        for (std::size_t f = 0; f < polygons.size(); ++f) {
            const auto &corners = polygons[f];
            out.report["polygons"].push_back({{"source_polygon", f}, {"source_corners", corners}});
            if (corners.empty())
                continue;
            require(corners.size() >= 3, "BGFB Polyface face has fewer than three corners");
            std::vector<std::uint32_t> indices;
            std::vector<Point3> points;
            for (auto corner : corners) {
                const auto id = style == 1 ? index(point_indices[corner], g.vertices.size())
                                           : std::uint32_t(corner);
                indices.push_back(id);
                points.push_back(g.vertices[id]);
            }
            // Normalize the projection scale, retaining original 3D positions.
            double scale = 0;
            const auto origin = points.front();
            for (auto &point : points)
                for (unsigned axis = 0; axis < 3; ++axis) {
                    point[axis] -= origin[axis];
                    require(std::isfinite(point[axis]), "BGFB Polyface projection overflow");
                    scale = std::max(scale, std::abs(point[axis]));
                }
            require(scale > 0, "BGFB Polyface collapsed polygon");
            for (auto &point : points)
                for (auto &v : point)
                    v /= scale;
            Point3 normal{};
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto &a = points[i], &b = points[(i + 1) % points.size()];
                for (unsigned axis = 0; axis < 3; ++axis)
                    normal[axis] += a[(axis + 1) % 3] * b[(axis + 2) % 3] -
                                    a[(axis + 2) % 3] * b[(axis + 1) % 3];
            }
            unsigned axis = 0;
            for (unsigned k = 1; k < 3; ++k)
                if (std::abs(normal[k]) > std::abs(normal[axis]))
                    axis = k;
            require(normal[axis] != 0, "BGFB Polyface degenerate projected area");
            const auto x = (axis + 1) % 3, y = (axis + 2) % 3;
            auto orient = [&](const Point3 &a, const Point3 &b, const Point3 &c) {
                return (static_cast<long double>(b[x]) - a[x]) *
                           (static_cast<long double>(c[y]) - a[y]) -
                       (static_cast<long double>(b[y]) - a[y]) *
                           (static_cast<long double>(c[x]) - a[x]);
            };
            auto on_segment = [&](const Point3 &a, const Point3 &b, const Point3 &p) {
                return p[x] >= std::min(a[x], b[x]) && p[x] <= std::max(a[x], b[x]) &&
                       p[y] >= std::min(a[y], b[y]) && p[y] <= std::max(a[y], b[y]);
            };
            auto edge_tests = out.report.at("polygon_edge_tests").get<std::size_t>();
            for (std::size_t i = 0; i < points.size(); ++i)
                for (std::size_t j = i + 2; j < points.size(); ++j) {
                    if (i == 0 && j + 1 == points.size())
                        continue;
                    require(edge_tests < options.max_polygon_edge_tests,
                            "BGFB Polyface edge validation budget");
                    ++edge_tests;
                    const auto &a = points[i], &b = points[(i + 1) % points.size()], &c = points[j],
                               &d = points[(j + 1) % points.size()];
                    const auto ac = orient(a, b, c), ad = orient(a, b, d), ca = orient(c, d, a),
                               cb = orient(c, d, b);
                    const bool crosses = ((ac > 0 && ad < 0) || (ac < 0 && ad > 0)) &&
                                         ((ca > 0 && cb < 0) || (ca < 0 && cb > 0));
                    const bool touches =
                        (ac == 0 && on_segment(a, b, c)) || (ad == 0 && on_segment(a, b, d)) ||
                        (ca == 0 && on_segment(c, d, a)) || (cb == 0 && on_segment(c, d, b));
                    require(!crosses && !touches,
                            "BGFB Polyface self-intersecting or touching polygon");
                }
            out.report["polygon_edge_tests"] = edge_tests;
            const auto triangles =
                corners.size() == 3 ? std::vector<Triangle>{{0, 1, 2}} : polygon_faces(points);
            require(!triangles.empty(), "BGFB Polyface polygon could not be triangulated");
            double area = 0;
            for (const auto &t : triangles) {
                const auto &a = points[t[0]], &b = points[t[1]], &c = points[t[2]];
                const double twice = (b[x] - a[x]) * (c[y] - a[y]) - (b[y] - a[y]) * (c[x] - a[x]);
                require(twice * normal[axis] > 0, "BGFB Polyface degenerate or reversed triangle");
                area += twice;
            }
            require(std::abs(area - normal[axis]) <=
                        1e-10 * std::abs(normal[axis]) * corners.size(),
                    "BGFB Polyface triangulation area mismatch");
            require(triangles.size() <= options.max_triangles - g.faces.size(),
                    "BGFB Polyface triangle budget");
            for (const auto &t : triangles) {
                const std::array<std::size_t, 3> source{corners[t[0]], corners[t[1]],
                                                        corners[t[2]]};
                g.faces.push_back({indices[t[0]], indices[t[1]], indices[t[2]]});
                g.face_source_polygons.push_back(std::uint32_t(f));
                out.face_source_corners.push_back(source);
                g.face_normal_indices.push_back(binding("normalIndex", "normal", g.normals.size(),
                                                        present(table, "normal"), source));
                g.face_uv_indices.push_back(
                    binding("paramIndex", "param", g.uvs.size(), present(table, "param"), source));
                // Keep all color pools. No unproved precedence between integer,
                // floating RGB, and table colors is applied here.
                out.face_double_color_indices.push_back(
                    binding("colorIndex", "doubleColor", colors.size(),
                            present(table, "doubleColor"), source));
                out.face_int_color_indices.push_back(binding("colorIndex", "intColor",
                                                             integer_colors.size(),
                                                             present(table, "intColor"), source));
                out.face_color_table_indices.push_back(
                    binding("colorIndex", "colorTable", color_table.size(),
                            present(table, "colorTable"), source));
                if (g.face_uv_indices.back()) {
                    const auto &u = *g.face_uv_indices.back();
                    g.face_uvs.push_back(
                        std::array<Point2, 3>{g.uvs[u[0]], g.uvs[u[1]], g.uvs[u[2]]});
                } else
                    g.face_uvs.push_back(std::nullopt);
            }
        }
        out.status = "meshed";
    } catch (const std::exception &e) {
        out.geometry = {};
        out.face_source_corners.clear();
        out.face_double_color_indices.clear();
        out.face_int_color_indices.clear();
        out.face_color_table_indices.clear();
        out.report["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
