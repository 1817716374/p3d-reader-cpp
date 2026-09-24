#include <p3d/polyface.hpp>
#include "geometry.hpp"
#include <cstring>

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
// Exact zero determinant for finite binary64 coordinates. Separate positive
// and negative integer accumulators avoid deleting a tiny, nonzero feature
// because a floating cross product rounded to zero. Product exponents span
// -2148..2047; 132 base-2^32 words also cover the sum of six products.
bool collinear_projection(const Point3 &a, const Point3 &b, const Point3 &c, unsigned x,
                          unsigned y) {
    if ((a[x] == b[x] && b[x] == c[x]) || (a[y] == b[y] && b[y] == c[y]))
        return true;
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    using Sum = std::array<std::uint32_t, 132>;
    Sum positive{}, negative{};
    auto product = [&](double u, double v, bool subtract) {
        std::uint64_t ub, vb;
        std::memcpy(&ub, &u, 8);
        std::memcpy(&vb, &v, 8);
        const auto ue = unsigned((ub >> 52) & 2047), ve = unsigned((vb >> 52) & 2047);
        auto um = ub & ((UINT64_C(1) << 52) - 1);
        auto vm = vb & ((UINT64_C(1) << 52) - 1);
        if (ue)
            um |= UINT64_C(1) << 52;
        if (ve)
            vm |= UINT64_C(1) << 52;
        if (!um || !vm)
            return;
        const unsigned shift =
            unsigned((ue ? int(ue) - 1075 : -1074) + (ve ? int(ve) - 1075 : -1074) + 2148);
        auto &sum = ((ub >> 63) ^ (vb >> 63) ^ subtract) ? negative : positive;
        auto add_word = [&](std::uint32_t word, unsigned bit) {
            std::uint64_t carry = std::uint64_t(word) << (bit % 32);
            for (std::size_t j = bit / 32; carry; ++j) {
                require(j < sum.size(), "BGFB Polyface exact determinant capacity");
                carry += sum[j];
                sum[j] = std::uint32_t(carry);
                carry >>= 32;
            }
        };
        for (unsigned i = 0; i < 2; ++i)
            for (unsigned j = 0; j < 2; ++j) {
                const auto part =
                    std::uint64_t(std::uint32_t(um >> (32 * i))) * std::uint32_t(vm >> (32 * j));
                add_word(std::uint32_t(part), shift + 32 * (i + j));
                add_word(std::uint32_t(part >> 32), shift + 32 * (i + j + 1));
            }
    };
    product(a[x], b[y], false);
    product(b[x], c[y], false);
    product(c[x], a[y], false);
    product(a[y], b[x], true);
    product(b[y], c[x], true);
    product(c[y], a[x], true);
    return positive == negative;
}
// Cancel exact zero-area folds. In the projection pass, retain a non-collinear
// spatial ear as a triangle. Original pools and source corners survive.
// A linked ring plus local worklist bounds the candidate checks by 3*n.
using BoundaryTriangle = std::pair<Triangle, std::array<std::size_t, 3>>;
void reduce_retraced_edges(std::vector<Point3> &points, std::vector<std::uint32_t> &indices,
                           std::vector<std::size_t> &corners, Json &report,
                           std::optional<unsigned> projection_axis = {},
                           std::vector<BoundaryTriangle> *split_triangles = nullptr) {
    const auto n = points.size();
    std::vector<std::size_t> previous(n), next(n), pending;
    std::vector<bool> alive(n, true);
    for (std::size_t i = 0; i < n; ++i) {
        previous[i] = (i + n - 1) % n;
        next[i] = (i + 1) % n;
        pending.push_back(n - 1 - i);
    }
    std::size_t remaining = n, tests = 0;
    const char *changes = projection_axis ? "projected_boundary_splits" : "boundary_reductions";
    report[changes] = Json::array();
    while (!pending.empty() && remaining >= 3) {
        const auto i = pending.back();
        pending.pop_back();
        if (!alive[i])
            continue;
        ++tests;
        const auto p = previous[i], q = next[i];
        const auto &a = points[p], &b = points[i], &c = points[q];
        const bool duplicate = a == b || b == c;
        bool reversal = false;
        for (unsigned k = 0; k < 3; ++k)
            reversal |= (a[k] < b[k] && c[k] < b[k]) || (a[k] > b[k] && c[k] > b[k]);
        bool exact = duplicate ||
                     (reversal && collinear_projection(a, b, c, 0, 1) &&
                      collinear_projection(a, b, c, 1, 2) && collinear_projection(a, b, c, 2, 0));
        bool split = false;
        if (!exact && projection_axis) {
            const auto x = (*projection_axis + 1) % 3, y = (*projection_axis + 2) % 3;
            bool projected_reversal = false;
            for (auto k : {x, y})
                projected_reversal |= (a[k] < b[k] && c[k] < b[k]) || (a[k] > b[k] && c[k] > b[k]);
            const bool zero_edge = (a[x] == b[x] && a[y] == b[y]) || (b[x] == c[x] && b[y] == c[y]);
            split = (projected_reversal || zero_edge) && collinear_projection(a, b, c, x, y);
            if (split && collinear_projection(a, b, c, 0, 1) &&
                collinear_projection(a, b, c, 1, 2) && collinear_projection(a, b, c, 2, 0)) {
                // An edge along the projection normal can be forward-collinear
                // in 3D. It has no spatial triangle to retain.
                exact = true;
                split = false;
            }
        }
        if (!exact && !split)
            continue;
        if (split)
            split_triangles->push_back(
                {{indices[p], indices[i], indices[q]}, {corners[p], corners[i], corners[q]}});
        report[changes].push_back({{"source_corner", corners[i]},
                                   {"previous_source_corner", corners[p]},
                                   {"next_source_corner", corners[q]},
                                   {"reason", split       ? "retained_spatial_triangle"
                                              : duplicate ? "zero_length_edge"
                                              : reversal  ? "exact_collinear_retrace"
                                                          : "exact_collinear_edge"}});
        alive[i] = false;
        --remaining;
        next[p] = q;
        previous[q] = p;
        pending.push_back(p);
        pending.push_back(q);
    }
    report[projection_axis ? "projected_boundary_tests" : "boundary_reduction_tests"] = tests;
    std::size_t write = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (alive[i]) {
            points[write] = points[i];
            indices[write] = indices[i];
            corners[write] = corners[i];
            ++write;
        }
    points.resize(write);
    indices.resize(write);
    corners.resize(write);
    report["triangulation_source_corners"] = corners;
    require(write >= 3, "BGFB Polyface collapsed boundary after exact edge cancellation");
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
        const bool grid = style == 5 || style == 6;
        require(style == 1 || style == 3 || style == 4 || grid,
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
        } else if (grid) {
            const auto columns_value = table.value("numPerRow", 0);
            require(columns_value >= 2, "BGFB Polyface grid needs at least two points per row");
            const auto columns = std::size_t(columns_value), count = g.vertices.size();
            const auto rows = count / columns;
            // The native visitor's upper-right guard accepts index == count,
            // then reads that missing point. Reject such a partial cell safely.
            require(count < columns + 1 || count % columns == 0,
                    "BGFB Polyface incomplete grid row references a missing corner");
            const auto cells = rows > 1 ? (rows - 1) * (columns - 1) : 0;
            const std::size_t faces_per_cell = style == 5 ? 2 : 1;
            const std::size_t corners_per_cell = style == 5 ? 6 : 4;
            require(cells <= options.max_corners / corners_per_cell,
                    "BGFB Polyface grid corner budget");
            require(cells <= UINT32_MAX / faces_per_cell,
                    "BGFB Polyface grid source face capacity");
            out.report["num_per_row"] = columns;
            out.report["grid_rows"] = rows;
            out.report["grid_cells"] = cells;
            out.report["layout_rule"] = "native_direct_grid_visitor";
            out.report["source_corner_count"] = cells * corners_per_cell;
            out.report["unused_trailing_points"] = cells ? 0 : count;
            for (std::size_t row = 0; row + 1 < rows; ++row)
                for (std::size_t col = 0; col + 1 < columns; ++col) {
                    const auto a = row * columns + col, b = a + 1;
                    const auto c = a + columns, d = c + 1;
                    if (style == 5) {
                        polygons.push_back({a, b, c});
                        polygons.push_back({c, b, d});
                    } else
                        polygons.push_back({a, b, d, c});
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
        if (!grid)
            out.report["source_corner_count"] =
                style == 1 ? point_indices.size() : g.vertices.size();
        auto binding = [&](const char *name, const char *report_name, std::size_t count,
                           bool active, const std::array<std::size_t, 3> &corners,
                           bool allow_point_default = false) -> std::optional<Triangle> {
            auto &report = out.report["attribute_bindings"][report_name];
            if (report.is_null())
                report = {{"status", "mapped"}, {"invalid_triangles", 0}};
            const auto &indices = array(table, name);
            const bool point_default = active && style == 1 && indices.empty() &&
                                       allow_point_default && count == g.vertices.size();
            report["index_source"] = !active            ? Json(nullptr)
                                     : grid             ? Json("implicit_grid_point_index")
                                     : style != 1       ? Json("implicit_point_sequence")
                                     : point_default    ? Json("pointIndex")
                                     : !indices.empty() ? Json(name)
                                                        : Json(nullptr);
            report["binding_rule"] = point_default      ? "native_equal_point_count_default"
                                     : !active          ? "not_present"
                                     : grid             ? "implicit_grid_point_index"
                                     : style != 1       ? "implicit_point_sequence"
                                     : !indices.empty() ? "explicit_indices"
                                                        : "unbound_pool";
            if (!active || (style == 1 && indices.empty() && !point_default)) {
                report["status"] = active ? "unbound_pool" : "not_present";
                return std::nullopt;
            }
            try {
                Triangle result{};
                for (unsigned k = 0; k < 3; ++k)
                    result[k] =
                        style == 1
                            ? index((point_default ? point_indices : indices).at(corners[k]), count)
                            : index(Json(corners[k] + 1), count);
                return result;
            } catch (const std::exception &) {
                report["status"] = "invalid_indices";
                report["invalid_triangles"] = report.at("invalid_triangles").get<std::size_t>() + 1;
                return std::nullopt;
            }
        };
        for (std::size_t f = 0; f < polygons.size(); ++f) {
            auto corners = polygons[f];
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
            const auto edge_begin = out.source_edges.size();
            for (std::size_t i = 0; i < corners.size(); ++i) {
                const auto next = (i + 1) % corners.size();
                out.source_edges.push_back(
                    {f,
                     corners[i],
                     corners[next],
                     {indices[i], indices[next]},
                     style != 1 || point_indices.at(corners[i]).get<std::int64_t>() > 0});
            }
            reduce_retraced_edges(points, indices, corners, out.report["polygons"].back());
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
            // A fold can be exactly collinear only in the chosen projection.
            // Split its nonzero 3D triangle off instead of deleting that feature.
            // Predicates use source binary64 coordinates, not normalized copies.
            std::vector<Point3> spatial_points;
            for (auto id : indices)
                spatial_points.push_back(g.vertices[id]);
            std::vector<BoundaryTriangle> boundary_triangles;
            reduce_retraced_edges(spatial_points, indices, corners, out.report["polygons"].back(),
                                  axis, &boundary_triangles);
            points = std::move(spatial_points);
            for (auto &point : points)
                for (unsigned k = 0; k < 3; ++k)
                    point[k] = (point[k] - origin[k]) / scale;
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
            require(boundary_triangles.size() <= options.max_triangles - g.faces.size() &&
                        triangles.size() <=
                            options.max_triangles - g.faces.size() - boundary_triangles.size(),
                    "BGFB Polyface triangle budget");
            for (const auto &t : triangles) {
                const std::array<std::size_t, 3> source{corners[t[0]], corners[t[1]],
                                                        corners[t[2]]};
                boundary_triangles.push_back(
                    {{indices[t[0]], indices[t[1]], indices[t[2]]}, source});
            }
            for (const auto &triangle : boundary_triangles) {
                const auto &source = triangle.second;
                g.faces.push_back(triangle.first);
                g.face_source_polygons.push_back(std::uint32_t(f));
                out.face_source_corners.push_back(source);
                std::array<std::optional<PolyfaceTriangleEdgeSource>, 3> edge_sources{};
                const auto &original_corners = polygons[f];
                for (unsigned k = 0; k < 3; ++k) {
                    // Grid faces use nonconsecutive point positions. Shared
                    // grid vertices do not merge distinct source edge records.
                    auto position = [&](std::size_t corner) {
                        return grid ? std::size_t(std::find(original_corners.begin(),
                                                            original_corners.end(), corner) -
                                                  original_corners.begin())
                                    : corner - original_corners.front();
                    };
                    const auto a = position(source[k]), b = position(source[(k + 1) % 3]);
                    if ((a + 1) % original_corners.size() == b)
                        edge_sources[k] = PolyfaceTriangleEdgeSource{edge_begin + a, false};
                    else if ((b + 1) % original_corners.size() == a)
                        edge_sources[k] = PolyfaceTriangleEdgeSource{edge_begin + b, true};
                }
                out.face_source_edges.push_back(edge_sources);
                g.face_normal_indices.push_back(binding("normalIndex", "normal", g.normals.size(),
                                                        present(table, "normal"), source, true));
                g.face_uv_indices.push_back(binding("paramIndex", "param", g.uvs.size(),
                                                    present(table, "param"), source, true));
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
        out.source_edges.clear();
        out.face_source_edges.clear();
        out.face_double_color_indices.clear();
        out.face_int_color_indices.clear();
        out.face_color_table_indices.clear();
        out.report["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
