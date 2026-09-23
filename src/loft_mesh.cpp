#include "internal.hpp"
#include "bspline_denominator.hpp"
#include "glu/sk_glu.h"
#include <deque>

namespace p3d {
namespace {
Point3 difference(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        a[k] -= b[k];
    return a;
}
Point3 cross_product(Point3 a, Point3 b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot_product(Point3 a, Point3 b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double length(Point3 a) {
    return std::hypot(a[0], a[1], a[2]);
}
std::vector<double> parameters(const BsplineDirection &direction, double step, unsigned limit) {
    require(!direction.closed() && direction.knot_domain() == std::array<double, 2>{0, 1},
            "loft mesh requires prepared open normalized surfaces");
    std::vector<double> result{0};
    for (const double knot : direction.knots()) {
        if (knot <= result.back() || knot > 1)
            continue;
        const double count = std::ceil((knot - result.back()) / step);
        require(std::isfinite(count) && count >= 1 && count <= limit - result.size(),
                "loft mesh parameter budget");
        const auto n = static_cast<unsigned>(count);
        const double start = result.back();
        for (unsigned i = 1; i < n; ++i) {
            const double t = start + (knot - start) * (double(i) / n);
            require(t > result.back() && t < knot, "loft mesh parameter precision");
            result.push_back(t);
        }
        result.push_back(knot);
    }
    require(result.back() == 1, "loft mesh parameter domain");
    return result;
}
struct CapVertex {
    Point3 projected;
    std::uint32_t index;
};
struct CapTess {
    std::deque<CapVertex> vertices;
    std::vector<std::uint32_t> corners;
    std::exception_ptr failure;
    std::size_t limit;
};
template <class F> void guarded(CapTess &context, F fn) noexcept {
    if (context.failure)
        return;
    try {
        fn();
    } catch (...) {
        context.failure = std::current_exception();
    }
}
void begin(unsigned primitive, void *data) noexcept {
    guarded(*static_cast<CapTess *>(data),
            [&] { require(primitive == GL_TRIANGLES, "loft cap tessellation primitive"); });
}
void vertex(void *data, void *context) noexcept {
    auto &c = *static_cast<CapTess *>(context);
    guarded(c, [&] {
        require(data && c.corners.size() < c.limit, "loft cap triangle budget");
        c.corners.push_back(static_cast<CapVertex *>(data)->index);
    });
}
void combine(double[3], void *[4], float[4], void **result, void *data) noexcept {
    *result = nullptr;
    guarded(*static_cast<CapTess *>(data), [] {
        throw std::runtime_error(
            "loft cap intersection or coincident vertices require boundary splitting");
    });
}
void error(unsigned code, void *data) noexcept {
    guarded(*static_cast<CapTess *>(data), [&] {
        throw std::runtime_error("loft cap tessellation error " + std::to_string(code));
    });
}
void edge(unsigned char, void *) noexcept {}
using Ring = std::vector<std::uint32_t>;
using EdgeKey = std::array<std::uint32_t, 2>;
EdgeKey edge_key(std::uint32_t a, std::uint32_t b) {
    return {std::min(a, b), std::max(a, b)};
}
std::vector<Triangle> restore_boundary_samples(const std::vector<Triangle> &faces,
                                               const std::vector<Ring> &rings,
                                               const std::vector<Point3> &vertices, unsigned axis,
                                               unsigned budget, double collinear_distance) {
    std::map<EdgeKey, unsigned> edges;
    for (const auto &face : faces)
        for (unsigned k = 0; k < 3; ++k)
            ++edges[edge_key(face[k], face[(k + 1) % 3])];
    auto projected = [&](std::uint32_t id) {
        std::array<long double, 2> out{};
        unsigned n = 0;
        for (unsigned k = 0; k < 3; ++k)
            if (k != axis)
                out[n++] = vertices[id][k];
        return out;
    };
    using Coordinate = std::pair<long double, std::uint32_t>;
    std::array<std::vector<Coordinate>, 2> ordered;
    std::set<std::uint32_t> seen;
    for (const auto &ring : rings)
        for (auto id : ring) {
            require(seen.insert(id).second, "loft cap repeated boundary vertex");
            const auto p = projected(id);
            for (unsigned k = 0; k < 2; ++k)
                ordered[k].emplace_back(p[k], id);
        }
    for (auto &list : ordered)
        std::sort(list.begin(), list.end());
    std::size_t checks = 0;
    std::map<EdgeKey, Ring> split;
    for (const auto &edge : edges) {
        const auto a = edge.first[0], b = edge.first[1];
        const auto pa = projected(a), pb = projected(b);
        const long double dx = pb[0] - pa[0], dy = pb[1] - pa[1], len2 = dx * dx + dy * dy;
        require(len2 > 0 && std::isfinite(len2), "loft cap collapsed boundary edge");
        const unsigned coordinate = std::abs(dx) >= std::abs(dy) ? 0 : 1;
        const auto &list = ordered[coordinate];
        const auto first = std::lower_bound(
            list.begin(), list.end(), Coordinate{std::min(pa[coordinate], pb[coordinate]), 0});
        const auto last =
            std::upper_bound(list.begin(), list.end(),
                             Coordinate{std::max(pa[coordinate], pb[coordinate]), UINT32_MAX});
        std::vector<Coordinate> middle;
        for (auto i = first; i != last; ++i) {
            require(checks++ < std::size_t(budget) * 32, "loft cap boundary search budget");
            if (i->second == a || i->second == b)
                continue;
            const auto p = projected(i->second);
            const auto x = p[0] - pa[0], y = p[1] - pa[1], cross = dx * y - dy * x,
                       along = dx * x + dy * y;
            if (along > 0 && along < len2 &&
                std::abs(cross) <= collinear_distance * std::sqrt(len2))
                middle.emplace_back(along, i->second);
        }
        if (middle.empty())
            continue;
        std::sort(middle.begin(), middle.end());
        Ring path{a};
        for (const auto &p : middle)
            path.push_back(p.second);
        path.push_back(b);
        split.emplace(edge.first, std::move(path));
    }
    std::vector<Triangle> result;
    for (const auto &original : faces) {
        std::vector<Triangle> local{original};
        for (unsigned edge = 0; edge < 3; ++edge) {
            const auto key = edge_key(original[edge], original[(edge + 1) % 3]);
            const auto at = split.find(key);
            if (at == split.end())
                continue;
            bool done = false;
            for (std::size_t f = 0; f < local.size() && !done; ++f)
                for (unsigned k = 0; k < 3; ++k) {
                    const auto current = local[f];
                    if (edge_key(current[k], current[(k + 1) % 3]) != key)
                        continue;
                    auto path = at->second;
                    if (path.front() != current[k])
                        std::reverse(path.begin(), path.end());
                    const auto opposite = current[(k + 2) % 3];
                    require(path.size() - 2 <= budget && local.size() <= budget - (path.size() - 2),
                            "loft cap boundary subdivision budget");
                    local[f] = {path[0], path[1], opposite};
                    for (std::size_t i = 1; i + 1 < path.size(); ++i)
                        local.push_back({path[i], path[i + 1], opposite});
                    done = true;
                    break;
                }
            require(done, "loft cap subdivision edge not found");
        }
        require(local.size() <= budget - result.size(), "loft cap triangle budget");
        result.insert(result.end(), local.begin(), local.end());
    }
    return result;
}
std::vector<Triangle> cap_faces(const std::vector<Ring> &rings, const std::vector<Point3> &vertices,
                                const Json &region, const LoftMeshOptions &options, unsigned budget,
                                double &max_plane_error, double &max_collinear_distance,
                                unsigned &plane_budget, Json &plane_reports) {
    require(!rings.empty() && rings[0].size() >= 3, "loft cap boundary vertex count");
    const auto origin = vertices[rings[0][0]];
    Point3 normal{};
    for (std::size_t i = 0; i < rings[0].size(); ++i) {
        const auto n =
            cross_product(difference(vertices[rings[0][i]], origin),
                          difference(vertices[rings[0][(i + 1) % rings[0].size()]], origin));
        for (unsigned k = 0; k < 3; ++k)
            normal[k] += n[k];
    }
    const auto magnitude = length(normal);
    require(std::isfinite(magnitude) && magnitude > 0, "loft cap has no finite oriented plane");
    for (auto &n : normal)
        n /= magnitude;
    auto check_plane = [&](Point3 p) {
        const auto d = std::abs(dot_product(difference(p, origin), normal));
        require(std::isfinite(d) && d <= options.planarity_tolerance,
                "loft cap boundary is not planar within tolerance");
        max_plane_error = std::max(max_plane_error, d);
    };
    // Same-sign weight cap curves lie in their Cartesian control hull. Check
    // the full curves, not just the chosen mesh samples.
    std::function<void(const Json &)> check_curves = [&](const Json &j) {
        if (j.is_object()) {
            if (j.value("_type", std::string()) == "BsplineCurve") {
                const auto curve = BsplineCurve::from_bgfb(j);
                bool same_sign = true;
                if (curve.rational()) {
                    const bool positive = curve.weights().front() > 0;
                    for (double w : curve.weights())
                        same_sign &= w != 0 && (w > 0) == positive;
                }
                if (!same_sign) {
                    auto proof = certify_curve_plane(curve, origin, normal,
                                                     options.planarity_tolerance, plane_budget);
                    const auto used = proof.at("work_steps").get<unsigned>();
                    require(used <= plane_budget, "loft cap plane work accounting");
                    plane_budget -= used;
                    proof["curve_index"] = plane_reports.size();
                    plane_reports.push_back(proof);
                    require(proof.at("status") == "verified",
                            "loft cap plane bound not established");
                    max_plane_error =
                        std::max(max_plane_error, proof.at("distance_bound").get<double>());
                    return;
                }
                double curve_bound = 0;
                for (std::size_t i = 0; i < curve.poles().size(); ++i) {
                    auto p = curve.poles()[i];
                    if (curve.rational()) {
                        for (auto &x : p)
                            x /= curve.weights()[i];
                    }
                    check_plane(p);
                    curve_bound =
                        std::max(curve_bound, std::abs(dot_product(difference(p, origin), normal)));
                }
                plane_reports.push_back({{"curve_index", plane_reports.size()},
                                         {"status", "verified"},
                                         {"method", "control_hull"},
                                         {"distance_bound", curve_bound}});
            } else
                for (const auto &v : j)
                    if (v.is_structured())
                        check_curves(v);
        } else if (j.is_array())
            for (const auto &v : j)
                check_curves(v);
    };
    check_curves(region);
    unsigned axis = 0;
    for (unsigned k = 1; k < 3; ++k)
        if (std::abs(normal[k]) > std::abs(normal[axis]))
            axis = k;
    // The input coordinates themselves have rounding uncertainty, especially
    // after translation. Use one distance threshold for both projected sliver
    // removal and boundary reinsertion; never exceed the caller's join limit.
    double coordinate_scale = 1;
    for (const auto &ring : rings)
        for (auto index : ring)
            for (unsigned k = 0; k < 3; ++k)
                if (k != axis)
                    coordinate_scale = std::max(coordinate_scale, std::abs(vertices[index][k]));
    const double collinear_distance = std::min(
        options.join_tolerance, 64 * std::numeric_limits<double>::epsilon() * coordinate_scale);
    max_collinear_distance = std::max(max_collinear_distance, collinear_distance);
    CapTess context;
    context.limit = std::size_t(budget) * 3;
    std::unique_ptr<GLUtesselator, decltype(&gluDeleteTess)> tess(gluNewTess(), gluDeleteTess);
    require(bool(tess), "loft cap tessellator allocation");
    gluTessCallback(tess.get(), GLU_TESS_BEGIN_DATA, reinterpret_cast<void (*)()>(begin));
    gluTessCallback(tess.get(), GLU_TESS_VERTEX_DATA, reinterpret_cast<void (*)()>(vertex));
    gluTessCallback(tess.get(), GLU_TESS_COMBINE_DATA, reinterpret_cast<void (*)()>(combine));
    gluTessCallback(tess.get(), GLU_TESS_ERROR_DATA, reinterpret_cast<void (*)()>(error));
    gluTessCallback(tess.get(), GLU_TESS_EDGE_FLAG_DATA, reinterpret_cast<void (*)()>(edge));
    gluTessProperty(tess.get(), GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_ODD);
    gluTessProperty(tess.get(), GLU_TESS_TOLERANCE, 0);
    gluTessNormal(tess.get(), 0, 0, 1);
    std::vector<std::size_t> counts;
    for (const auto &ring : rings) {
        require(ring.size() >= 3, "loft cap ring has too few vertices");
        counts.push_back(ring.size());
        for (auto index : ring) {
            check_plane(vertices[index]);
            const auto p = difference(vertices[index], origin);
            Point3 projected{};
            unsigned dest = 0;
            for (unsigned k = 0; k < 3; ++k)
                if (k != axis) {
                    require(std::isfinite(p[k]) && std::abs(p[k]) <= GLU_TESS_MAX_COORD,
                            "loft cap projection range");
                    projected[dest++] = p[k];
                }
            context.vertices.push_back({projected, index});
        }
    }
    gluTessBeginPolygon(tess.get(), &context);
    std::size_t index = 0;
    for (auto count : counts) {
        gluTessBeginContour(tess.get());
        for (std::size_t i = 0; i < count; ++i, ++index)
            gluTessVertex(tess.get(), context.vertices[index].projected.data(),
                          &context.vertices[index]);
        gluTessEndContour(tess.get());
    }
    gluTessEndPolygon(tess.get());
    if (context.failure)
        std::rethrow_exception(context.failure);
    require(context.corners.size() % 3 == 0, "loft cap incomplete triangle");
    std::vector<Triangle> faces;
    for (std::size_t i = 0; i < context.corners.size(); i += 3) {
        Triangle face{context.corners[i], context.corners[i + 1], context.corners[i + 2]};
        // Classify in the same 2D plane as the tessellation. A projected zero
        // area triangle can have a tiny nonzero 3D normal from roundoff in
        // otherwise planar input; it must not survive as a cap sliver.
        std::array<long double, 2> b{}, c{};
        unsigned coordinate = 0;
        for (unsigned k = 0; k < 3; ++k)
            if (k != axis) {
                b[coordinate] =
                    static_cast<long double>(vertices[face[1]][k]) - vertices[face[0]][k];
                c[coordinate] =
                    static_cast<long double>(vertices[face[2]][k]) - vertices[face[0]][k];
                ++coordinate;
            }
        const auto area = b[0] * c[1] - b[1] * c[0];
        const auto edge2 =
            std::max({b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1],
                      (b[0] - c[0]) * (b[0] - c[0]) + (b[1] - c[1]) * (b[1] - c[1])});
        if (std::abs(area) <= collinear_distance * std::sqrt(edge2))
            continue;
        const auto orientation = area * (axis == 1 ? -normal[axis] : normal[axis]);
        require(std::isfinite(orientation), "loft cap nonfinite triangle");
        if (orientation < 0)
            std::swap(face[1], face[2]);
        faces.push_back(face);
    }
    faces = restore_boundary_samples(faces, rings, vertices, axis, budget, collinear_distance);
    // Every cap perimeter segment must survive triangulation; no silent
    // removal of collinear samples that would leave side/cap T junctions.
    std::map<std::array<std::uint32_t, 2>, unsigned> expected, actual;
    auto key = [](std::uint32_t a, std::uint32_t b) {
        return std::array<std::uint32_t, 2>{std::min(a, b), std::max(a, b)};
    };
    for (const auto &ring : rings)
        for (std::size_t i = 0; i < ring.size(); ++i)
            ++expected[key(ring[i], ring[(i + 1) % ring.size()])];
    for (const auto &face : faces)
        for (unsigned i = 0; i < 3; ++i)
            ++actual[key(face[i], face[(i + 1) % 3])];
    for (const auto &item : expected)
        require(item.second == 1 && actual[item.first] == 1, "loft cap perimeter is not preserved");
    for (const auto &item : actual)
        require(item.second == (expected.count(item.first) ? 1u : 2u),
                "loft cap has an internal crack or nonmanifold edge");
    return faces;
}
} // namespace

LoftMesh SectionLoft::mesh(const LoftMeshOptions &options) const {
    require(std::isfinite(options.max_uv_edge) && options.max_uv_edge > 0 &&
                std::isfinite(options.join_tolerance) && options.join_tolerance >= 0 &&
                std::isfinite(options.planarity_tolerance) && options.planarity_tolerance >= 0 &&
                options.max_vertices >= 3 && options.max_triangles > 0 &&
                options.max_cap_control_points > 0 && options.max_denominator_steps > 0 &&
                options.max_planarity_steps > 0,
            "invalid loft mesh options");
    LoftMesh out;
    out.report = {{"status", "incomplete"},
                  {"representation", "derived_loft_surface_mesh"},
                  {"world_space_error_bound", nullptr},
                  {"self_intersections", "not_evaluated"},
                  {"max_side_uv_edge", options.max_uv_edge},
                  {"join_tolerance", options.join_tolerance},
                  {"planarity_tolerance", options.planarity_tolerance}};
    out.report["side_denominators"] = Json::array();
    out.report["cap_planarity"] = {{"bottom", Json::array()}, {"top", Json::array()}};
    try {
        const bool capped = source_.at("capped").get<bool>();
        const auto caps = cap_regions(options.max_cap_control_points);
        out.report["caps"] = caps.report;
        require(!capped || caps.report.at("status") == "complete", "loft cap regions unavailable");
        const auto step = options.max_uv_edge / std::sqrt(2.0);
        require(step > 0, "loft mesh parameter step underflow");
        double max_join = 0, max_plane = 0, max_collinear = 0;
        unsigned plane_budget = options.max_planarity_steps;
        auto add_vertex = [&](Point3 p) {
            require(out.vertices.size() < options.max_vertices, "loft mesh vertex budget");
            for (double x : p)
                require(std::isfinite(x), "nonfinite loft mesh vertex");
            const auto id = std::uint32_t(out.vertices.size());
            out.vertices.push_back(p);
            return id;
        };
        auto join = [&](std::uint32_t id, Point3 p) {
            const double error = length(difference(p, out.vertices.at(id)));
            require(std::isfinite(error) && error <= options.join_tolerance,
                    "adjacent loft boundaries exceed join tolerance");
            max_join = std::max(max_join, error);
            return id;
        };
        auto emit = [&](Triangle face, std::optional<std::array<Point2, 3>> uv) {
            require(out.faces.size() < options.max_triangles, "loft mesh triangle budget");
            const double area =
                length(cross_product(difference(out.vertices[face[1]], out.vertices[face[0]]),
                                     difference(out.vertices[face[2]], out.vertices[face[0]])));
            require(std::isfinite(area) && area > 0, "loft mesh has a degenerate triangle");
            out.faces.push_back(face);
            out.face_parameters.push_back(uv);
        };
        std::array<std::vector<Ring>, 2> rings;
        std::size_t side_index = 0;
        for (const auto &loop : report_.at("loops")) {
            const auto count = loop.at("side_count").get<std::size_t>();
            require(count && count <= sides_.size() - side_index, "loft mesh side groups");
            const bool closed_loop = loop.at("boundary_type") != 1;
            const auto v = parameters(sides_[side_index].surface.v(), step, options.max_vertices);
            std::vector<std::uint32_t> first_column, previous_column;
            std::array<Ring, 2> boundary;
            for (std::size_t segment = 0; segment < count; ++segment, ++side_index) {
                const auto &surface = sides_[side_index].surface;
                auto denominator =
                    certify_surface_denominator(surface, options.max_denominator_steps);
                denominator["side_index"] = side_index;
                out.report["side_denominators"].push_back(denominator);
                require(denominator.at("status") == "verified",
                        "loft side denominator sign not established for mesh");
                require(surface.v().knots() == sides_[side_index - segment].surface.v().knots(),
                        "loft guides have incompatible sample parameters");
                const auto u = parameters(surface.u(), step, options.max_vertices);
                require(u.size() <= options.max_vertices / v.size(), "loft mesh grid budget");
                std::vector<std::uint32_t> grid(u.size() * v.size());
                for (std::size_t j = 0; j < v.size(); ++j)
                    for (std::size_t i = 0; i < u.size(); ++i) {
                        const auto p = surface.point_at(u[i], v[j]);
                        std::uint32_t id;
                        if (i == 0 && segment)
                            id = join(previous_column[j], p);
                        else if (i + 1 == u.size() && segment + 1 == count &&
                                 (closed_loop || (capped && (j == 0 || j + 1 == v.size()))))
                            id = join(first_column[j], p);
                        else
                            id = add_vertex(p);
                        grid[j * u.size() + i] = id;
                        if (segment == 0 && i == 0)
                            first_column.push_back(id);
                    }
                previous_column.clear();
                for (std::size_t j = 0; j < v.size(); ++j)
                    previous_column.push_back(grid[j * u.size() + u.size() - 1]);
                for (unsigned end = 0; end < 2; ++end) {
                    const auto row = end ? v.size() - 1 : 0;
                    for (std::size_t i = 0; i + 1 < u.size(); ++i)
                        boundary[end].push_back(grid[row * u.size() + i]);
                    if (!closed_loop && !capped && segment + 1 == count)
                        boundary[end].push_back(grid[row * u.size() + u.size() - 1]);
                }
                const auto start = out.faces.size();
                for (std::size_t j = 0; j + 1 < v.size(); ++j)
                    for (std::size_t i = 0; i + 1 < u.size(); ++i) {
                        const auto a = grid[j * u.size() + i], b = grid[j * u.size() + i + 1],
                                   c = grid[(j + 1) * u.size() + i + 1],
                                   d = grid[(j + 1) * u.size() + i];
                        emit({a, b, c},
                             std::array<Point2, 3>{Point2{u[i], v[j]}, Point2{u[i + 1], v[j]},
                                                   Point2{u[i + 1], v[j + 1]}});
                        emit({a, c, d},
                             std::array<Point2, 3>{Point2{u[i], v[j]}, Point2{u[i + 1], v[j + 1]},
                                                   Point2{u[i], v[j + 1]}});
                    }
                out.parts.push_back(
                    {"side", side_index, start, out.faces.size() - start,
                     std::array<std::int64_t, 3>{0, static_cast<std::int64_t>(side_index), 0}});
            }
            std::reverse(boundary[0].begin(), boundary[0].end());
            for (unsigned end = 0; end < 2; ++end)
                rings[end].push_back(std::move(boundary[end]));
        }
        require(side_index == sides_.size(), "loft mesh side coverage");
        if (capped)
            for (unsigned end = 0; end < 2; ++end) {
                const auto start = out.faces.size();
                auto faces = cap_faces(rings[end], out.vertices, end ? caps.top : caps.bottom,
                                       options, options.max_triangles - unsigned(out.faces.size()),
                                       max_plane, max_collinear, plane_budget,
                                       out.report["cap_planarity"][end ? "top" : "bottom"]);
                for (auto face : faces)
                    emit(face, {});
                out.parts.push_back(
                    {end ? "top" : "bottom",
                     {},
                     start,
                     out.faces.size() - start,
                     std::array<std::int64_t, 3>{-1, static_cast<std::int64_t>(end), 0}});
            }
        std::map<std::array<std::uint32_t, 2>, std::pair<unsigned, int>> edges;
        for (const auto &face : out.faces)
            for (unsigned i = 0; i < 3; ++i) {
                const auto a = face[i], b = face[(i + 1) % 3];
                auto &edge = edges[{std::min(a, b), std::max(a, b)}];
                ++edge.first;
                edge.second += a < b ? 1 : -1;
            }
        std::size_t boundary = 0, nonmanifold = 0, orientation = 0;
        for (const auto &e : edges) {
            boundary += e.second.first == 1;
            nonmanifold += e.second.first > 2;
            orientation += e.second.first == 2 && e.second.second != 0;
        }
        out.report["indexed_edge_topology"] = {
            {"boundary_edges", boundary},
            {"nonmanifold_edges", nonmanifold},
            {"orientation_conflicts", orientation},
            {"closed_oriented_edges", !boundary && !nonmanifold && !orientation}};
        out.report["max_join_displacement"] = max_join;
        out.report["max_cap_plane_distance"] = max_plane;
        out.report["cap_collinearity_distance"] = max_collinear;
        out.report["vertices"] = out.vertices.size();
        out.report["triangles"] = out.faces.size();
        const auto native = face_indices(options.max_cap_control_points);
        out.report["native_face_indices"] = native.report;
        if (native.report.at("status") != "complete")
            for (auto &part : out.parts)
                part.native_face_indices.reset();
        out.report["status"] = "complete";
    } catch (const std::exception &e) {
        out.report["reason"] = e.what();
        out.vertices.clear();
        out.faces.clear();
        out.face_parameters.clear();
        out.parts.clear();
    }
    return out;
}
} // namespace p3d
