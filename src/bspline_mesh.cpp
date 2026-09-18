#include "internal.hpp"
#include "glu/sk_glu.h"
#include <deque>

namespace p3d {
namespace {
using Loop = std::vector<Point2>;
struct TessContext {
    std::deque<Point3> storage;
    std::vector<Loop> contours;
    std::vector<Point2> corners;
    std::exception_ptr failure;
    std::size_t vertices_limit, output_limit, emitted = 0;
    bool boundaries;
};
template <class F> void guarded(TessContext &context, F action) noexcept {
    if (context.failure)
        return;
    try {
        action();
    } catch (...) {
        context.failure = std::current_exception();
    }
}
void begin(unsigned primitive, void *data) noexcept {
    auto &context = *static_cast<TessContext *>(data);
    guarded(context, [&] {
        require(primitive == (context.boundaries ? GL_LINE_LOOP : GL_TRIANGLES),
                "surface trim tessellation primitive");
        if (context.boundaries)
            context.contours.emplace_back();
    });
}
void vertex(void *point, void *data) noexcept {
    auto &context = *static_cast<TessContext *>(data);
    guarded(context, [&] {
        require(point != nullptr, "surface trim tessellation missing vertex");
        require(context.emitted++ < context.output_limit,
                "surface trim tessellation output budget");
        const auto &p = *static_cast<Point3 *>(point);
        const Point2 uv{p[0], p[1]};
        if (context.boundaries) {
            require(!context.contours.empty(), "surface trim boundary sequence");
            context.contours.back().push_back(uv);
        } else {
            context.corners.push_back(uv);
        }
    });
}
void combine(double[3], void *sources[4], float[4], void **result, void *data) noexcept {
    auto &context = *static_cast<TessContext *>(data);
    *result = nullptr;
    guarded(context, [&] {
        require(context.storage.size() < context.vertices_limit,
                "surface trim intersection budget");
        require(sources[0] && sources[1], "surface trim combine source vertices");
        const auto a = *static_cast<Point3 *>(sources[0]);
        const auto b = *static_cast<Point3 *>(sources[1]);
        Point3 point = a;
        if (sources[2] && sources[3]) {
            const auto c = *static_cast<Point3 *>(sources[2]);
            const auto d = *static_cast<Point3 *>(sources[3]);
            const long double x = static_cast<long double>(b[0]) - a[0],
                              y = static_cast<long double>(b[1]) - a[1],
                              u = static_cast<long double>(d[0]) - c[0],
                              v = static_cast<long double>(d[1]) - c[1],
                              p = static_cast<long double>(c[0]) - a[0],
                              q = static_cast<long double>(c[1]) - a[1];
            const auto determinant = x * v - y * u;
            if (determinant == 0) {
                require(a == c || a == d || b == c || b == d,
                        "surface trim collinear intersection unresolved");
                point = (a == c || a == d) ? a : b;
            } else {
                const auto t = (p * v - q * u) / determinant;
                const auto s = (p * y - q * x) / determinant;
                constexpr long double margin = 64 * std::numeric_limits<double>::epsilon();
                require(t >= -margin && t <= 1 + margin && s >= -margin && s <= 1 + margin,
                        "surface trim intersection outside source segments");
                point = {double(a[0] + t * x), double(a[1] + t * y), 0};
            }
        } else {
            require(!sources[2] && !sources[3], "surface trim combine source pattern");
            const double scale =
                std::max({1.0, std::abs(a[0]), std::abs(a[1]), std::abs(b[0]), std::abs(b[1])});
            require(std::hypot(a[0] - b[0], a[1] - b[1]) <=
                        64 * std::numeric_limits<double>::epsilon() * scale,
                    "surface trim coincident vertices disagree");
        }
        // GLU's generic callback coordinate uses float interpolation weights.
        // Reconstruct planar UV intersections from the actual edge endpoints.
        context.storage.push_back(point);
        *result = &context.storage.back();
    });
}
void edge(unsigned char, void *) noexcept {}
void error(unsigned code, void *data) noexcept {
    auto &context = *static_cast<TessContext *>(data);
    guarded(context, [&] {
        throw std::runtime_error("surface trim tessellation error " + std::to_string(code));
    });
}
TessContext tessellate(const std::vector<Loop> &contours, bool boundaries, unsigned rule,
                       const BsplineMeshOptions &options) {
    TessContext context;
    context.boundaries = boundaries;
    context.vertices_limit = options.max_vertices;
    context.output_limit = std::size_t(options.max_triangles) * 3;
    std::unique_ptr<GLUtesselator, decltype(&gluDeleteTess)> tess(gluNewTess(), gluDeleteTess);
    require(bool(tess), "surface trim tessellator allocation");
    gluTessCallback(tess.get(), GLU_TESS_BEGIN_DATA, reinterpret_cast<void (*)()>(begin));
    gluTessCallback(tess.get(), GLU_TESS_VERTEX_DATA, reinterpret_cast<void (*)()>(vertex));
    gluTessCallback(tess.get(), GLU_TESS_EDGE_FLAG_DATA, reinterpret_cast<void (*)()>(edge));
    gluTessCallback(tess.get(), GLU_TESS_COMBINE_DATA, reinterpret_cast<void (*)()>(combine));
    gluTessCallback(tess.get(), GLU_TESS_ERROR_DATA, reinterpret_cast<void (*)()>(error));
    gluTessProperty(tess.get(), GLU_TESS_WINDING_RULE, rule);
    gluTessProperty(tess.get(), GLU_TESS_BOUNDARY_ONLY, boundaries ? 1 : 0);
    gluTessProperty(tess.get(), GLU_TESS_TOLERANCE, 0);
    gluTessNormal(tess.get(), 0, 0, 1);
    // Validate input and budget before entering the C callback state machine.
    std::size_t count = 0;
    for (const auto &loop : contours)
        for (const auto &uv : loop) {
            require(++count <= options.max_vertices, "surface trim input vertex budget");
            require(std::isfinite(uv[0]) && std::isfinite(uv[1]) &&
                        std::abs(uv[0]) <= GLU_TESS_MAX_COORD &&
                        std::abs(uv[1]) <= GLU_TESS_MAX_COORD,
                    "surface trim coordinate outside tessellator range");
        }
    gluTessBeginPolygon(tess.get(), &context);
    for (const auto &loop : contours) {
        gluTessBeginContour(tess.get());
        const auto size =
            loop.size() > 1 && loop.front() == loop.back() ? loop.size() - 1 : loop.size();
        for (std::size_t i = 0; i < size; ++i) {
            context.storage.push_back({loop[i][0], loop[i][1], 0});
            gluTessVertex(tess.get(), context.storage.back().data(), &context.storage.back());
        }
        gluTessEndContour(tess.get());
    }
    gluTessEndPolygon(tess.get());
    if (context.failure)
        std::rethrow_exception(context.failure);
    require(boundaries || context.corners.size() % 3 == 0, "surface trim incomplete triangle");
    return context;
}

using Edge = std::array<std::uint32_t, 2>;
Edge edge_key(std::uint32_t a, std::uint32_t b) {
    return {std::min(a, b), std::max(a, b)};
}
double length(Point2 a, Point2 b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
long double area(Point2 a, Point2 b, Point2 c) {
    return (static_cast<long double>(b[0]) - a[0]) * (static_cast<long double>(c[1]) - a[1]) -
           (static_cast<long double>(b[1]) - a[1]) * (static_cast<long double>(c[0]) - a[0]);
}
void check_continuity(const BsplineDirection &direction) {
    const auto domain = direction.knot_domain();
    const auto &knots = direction.knots();
    for (std::size_t i = 0; i < knots.size();) {
        const auto end = std::upper_bound(knots.begin() + i, knots.end(), knots[i]);
        if (knots[i] > domain[0] && knots[i] < domain[1])
            require(std::size_t(end - (knots.begin() + i)) < direction.order(),
                    "surface mesh requires one-sided evaluation at discontinuous knots");
        i = std::size_t(end - knots.begin());
    }
}
} // namespace

BsplineSurfaceMesh BsplineSurface::mesh(const BsplineMeshOptions &options) const {
    require(std::isfinite(options.uv_tolerance) && options.uv_tolerance > 0 &&
                std::isfinite(options.max_uv_edge) && options.max_uv_edge > 0,
            "surface mesh tolerances must be finite and positive");
    require(options.max_trim_segments && options.max_vertices && options.max_triangles,
            "surface mesh budgets must be positive");
    BsplineSurfaceMesh result;
    result.report = {{"status", "incomplete"},
                     {"representation", "derived_trimmed_surface_mesh"},
                     {"parameter_domain", {{0, 1}, {0, 1}}},
                     {"max_uv_edge", options.max_uv_edge},
                     {"world_space_error_bound", nullptr},
                     {"periodic_trim_unwrapping", "not_performed"}};
    try {
        const auto region = trim(options.uv_tolerance, options.max_trim_segments);
        result.report["trim"] = region.report();
        require(region.report().at("status") == "complete", "surface trim conversion incomplete");
        require(region.loops().empty() || (!u().closed() && !v().closed()),
                "periodic trimmed surface requires seam unwrapping");
        if (!region.loops().empty())
            require(u().knot_domain() == std::array<double, 2>{0, 1} &&
                        v().knot_domain() == std::array<double, 2>{0, 1},
                    "trim UV mapping requires normalized surface knot domains");
        check_continuity(u());
        check_continuity(v());
        if (rational()) {
            const bool positive = weights().front() > 0;
            for (const auto weight : weights())
                require(weight != 0 && (weight > 0) == positive,
                        "surface denominator sign not established for mesh");
        }
        const Loop square{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        auto source = region.loops();
        if (source.empty() || outer_boundary_active())
            source.push_back(square);
        // First resolve parity, including overlapping/self-intersecting loops.
        // GLU boundary output has the filled region on its left. Its signed
        // winding is therefore 1 inside, including correct hole orientation.
        auto parity = tessellate(source, true, GLU_TESS_WINDING_ODD, options);
        parity.contours.push_back(square);
        // Winding >=2 is the intersection of that region and the UV square.
        const auto clipped =
            tessellate(parity.contours, false, GLU_TESS_WINDING_ABS_GEQ_TWO, options);
        std::map<Point2, std::uint32_t> vertices;
        auto insert = [&](Point2 uv) {
            constexpr double margin = 64 * std::numeric_limits<double>::epsilon();
            for (auto &x : uv) {
                require(std::isfinite(x) && x >= -margin && x <= 1 + margin,
                        "surface tessellation left parameter domain");
                x = std::max(0.0, std::min(1.0, x));
            }
            const auto found = vertices.find(uv);
            if (found != vertices.end())
                return found->second;
            require(result.parameters.size() < options.max_vertices, "surface mesh vertex budget");
            const auto index = std::uint32_t(result.parameters.size());
            vertices.emplace(uv, index);
            result.parameters.push_back(uv);
            return index;
        };
        for (std::size_t i = 0; i < clipped.corners.size(); i += 3) {
            Triangle face{insert(clipped.corners[i]), insert(clipped.corners[i + 1]),
                          insert(clipped.corners[i + 2])};
            const auto signed_area = area(result.parameters[face[0]], result.parameters[face[1]],
                                          result.parameters[face[2]]);
            if (signed_area == 0)
                continue;
            if (signed_area < 0)
                std::swap(face[1], face[2]);
            result.faces.push_back(face);
        }
        unsigned iterations = 0;
        for (;;) {
            std::map<Edge, std::uint32_t> split;
            for (const auto &face : result.faces)
                for (unsigned e = 0; e < 3; ++e) {
                    const auto key = edge_key(face[e], face[(e + 1) % 3]);
                    if (length(result.parameters[key[0]], result.parameters[key[1]]) >
                        options.max_uv_edge)
                        split.emplace(key, 0);
                }
            if (split.empty())
                break;
            require(iterations++ < 64, "surface mesh refinement depth");
            for (auto &entry : split) {
                const auto a = result.parameters[entry.first[0]],
                           b = result.parameters[entry.first[1]];
                entry.second = insert({a[0] + (b[0] - a[0]) / 2, a[1] + (b[1] - a[1]) / 2});
                require(entry.second != entry.first[0] && entry.second != entry.first[1],
                        "surface mesh refinement below coordinate precision");
            }
            std::vector<Triangle> next;
            auto emit = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
                require(next.size() < options.max_triangles, "surface mesh triangle budget");
                require(area(result.parameters[a], result.parameters[b], result.parameters[c]) > 0,
                        "surface mesh refinement lost triangle orientation");
                next.push_back({a, b, c});
            };
            for (const auto &f : result.faces) {
                std::array<std::optional<std::uint32_t>, 3> mid;
                unsigned count = 0;
                for (unsigned e = 0; e < 3; ++e) {
                    const auto at = split.find(edge_key(f[e], f[(e + 1) % 3]));
                    if (at != split.end()) {
                        mid[e] = at->second;
                        ++count;
                    }
                }
                if (count == 0)
                    emit(f[0], f[1], f[2]);
                else if (count == 3) {
                    emit(f[0], *mid[0], *mid[2]);
                    emit(*mid[0], f[1], *mid[1]);
                    emit(*mid[2], *mid[1], f[2]);
                    emit(*mid[0], *mid[1], *mid[2]);
                } else if (count == 1) {
                    unsigned e = 0;
                    while (!mid[e])
                        ++e;
                    emit(f[e], *mid[e], f[(e + 2) % 3]);
                    emit(*mid[e], f[(e + 1) % 3], f[(e + 2) % 3]);
                } else {
                    unsigned e = 0;
                    while (!mid[e] || !mid[(e + 1) % 3])
                        ++e;
                    const auto a = f[e], b = f[(e + 1) % 3], c = f[(e + 2) % 3];
                    const auto m = *mid[e], n = *mid[(e + 1) % 3];
                    emit(m, b, n);
                    emit(a, m, n);
                    emit(a, n, c);
                }
            }
            result.faces = std::move(next);
        }
        for (const auto &uv : result.parameters)
            result.vertices.push_back(point_at(uv[0], uv[1]));
        result.report["refinement_iterations"] = iterations;
        result.report["vertices"] = result.vertices.size();
        result.report["triangles"] = result.faces.size();
        result.report["status"] = "complete";
    } catch (const std::exception &e) {
        result.report["reason"] = e.what();
        result.vertices.clear();
        result.parameters.clear();
        result.faces.clear();
    }
    return result;
}
} // namespace p3d
