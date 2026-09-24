#include <p3d/solid.hpp>
#include "geometry.hpp"
#include "planar_rings.hpp"

namespace p3d {
namespace {
constexpr double tau = 6.283185307179586476925286766559;
using FaceId = std::array<std::int64_t, 3>;
using Ring = std::vector<std::uint32_t>;
Point3 minus(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        a[k] -= b[k];
    return a;
}
Point3 cross(Point3 a, Point3 b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(Point3 a, Point3 b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Point3 blend(Point3 a, Point3 b, double f) {
    for (unsigned k = 0; k < 3; ++k)
        a[k] = (1 - f) * a[k] + f * b[k];
    return a;
}
double number(const Json &j) {
    require(j.is_number(), "sweep coordinate is not numeric");
    const auto x = j.get<double>();
    require(std::isfinite(x), "sweep nonfinite coordinate");
    return x;
}
Point3 point(const Json &j, const std::string &prefix) {
    return {number(j.at(prefix + "X")), number(j.at(prefix + "Y")), number(j.at(prefix + "Z"))};
}
bool joins(Point3 a, Point3 b) {
    for (unsigned k = 0; k < 3; ++k)
        if (std::abs(a[k] - b[k]) > 16 * std::numeric_limits<double>::epsilon() *
                                        std::max({1., std::abs(a[k]), std::abs(b[k])}))
            return false;
    return true;
}
struct Primitive {
    std::string type;
    std::vector<Point3> points;
    Point3 center{}, x{}, y{};
    double start = 0, sweep = 0;
    std::size_t flat = 0;
    std::size_t pieces() const {
        return type == "EllipticArc" ? 1 : points.size() - 1;
    }
    Point3 at(std::size_t piece, double f) const {
        if (type != "EllipticArc")
            return blend(points.at(piece), points.at(piece + 1), f);
        const auto angle = start + sweep * f;
        Point3 p{};
        for (unsigned k = 0; k < 3; ++k)
            p[k] = center[k] + x[k] * std::cos(angle) + y[k] * std::sin(angle);
        return p;
    }
};
struct Loop {
    int type;
    std::vector<Primitive> primitives;
};
struct Profile {
    std::vector<Loop> loops;
    std::vector<std::vector<std::size_t>> regions;
};
struct Profiles {
    std::vector<Profile> profiles;
    std::size_t visited = 0, coordinates = 0;
    const PolyfaceMeshOptions &budget;
    Profile parse(const Json &j) {
        Profile out;
        std::size_t flat = 0;
        std::function<void(const Json &, unsigned, std::optional<std::size_t>)> walk;
        walk = [&](const Json &v, unsigned depth, std::optional<std::size_t> region) {
            require(depth <= 80 && visited++ < budget.max_corners,
                    "sweep profile traversal budget");
            require(v.at("_type") == "CurveVector", "sweep requires a curve array");
            const int type = v.at("type").get<int>();
            const auto &members = v.at("curves");
            require(members.is_array() && !members.empty(), "sweep empty curve array");
            if (type == 4 || type == 5) {
                require(!region, "nested sweep parity region not supported");
                if (type == 4) {
                    region = out.regions.size();
                    out.regions.emplace_back();
                }
                for (const auto &m : members)
                    walk(m.at("geometry"), depth + 1, region);
                return;
            }
            require(type >= 1 && type <= 3, "sweep boundary type not supported");
            require(!region || type == 2 || type == 3, "sweep parity child must be a closed loop");
            if (!region) {
                region = out.regions.size();
                out.regions.emplace_back();
            }
            out.regions[*region].push_back(out.loops.size());
            Loop loop{type, {}};
            for (const auto &m : members) {
                require(visited++ < budget.max_corners, "sweep primitive budget");
                const auto &g = m.at("geometry");
                Primitive p;
                p.type = g.at("_type").get<std::string>();
                p.flat = flat++;
                if (p.type == "LineString") {
                    const auto &a = g.at("points");
                    require(a.is_array() && a.size() % 3 == 0 && a.size() >= 6,
                            "sweep line-string coordinate count");
                    require(a.size() / 3 <= budget.max_points - coordinates,
                            "sweep source point budget");
                    coordinates += a.size() / 3;
                    for (std::size_t i = 0; i < a.size(); i += 3)
                        p.points.push_back({number(a[i]), number(a[i + 1]), number(a[i + 2])});
                } else if (p.type == "LineSegment") {
                    require(budget.max_points - coordinates >= 2, "sweep source point budget");
                    coordinates += 2;
                    p.points = {point(g.at("segment"), "point0"), point(g.at("segment"), "point1")};
                } else if (p.type == "EllipticArc") {
                    const auto &a = g.at("arc");
                    p.center = point(a, "center");
                    p.x = point(a, "vector0");
                    p.y = point(a, "vector90");
                    p.start = number(a.at("startRadians"));
                    p.sweep = number(a.at("sweepRadians"));
                    require(p.sweep != 0 && std::abs(p.sweep) <= tau,
                            "sweep zero or multi-turn profile arc not supported");
                } else
                    throw std::runtime_error("sweep primitive not supported: " + p.type);
                loop.primitives.push_back(std::move(p));
            }
            out.loops.push_back(std::move(loop));
        };
        walk(j, 0, {});
        return out;
    }
};
} // namespace

SolidMeshResult mesh_bgfb_sweep(const Json &table, const PolyfaceMeshOptions &options,
                                unsigned circle_segments) {
    SolidMeshResult out;
    out.source = table;
    try {
        require(circle_segments >= 3 && circle_segments <= options.max_points,
                "sweep angular segment budget");
        const bool extrusion = table.at("_type") == "DgnExtrusion";
        const bool capped = table.at("capped").get<bool>();
        Profiles reader{{}, 0, 0, options};
        if (extrusion) {
            reader.profiles.push_back(reader.parse(table.at("baseCurve")));
            auto top = reader.profiles.front();
            const auto &v = table.at("extrusionVector");
            Point3 delta{number(v.at("x")), number(v.at("y")), number(v.at("z"))};
            require(delta != Point3{}, "sweep zero extrusion vector");
            for (auto &loop : top.loops)
                for (auto &p : loop.primitives) {
                    for (auto &q : p.points)
                        for (unsigned k = 0; k < 3; ++k)
                            q[k] += delta[k];
                    for (unsigned k = 0; k < 3; ++k)
                        p.center[k] += delta[k];
                }
            reader.profiles.push_back(std::move(top));
        } else {
            require(table.at("_type") == "DgnRuledSweep", "sweep source type");
            const auto &sections = table.at("curves");
            require(sections.is_array() && sections.size() >= 2 &&
                        sections.size() <= options.max_points,
                    "sweep requires at least two ordered sections within budget");
            for (const auto &s : sections)
                reader.profiles.push_back(reader.parse(s));
        }
        const auto &profiles = reader.profiles;
        const auto &first = profiles.front();
        for (const auto &p : profiles) {
            require(p.regions == first.regions && p.loops.size() == first.loops.size(),
                    "sweep section region correspondence mismatch");
            for (std::size_t l = 0; l < first.loops.size(); ++l) {
                const auto &a = first.loops[l], &b = p.loops[l];
                require(a.type == b.type && a.primitives.size() == b.primitives.size(),
                        "sweep section loop correspondence mismatch");
                for (std::size_t i = 0; i < a.primitives.size(); ++i)
                    require(a.primitives[i].type == b.primitives[i].type &&
                                a.primitives[i].pieces() == b.primitives[i].pieces(),
                            "sweep corresponding primitive or component counts differ");
            }
        }
        std::vector<Point3> vertices;
        auto vertex = [&](Point3 p) {
            require(vertices.size() < std::min<std::size_t>(options.max_points, UINT32_MAX),
                    "sweep derived vertex budget");
            for (double x : p)
                require(std::isfinite(x), "sweep derived point overflow");
            vertices.push_back(p);
            return std::uint32_t(vertices.size() - 1);
        };
        struct Edge {
            std::size_t primitive, component;
        };
        std::vector<std::vector<Edge>> edges(first.loops.size());
        std::vector<std::vector<Ring>> rows(profiles.size(), std::vector<Ring>(first.loops.size()));
        for (std::size_t l = 0; l < first.loops.size(); ++l) {
            const auto &loop = first.loops[l];
            for (std::size_t i = 0; i < loop.primitives.size(); ++i) {
                const auto &p = loop.primitives[i];
                unsigned bands = 1;
                if (p.type == "EllipticArc")
                    for (const auto &profile : profiles)
                        bands = std::max(bands, unsigned(std::ceil(
                                                    std::abs(profile.loops[l].primitives[i].sweep) /
                                                    tau * circle_segments)));
                for (std::size_t c = 0; c < p.pieces(); ++c) {
                    require(bands <= options.max_corners - edges[l].size(),
                            "sweep sampled edge budget");
                    for (unsigned s = 0; s < bands; ++s)
                        edges[l].push_back({p.flat, c});
                    for (std::size_t row = 0; row < profiles.size(); ++row) {
                        const auto &q = profiles[row].loops[l].primitives[i];
                        auto &ring = rows[row][l];
                        const auto start = q.at(c, 0);
                        if (ring.empty())
                            ring.push_back(vertex(start));
                        else
                            require(joins(vertices[ring.back()], start),
                                    "sweep disconnected source primitives");
                        for (unsigned s = 1; s <= bands; ++s)
                            ring.push_back(vertex(q.at(c, double(s) / bands)));
                    }
                }
            }
            if (loop.type != 1)
                for (auto &row : rows) {
                    auto &ring = row[l];
                    require(ring.size() >= 4 &&
                                joins(vertices[ring.back()], vertices[ring.front()]),
                            "sweep closed boundary has an open or degenerate source chain");
                    // Only the specified consecutive seam shares a derived index.
                    ring.back() = ring.front();
                }
        }
        std::vector<Triangle> triangles;
        std::vector<FaceId> ids;
        auto emit = [&](Triangle t, FaceId id) {
            require(triangles.size() < options.max_triangles &&
                        triangles.size() < options.max_corners / 3,
                    "sweep derived triangle/corner budget");
            const auto n =
                cross(minus(vertices[t[1]], vertices[t[0]]), minus(vertices[t[2]], vertices[t[0]]));
            require(std::isfinite(dot(n, n)) && dot(n, n) > 0,
                    "sweep collapsed or overflowing triangle");
            triangles.push_back(t);
            ids.push_back(id);
        };
        std::vector<bool> reverse(first.loops.size(), false);
        // Closed regions need the same parity classification for side orientation,
        // even when their caps are omitted from the output.
        for (const auto &group : first.regions) {
            if (first.loops[group.front()].type == 1) {
                require(!capped, "capped open sweep profile is not supported");
                continue;
            }
            std::vector<Ring> bottom, top;
            for (auto l : group) {
                auto a = rows.front()[l], b = rows.back()[l];
                a.pop_back();
                b.pop_back();
                bottom.push_back(std::move(a));
                top.push_back(std::move(b));
            }
            const auto budget = unsigned(
                std::min<std::size_t>(options.max_triangles - triangles.size(), UINT32_MAX));
            auto a = triangulate_planar_sample_rings(bottom, vertices, budget);
            auto b = triangulate_planar_sample_rings(top, vertices, budget);
            require(!a.empty() && !b.empty(), "sweep empty caps");
            auto normal = [&](Triangle t) {
                return cross(minus(vertices[t[1]], vertices[t[0]]),
                             minus(vertices[t[2]], vertices[t[0]]));
            };
            // Samples alone cannot prove that the full analytic arc is planar.
            // Check its center and both complete axis vectors against the cap.
            auto analytic_plane = [&](const Profile &profile, Triangle t) {
                auto n = normal(t);
                const double length = std::sqrt(dot(n, n));
                require(length > 0 && std::isfinite(length), "sweep cap normal");
                for (auto &x : n)
                    x /= length;
                const auto origin = vertices[t[0]];
                for (auto l : group)
                    for (const auto &p : profile.loops[l].primitives)
                        if (p.type == "EllipticArc") {
                            const auto bound = std::abs(dot(minus(p.center, origin), n)) +
                                               std::hypot(dot(p.x, n), dot(p.y, n));
                            require(std::isfinite(bound) && bound <= 1e-8,
                                    "sweep analytic cap arc is not planar");
                        }
            };
            analytic_plane(profiles.front(), a.front());
            analytic_plane(profiles.back(), b.front());
            const auto direction =
                minus(vertices[rows[1][group.front()][0]], vertices[rows[0][group.front()][0]]);
            const auto orientation = dot(normal(a.front()), direction);
            require(std::isfinite(orientation) && orientation != 0,
                    "sweep initial section has no transverse direction");
            if (orientation < 0)
                for (auto &t : a)
                    std::swap(t[1], t[2]);
            // Orient sides from actual parity cap perimeter, not the source winding.
            std::map<std::pair<std::uint32_t, std::uint32_t>, unsigned> boundary;
            for (auto t : a)
                for (unsigned k = 0; k < 3; ++k)
                    ++boundary[{t[k], t[(k + 1) % 3]}];
            for (auto l : group) {
                const auto &r = rows.front()[l];
                const auto forward = boundary.count({r[0], r[1]}),
                           backward = boundary.count({r[1], r[0]});
                require(forward != backward, "sweep side perimeter orientation unavailable");
                reverse[l] = !forward;
            }
            // Source end sections must keep the same boundary orientation.
            std::map<std::pair<std::uint32_t, std::uint32_t>, unsigned> end_boundary;
            for (auto t : b)
                for (unsigned k = 0; k < 3; ++k)
                    ++end_boundary[{t[k], t[(k + 1) % 3]}];
            const auto &r = rows.back()[group.front()];
            if (bool(end_boundary.count({r[0], r[1]})) == reverse[group.front()])
                for (auto &t : b)
                    std::swap(t[1], t[2]);
            end_boundary.clear();
            for (auto t : b)
                for (unsigned k = 0; k < 3; ++k)
                    ++end_boundary[{t[k], t[(k + 1) % 3]}];
            for (auto l : group) {
                const auto &ring = rows.back()[l];
                require(bool(end_boundary.count({ring[0], ring[1]})) != reverse[l],
                        "sweep end-region boundary correspondence reverses orientation");
            }
            if (capped) {
                for (auto t : a) {
                    std::swap(t[1], t[2]);
                    emit(t, {-1, 0, 0});
                }
                for (auto t : b)
                    emit(t, {-1, 1, 0});
            }
        }
        std::size_t derived_bands = 0;
        for (std::size_t row = 0; row + 1 < profiles.size(); ++row) {
            unsigned bands = 1;
            for (std::size_t l = 0; l < first.loops.size(); ++l)
                for (std::size_t e = 0; e < edges[l].size(); ++e) {
                    const auto a = vertices[rows[row][l][e]], b = vertices[rows[row][l][e + 1]],
                               c = vertices[rows[row + 1][l][e]],
                               d = vertices[rows[row + 1][l][e + 1]];
                    const auto n = cross(minus(b, a), minus(c, a)), edge = minus(d, a);
                    const auto value = dot(n, edge);
                    const auto scale = std::sqrt(dot(n, n)) * std::sqrt(dot(edge, edge));
                    require(std::isfinite(value) && std::isfinite(scale),
                            "sweep patch extent overflow");
                    if (std::abs(value) > 64 * std::numeric_limits<double>::epsilon() * scale)
                        bands = circle_segments;
                }
            derived_bands += bands;
            auto previous = rows[row];
            for (unsigned band = 1; band <= bands; ++band) {
                auto next = rows[row + 1];
                if (band < bands)
                    for (std::size_t l = 0; l < next.size(); ++l) {
                        auto &ring = next[l];
                        for (std::size_t e = 0; e < ring.size(); ++e) {
                            if (e + 1 == ring.size() && first.loops[l].type != 1)
                                ring[e] = ring.front();
                            else
                                ring[e] = vertex(blend(vertices[rows[row][l][e]],
                                                       vertices[rows[row + 1][l][e]],
                                                       double(band) / bands));
                        }
                    }
                for (std::size_t l = 0; l < first.loops.size(); ++l)
                    for (std::size_t e = 0; e < edges[l].size(); ++e) {
                        const auto a = previous[l][e], b = previous[l][e + 1], c = next[l][e],
                                   d = next[l][e + 1];
                        const auto source = edges[l][e];
                        FaceId id{std::int64_t(row), std::int64_t(source.primitive),
                                  std::int64_t(source.component)};
                        Triangle x{a, b, d}, y{a, d, c};
                        if (reverse[l]) {
                            std::swap(x[1], x[2]);
                            std::swap(y[1], y[2]);
                        }
                        emit(x, id);
                        emit(y, id);
                    }
                previous = std::move(next);
            }
        }
        Json points = Json::array(), indices = Json::array();
        for (auto p : vertices)
            for (double x : p)
                points.push_back(x);
        for (auto t : triangles) {
            for (auto x : t)
                indices.push_back(std::int64_t(x) + 1);
            indices.push_back(0);
        }
        out.derived = mesh_bgfb_polyface({{"_type", "Polyface"},
                                          {"meshStyle", 1},
                                          {"numPerFace", 0},
                                          {"point", std::move(points)},
                                          {"pointIndex", std::move(indices)}},
                                         options);
        out.derived.report["source_kind"] = table.at("_type");
        out.derived.report["scope"] = "derived_corresponding_curve_sweep";
        out.derived.report["native_tessellation_equivalence"] = "not_established";
        out.derived.report["world_space_error_bound"] = nullptr;
        out.derived.report["source_section_count"] = profiles.size();
        out.derived.report["source_loop_count"] = first.loops.size();
        out.derived.report["derived_ruled_bands"] = derived_bands;
        out.derived.report["native_uv_and_normals"] = "not_evaluated";
        if (out.derived.status == "meshed")
            out.face_indices = std::move(ids);
    } catch (const std::exception &e) {
        out.derived = {};
        out.derived.report["reason"] = e.what();
        out.face_indices.clear();
    }
    return out;
}
} // namespace p3d
