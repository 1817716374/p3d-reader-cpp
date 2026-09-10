#include "geometry.hpp"
#include "guided.hpp"
#include <mapbox/earcut.hpp>
namespace p3d {
static constexpr double pi = 3.1415926535897932384626433832795;
static Point3 add(Point3 a, Point3 b) {
    for (unsigned i = 0; i < 3; ++i)
        a[i] += b[i];
    return a;
}
static Point3 sub(Point3 a, Point3 b) {
    for (unsigned i = 0; i < 3; ++i)
        a[i] -= b[i];
    return a;
}
static Point3 scale(Point3 a, double s) {
    for (auto &x : a)
        x *= s;
    return a;
}
static Point3 cross(Point3 a, Point3 b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
static double dot(Point3 a, Point3 b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static double norm(Point3 a) {
    return std::sqrt(dot(a, a));
}
Matrix4 identity() {
    return {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
}
Matrix4 multiply(const Matrix4 &a, const Matrix4 &b) {
    Matrix4 m{};
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned j = 0; j < 4; ++j)
            for (unsigned k = 0; k < 4; ++k)
                m[i][j] += a[i][k] * b[k][j];
    return m;
}
Point3 vector_transform(const Matrix4 &m, const Point3 &p) {
    Point3 v{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            v[i] += m[i][j] * p[j];
    return v;
}
Point3 transform(const Matrix4 &m, const Point3 &p) {
    auto v = vector_transform(m, p);
    for (unsigned i = 0; i < 3; ++i)
        v[i] += m[i][3];
    return v;
}
double determinant(const Matrix4 &m) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
static void affine(const Matrix4 &m) {
    for (auto row : m)
        for (double x : row)
            require(std::isfinite(x), "nonfinite affine transform");
    require(m[3] == std::array<double, 4>{0, 0, 0, 1}, "nonaffine transform");
}
bool reverses_winding(const Matrix4 &m) {
    // Positive row scaling preserves orientation and avoids overflow/underflow
    // of the determinant for uniformly large/small instance scales.
    long double a[3][3];
    for (unsigned i = 0; i < 3; ++i) {
        long double bound = 0;
        for (unsigned j = 0; j < 3; ++j) {
            if (!std::isfinite(m[i][j]))
                return false;
            bound = std::max(bound, std::abs(static_cast<long double>(m[i][j])));
        }
        if (bound == 0)
            return false;
        for (unsigned j = 0; j < 3; ++j)
            a[i][j] = m[i][j] / bound;
    }
    return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
               a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
               a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]) <
           0;
}
// Solve the inverse once per placement. Long-double elimination avoids a
// determinant threshold that would reject small but invertible model scales.
static void transform_normals(Geometry &g, const Matrix4 &m, std::size_t start = 0) {
    if (start == g.normals.size())
        return;
    long double a[3][6]{};
    bool valid = true;
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j)
            a[i][j] = m[i][j];
        a[i][i + 3] = 1;
    }
    for (unsigned col = 0; col < 3 && valid; ++col) {
        unsigned pivot = col;
        for (unsigned row = col + 1; row < 3; ++row)
            if (std::abs(a[row][col]) > std::abs(a[pivot][col]))
                pivot = row;
        if (a[pivot][col] == 0) {
            valid = false;
            break;
        }
        for (unsigned j = 0; j < 6; ++j)
            std::swap(a[col][j], a[pivot][j]);
        const auto divisor = a[col][col];
        for (auto &v : a[col])
            v /= divisor;
        for (unsigned row = 0; row < 3; ++row) {
            if (row == col)
                continue;
            const auto factor = a[row][col];
            for (unsigned j = 0; j < 6; ++j)
                a[row][j] -= factor * a[col][j];
        }
    }
    for (const auto &row : a)
        for (auto value : row)
            valid &= std::isfinite(value);
    for (auto it = g.normals.begin() + start; it != g.normals.end(); ++it) {
        auto &normal = *it;
        if (!normal)
            continue;
        Point3 result{};
        bool finite = valid;
        for (unsigned i = 0; i < 3 && finite; ++i) {
            long double value = 0;
            for (unsigned j = 0; j < 3; ++j)
                value += a[j][i + 3] * (*normal)[j];
            result[i] = static_cast<double>(value);
            finite &= std::isfinite(result[i]);
        }
        normal = finite ? std::optional<Point3>(result) : std::nullopt;
    }
}
unsigned Tessellation::segments(double radius, double sweep) const {
    require(std::isfinite(radius) && std::isfinite(sweep), "nonfinite curve extent");
    radius = std::abs(radius);
    sweep = std::abs(sweep);
    double n;
    if (chord_tolerance) {
        require(std::isfinite(*chord_tolerance) && *chord_tolerance > 0,
                "chord tolerance must be finite and positive");
        auto angle = radius
                         ? 4 * std::asin(std::sqrt(std::min(1., *chord_tolerance / (2 * radius))))
                         : 2 * pi;
        require(angle > 0, "curve tolerance exceeds numerical range");
        n = std::max(
            {1., std::ceil(sweep / angle), std::ceil(min_full_circle_segments * sweep / (2 * pi))});
    } else {
        require(full_circle_segments >= 1, "circle segment count");
        n = std::max(1., std::ceil(full_circle_segments * sweep / (2 * pi)));
    }
    require(n <= max_segments, "curve tolerance exceeds caller segment budget");
    return unsigned(n);
}
static Matrix4 quaternion(const Json &q) {
    require(q.is_array() && q.size() == 4, "quaternion count");
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    auto m = identity();
    if (n < 1e-12)
        return m;
    w /= n;
    x /= n;
    y /= n;
    z /= n;
    m[0] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0};
    m[1] = {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0};
    m[2] = {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0};
    return m;
}
Matrix4 instance_transform(const Bytes &b) {
    require(b.size() == 28 || b.size() == 60 || b.size() == 100, "instance transform size");
    Reader r(b, 4);
    auto m = identity();
    if (b.size() == 100) {
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 4; ++j)
                m[i][j] = r.f64();
    } else {
        auto p = r.doubles(3);
        if (b.size() == 60)
            m = quaternion(r.doubles(4));
        for (unsigned i = 0; i < 3; ++i)
            m[i][3] = p[i];
    }
    affine(m);
    return m;
}
static double scale_bound(const Matrix4 &m) {
    double a[3][3]{};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            for (unsigned k = 0; k < 3; ++k)
                a[i][j] += m[k][i] * m[k][j];
    for (int it = 0; it < 32; ++it) {
        unsigned p = 0, q = 1;
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = i + 1; j < 3; ++j)
                if (std::abs(a[i][j]) > std::abs(a[p][q])) {
                    p = i;
                    q = j;
                }
        if (std::abs(a[p][q]) <
            1e-16 * std::max({1., std::abs(a[0][0]), std::abs(a[1][1]), std::abs(a[2][2])}))
            break;
        double angle = .5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]), c = std::cos(angle),
               s = std::sin(angle);
        double pp = c * c * a[p][p] - 2 * s * c * a[p][q] + s * s * a[q][q],
               qq = s * s * a[p][p] + 2 * s * c * a[p][q] + c * c * a[q][q];
        for (unsigned k = 0; k < 3; ++k)
            if (k != p && k != q) {
                double kp = c * a[k][p] - s * a[k][q], kq = s * a[k][p] + c * a[k][q];
                a[k][p] = a[p][k] = kp;
                a[k][q] = a[q][k] = kq;
            }
        a[p][p] = pp;
        a[q][q] = qq;
        a[p][q] = a[q][p] = 0;
    }
    return std::sqrt(std::max({0., a[0][0], a[1][1], a[2][2]}));
}
static std::vector<Triangle> polygon_faces(const std::vector<Point3> &points,
                                           const std::vector<std::size_t> &lengths = {}) {
    if (points.size() < 3)
        return {};
    std::vector<Point3> centered;
    Point3 normal{};
    for (auto p : points)
        centered.push_back(sub(p, points[0]));
    for (std::size_t i = 0; i < centered.size(); ++i)
        normal = add(normal, cross(centered[i], centered[(i + 1) % centered.size()]));
    if (norm(normal) < 1e-10)
        return {};
    unsigned axis = 0;
    for (unsigned i = 1; i < 3; ++i)
        if (std::abs(normal[i]) > std::abs(normal[axis]))
            axis = i;
    std::vector<std::vector<Point2>> rings;
    std::size_t off = 0;
    auto counts = lengths.empty() ? std::vector<std::size_t>{points.size()} : lengths;
    for (auto n : counts) {
        require(n <= points.size() - off, "polygon ring lengths");
        std::vector<Point2> ring;
        for (std::size_t i = off; i < off + n; ++i) {
            Point2 uv{};
            unsigned j = 0;
            for (unsigned k = 0; k < 3; ++k)
                if (k != axis)
                    uv[j++] = centered[i][k];
            ring.push_back(uv);
        }
        rings.push_back(ring);
        off += n;
    }
    require(off == points.size(), "polygon ring coverage");
    auto ids = mapbox::earcut<std::uint32_t>(rings);
    std::vector<Triangle> faces;
    for (std::size_t i = 0; i < ids.size(); i += 3)
        faces.push_back({ids[i], ids[i + 1], ids[i + 2]});
    if (!faces.empty()) {
        auto f = faces[0];
        if (dot(cross(sub(points[f[1]], points[f[0]]), sub(points[f[2]], points[f[0]])), normal) <
            0)
            for (auto &f : faces)
                std::reverse(f.begin(), f.end());
    }
    return faces;
}
static void append(Geometry &g, const std::vector<Point3> &p, const std::vector<Triangle> &f,
                   const std::vector<std::optional<std::array<Point2, 3>>> *uv = nullptr,
                   const std::vector<std::optional<std::uint32_t>> *src = nullptr) {
    auto off = g.vertices.size();
    require(off + p.size() <= UINT32_MAX, "geometry vertex index capacity");
    g.vertices.insert(g.vertices.end(), p.begin(), p.end());
    for (std::size_t i = 0; i < f.size(); ++i) {
        auto tri = f[i];
        for (auto &x : tri) {
            require(x < p.size(), "geometry face index");
            x += std::uint32_t(off);
        }
        g.faces.push_back(tri);
        g.face_uvs.push_back(uv ? uv->at(i) : std::nullopt);
        g.face_source_polygons.push_back(src ? src->at(i) : std::nullopt);
        g.face_normal_indices.push_back(std::nullopt);
        g.face_uv_indices.push_back(std::nullopt);
    }
}
static void loft(Geometry &g, const std::vector<std::vector<Point3>> &sections, bool capped,
                 const std::vector<std::size_t> &ring_lengths = {}) {
    if (sections.size() < 2)
        return;
    std::set<std::size_t> counts;
    for (auto &s : sections)
        counts.insert(s.size());
    if (counts.size() != 1) {
        g.unknown.push_back({{"reason", "unequal loft section point counts"}, {"counts", counts}});
        return;
    }
    auto n = sections[0].size();
    require(sections.size() * n <= UINT32_MAX, "loft vertex capacity");
    std::vector<Point3> points;
    for (auto &s : sections)
        points.insert(points.end(), s.begin(), s.end());
    std::vector<Triangle> faces;
    auto lengths = ring_lengths.empty() ? std::vector<std::size_t>{n} : ring_lengths;
    std::size_t sum = 0;
    for (auto c : lengths)
        sum += c;
    require(sum == n, "loft ring coverage");
    for (std::size_t j = 0; j + 1 < sections.size(); ++j) {
        std::size_t start = 0;
        for (auto count : lengths) {
            for (std::size_t i = start; i + 1 < start + count; ++i) {
                auto a = std::uint32_t(j * n + i), b = a + 1, c = std::uint32_t(a + n), d = c + 1;
                faces.push_back({a, b, d});
                faces.push_back({a, d, c});
            }
            start += count;
        }
    }
    if (capped) {
        for (auto f : polygon_faces(sections.front(), ring_lengths)) {
            std::reverse(f.begin(), f.end());
            faces.push_back(f);
        }
        for (auto f : polygon_faces(sections.back(), ring_lengths)) {
            for (auto &i : f)
                i += std::uint32_t((sections.size() - 1) * n);
            faces.push_back(f);
        }
    }
    append(g, points, faces);
}
static std::vector<Point3> curve(unsigned op, const Json &decoded, const Tessellation &policy,
                                 double bound = 1) {
    if (op == 1 || op == 9)
        return decoded.at("points").get<std::vector<Point3>>();
    require(op == 5 || op == 7, "curve opcode");
    auto origin = decoded.at("origin").get<Point3>();
    auto q = quaternion(decoded.at("quaternion"));
    Matrix4 inv = identity();
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            inv[i][j] = q[j][i];
    double rx = decoded["radii"][0], ry = decoded["radii"][1],
           start = decoded.value("start_angle", 0.), sweep = decoded.value("sweep_angle", 2 * pi);
    auto n = policy.segments(std::max(std::abs(rx), std::abs(ry)) * bound, sweep);
    std::vector<Point3> p;
    for (unsigned i = 0; i <= n; ++i) {
        double t = i == n ? start + sweep : start + (sweep / n) * i;
        p.push_back(add(vector_transform(inv, {rx * std::cos(t), ry * std::sin(t), 0}), origin));
    }
    return p;
}
static std::vector<Point3> tangents(unsigned op, const Json &d, const std::vector<Point3> &p) {
    std::vector<Point3> t;
    if (op == 5 || op == 7) {
        double start = d.value("start_angle", 0.), sweep = d.value("sweep_angle", 2 * pi),
               rx = d["radii"][0], ry = d["radii"][1];
        auto q = quaternion(d["quaternion"]);
        auto inv = identity();
        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                inv[i][j] = q[j][i];
        for (std::size_t i = 0; i < p.size(); ++i) {
            double a = i + 1 == p.size() ? start + sweep : start + (sweep / (p.size() - 1)) * i;
            t.push_back(scale(vector_transform(inv, {-rx * std::sin(a), ry * std::cos(a), 0}),
                              sweep > 0   ? 1
                              : sweep < 0 ? -1
                                          : 0));
        }
    } else {
        require(p.size() >= 2, "curve tangent point count");
        for (std::size_t i = 0; i < p.size(); ++i)
            t.push_back(sub(p[std::min(i + 1, p.size() - 1)], p[i ? i - 1 : 0]));
    }
    return t;
}
Geometry reconstruct(const Json &commands, const Tessellation &policy) {
    Geometry g;
    Json style = Json::object();
    unsigned solid = 0, stage = 0, path_kind = 0;
    Bytes payload;
    bool in_path = false;
    std::vector<GuidedBoundary> path_native, guide_native;
    std::vector<std::vector<GuidedBoundary>> section_native;
    std::vector<bool> section_closed;
    std::vector<std::vector<std::size_t>> section_native_lengths;
    std::vector<std::size_t> native_ring_ends, guide_counts;
    Matrix4 matrix = identity();
    std::vector<Matrix4> stack;
    std::vector<std::vector<Point3>> sections, guides, path_parts, section_tangents;
    std::vector<std::vector<std::vector<Point3>>> section_parts;
    std::vector<std::vector<std::size_t>> section_rings;
    std::vector<Point3> path, path_tangents;
    std::vector<std::size_t> ring_ends;
    auto world = [&](const std::vector<Point3> &p) {
        std::vector<Point3> out;
        for (auto v : p)
            out.push_back(transform(matrix, v));
        return out;
    };
    for (auto &cmd : commands) {
        unsigned op = cmd["op"];
        auto body = bytesof(cmd["body"]);
        auto &d = cmd["decoded"];
        Json mesh_metadata;
        auto first_face = g.faces.size(), first_line = g.lines.size(), first_text = g.texts.size();
        try {
            auto execute = [&]() {
                if (op == 28) {
                    auto st = decode_symbology(body);
                    apply_symbology(style, st);
                    return;
                }
                if (op == 40) {
                    apply_symbology_extension(style, d);
                    return;
                }
                if (d.contains("field_decode_error"))
                    throw std::runtime_error(d["field_decode_error"].get<std::string>());
                if (op == 29)
                    return;
                if (op == 37) {
                    require(d.at("version") == 1, "text version");
                    auto origin = d.at("origin").get<Point3>();
                    g.texts.push_back({{"text", d["text"]},
                                       {"origin", transform(matrix, origin)},
                                       {"source_origin", origin},
                                       {"placement_matrix", matrix},
                                       {"quaternion", d["quaternion"]},
                                       {"width", d["width"]},
                                       {"height", d["height"]},
                                       {"style_words", d["style_words"]},
                                       {"extra_style_hex", d["extra_style_hex"]}});
                    return;
                }
                if (op == 1 || op == 3 || op == 5 || op == 7 || op == 9) {
                    if (op == 3) {
                        g.lines.push_back(world(d["points"].get<std::vector<Point3>>()));
                        return;
                    }
                    auto local =
                             curve(op, d, policy, policy.chord_tolerance ? scale_bound(matrix) : 1),
                         points = world(local);
                    if (op == 9 && !solid)
                        append(g, points, polygon_faces(points));
                    else {
                        auto tan = tangents(op, d, local);
                        for (auto &t : tan)
                            t = vector_transform(matrix, t);
                        if (in_path) {
                            if (solid == 56) {
                                GuidedBoundary boundary;
                                if (op == 5 || op == 7) {
                                    boundary.ellipse = true;
                                    boundary.center = transform(matrix, d["origin"].get<Point3>());
                                    auto q = quaternion(d["quaternion"]);
                                    double rx = d["radii"][0], ry = d["radii"][1];
                                    boundary.axis_x = vector_transform(
                                        matrix, {q[0][0] * rx, q[0][1] * rx, q[0][2] * rx});
                                    boundary.axis_y = vector_transform(
                                        matrix, {q[1][0] * ry, q[1][1] * ry, q[1][2] * ry});
                                    boundary.start = d.value("start_angle", 0.);
                                    boundary.sweep = d.value("sweep_angle", 2 * pi);
                                } else
                                    boundary.points = points;
                                path_native.push_back(std::move(boundary));
                            }
                            path_parts.push_back(points);
                            std::size_t start =
                                !path.empty() && !points.empty() &&
                                        norm(sub(path.back(), points.front())) < 1e-6
                                    ? 1
                                    : 0;
                            path.insert(path.end(), points.begin() + start, points.end());
                            path_tangents.insert(path_tangents.end(), tan.begin() + start,
                                                 tan.end());
                        } else if (solid)
                            sections.push_back(points);
                        else
                            g.lines.push_back(points);
                    }
                    return;
                }
                if (op == 25) {
                    auto points = d["points"].get<std::vector<Point3>>();
                    Geometry channels;
                    channels.source_normals = d["normals"].get<std::vector<Point3>>();
                    for (auto normal : channels.source_normals)
                        channels.normals.push_back(normal);
                    channels.uvs = d["uvs"].get<std::vector<Point2>>();
                    transform_normals(channels, matrix);
                    std::vector<Triangle> faces;
                    std::vector<Triangle> source_corners;
                    const bool extended =
                        d.contains("mesh_channels") && has_mesh_channels(d["mesh_channels"]);
                    std::vector<std::optional<std::array<Point2, 3>>> uvs;
                    std::vector<std::optional<std::uint32_t>> sources;
                    for (std::size_t i = 0; i < d["polygons"].size(); ++i) {
                        auto &poly = d["polygons"][i];
                        std::vector<unsigned> ids;
                        std::vector<Point3> pts;
                        for (auto &ix : poly["point_indices"]) {
                            auto index = unsigned(std::llabs(ix.get<std::int64_t>()) - 1);
                            ids.push_back(index);
                            pts.push_back(points.at(index));
                        }
                        if (ids.empty())
                            continue;
                        auto tris =
                            ids.size() == 3 ? std::vector<Triangle>{{0, 1, 2}} : polygon_faces(pts);
                        for (auto tri : tris) {
                            faces.push_back({ids[tri[0]], ids[tri[1]], ids[tri[2]]});
                            if (extended)
                                source_corners.push_back(tri);
                            sources.push_back(unsigned(i));
                            for (const auto &channel :
                                 {std::pair<const char *, std::vector<std::optional<Triangle>> *>(
                                      "normal_indices", &channels.face_normal_indices),
                                  {"uv_indices", &channels.face_uv_indices}}) {
                                if (poly[channel.first].empty())
                                    channel.second->push_back(std::nullopt);
                                else {
                                    Triangle indices;
                                    for (unsigned j = 0; j < 3; ++j)
                                        indices[j] = static_cast<std::uint32_t>(
                                            std::llabs(
                                                poly[channel.first][tri[j]].get<std::int64_t>()) -
                                            1);
                                    channel.second->push_back(indices);
                                }
                            }
                            if (poly["uv_indices"].empty())
                                uvs.push_back(std::nullopt);
                            else {
                                std::array<Point2, 3> uv;
                                for (unsigned j = 0; j < 3; ++j)
                                    uv[j] = d["uvs"]
                                                .at(std::llabs(poly["uv_indices"][tri[j]]
                                                                   .get<std::int64_t>()) -
                                                    1)
                                                .get<Point2>();
                                uvs.push_back(uv);
                            }
                        }
                    }
                    if (extended)
                        mesh_metadata =
                            mesh_triangle_channels(d["mesh_channels"], sources, source_corners);
                    append(g, world(points), faces, &uvs, &sources);
                    channels.faces = std::move(faces);
                    merge_mesh_channels(g, channels, first_face);
                    if (d["trailing_channels_status"] == "opaque")
                        g.unknown.push_back({{"opcode", 25},
                                             {"offset", cmd["offset"]},
                                             {"reason", "uninterpreted mesh channels after UVs"}});
                    if (!mesh_metadata.is_null())
                        for (const auto *name : {"color_status", "face_uv_status",
                                                 "material_id_status", "smoothing_group_status"}) {
                            const auto &status = mesh_metadata["source"]["bindings"][name];
                            if (status == "count_mismatch" || status == "invalid_reader_indices" ||
                                status == "not_evaluated_for_fixed_width")
                                g.unknown.push_back(
                                    {{"opcode", 25},
                                     {"offset", cmd["offset"]},
                                     {"reason", "mesh extension binding unavailable"},
                                     {"channel", name},
                                     {"status", status}});
                        }
                    return;
                }
                if (op == 11 || op == 30) {
                    auto u = d["axis_u"].get<Point3>(), v = d["axis_v"].get<Point3>(),
                         c0 = d["origin0"].get<Point3>(), c1 = d["origin1"].get<Point3>();
                    std::vector<std::vector<Point3>> rings;
                    if (op == 11) {
                        double r0 = d["radius0"], r1 = d["radius1"];
                        auto m = identity();
                        for (unsigned i = 0; i < 3; ++i) {
                            m[i][0] = u[i];
                            m[i][1] = v[i];
                            m[i][2] = 0;
                        }
                        auto n = policy.segments(
                            std::max(std::abs(r0), std::abs(r1)) *
                            (policy.chord_tolerance ? scale_bound(multiply(matrix, m)) : 1));
                        std::vector<Point3> a, b;
                        for (unsigned i = 0; i <= n; ++i) {
                            double t = i == n ? 2 * pi : (2 * pi / n) * i;
                            auto ring = add(scale(u, std::cos(t)), scale(v, std::sin(t)));
                            a.push_back(add(c0, scale(ring, r0)));
                            b.push_back(add(c1, scale(ring, r1)));
                        }
                        rings = {world(a), world(b)};
                    } else {
                        for (unsigned j = 0; j < 2; ++j) {
                            auto c = j ? c1 : c0;
                            double x = d[j ? "x1" : "x0"], y = d[j ? "y1" : "y0"];
                            std::vector<Point3> ring;
                            for (auto signs :
                                 std::vector<Point2>{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}})
                                ring.push_back(add(add(c, scale(u, signs[0] * x / 2)),
                                                   scale(v, signs[1] * y / 2)));
                            rings.push_back(world(ring));
                        }
                    }
                    loft(g, rings, d["capped"].get<bool>());
                    return;
                }
                if (op == 16 || op == 17 || op == 52 || op == 56) {
                    require(!solid, "nested solid block");
                    solid = op;
                    payload = body;
                    sections.clear();
                    section_parts.clear();
                    section_rings.clear();
                    section_tangents.clear();
                    stage = 0;
                    guides.clear();
                    section_native.clear();
                    section_closed.clear();
                    section_native_lengths.clear();
                    guide_native.clear();
                    guide_counts.clear();
                    return;
                }
                if (op == 50 || op == 51 || op == 53 || op == 54 || op == 55) {
                    if (op == 55)
                        guide_counts = d.at("guide_counts").get<std::vector<std::size_t>>();
                    stage = op;
                    return;
                }
                if (op == 20 || op == 21) {
                    require(!in_path, "nested curve block");
                    in_path = true;
                    path.clear();
                    path_native.clear();
                    path_parts.clear();
                    ring_ends.clear();
                    native_ring_ends.clear();
                    path_tangents.clear();
                    path_kind = op;
                    return;
                }
                if (op == 23) {
                    require(in_path, "inner ring outside region");
                    ring_ends.push_back(path.size());
                    native_ring_ends.push_back(path_native.size());
                    return;
                }
                if (op == 22) {
                    require(in_path, "curve block underflow");
                    in_path = false;
                    std::vector<std::size_t> lengths;
                    std::size_t prev = 0;
                    for (auto end : ring_ends) {
                        lengths.push_back(end - prev);
                        prev = end;
                    }
                    lengths.push_back(path.size() - prev);
                    if (solid) {
                        if (stage != 55) {
                            sections.push_back(path);
                            section_parts.push_back(path_parts);
                            section_rings.push_back(lengths);
                            section_tangents.push_back(path_tangents);
                            section_native.push_back(path_native);
                            section_closed.push_back(path_kind == 21);
                            auto ends = native_ring_ends;
                            ends.push_back(path_native.size());
                            std::vector<std::size_t> native_lengths;
                            std::size_t start = 0;
                            for (auto end : ends) {
                                native_lengths.push_back(end - start);
                                start = end;
                            }
                            section_native_lengths.push_back(std::move(native_lengths));
                        } else {
                            guides.push_back(path);
                            if (path_native.size() == 1)
                                guide_native.push_back(path_native.front());
                            else {
                                GuidedBoundary boundary;
                                boundary.parts = path_native;
                                guide_native.push_back(std::move(boundary));
                            }
                        }
                    } else if (path_kind == 20)
                        g.lines.push_back(path);
                    else if (path.size() > 2)
                        append(g, path, polygon_faces(path, lengths));
                    else if (path.size() > 1)
                        g.lines.push_back(path);
                    return;
                }
                if (op == 19) {
                    require(solid, "solid block underflow");
                    if (solid == 17) {
                        Reader r(payload);
                        auto vector = vector_transform(matrix, r.doubles(3).get<Point3>());
                        bool capped = r.u8();
                        for (std::size_t i = 0; i < sections.size(); ++i) {
                            auto shifted = sections[i];
                            for (auto &p : shifted)
                                p = add(p, vector);
                            loft(g, {sections[i], shifted}, capped,
                                 i < section_rings.size() ? section_rings[i]
                                                          : std::vector<std::size_t>{});
                        }
                        if (sections.size() > 1)
                            g.notes.push_back(
                                "extrusion with multiple profiles: holes require verification");
                    } else if (solid == 52) {
                        require(sections.size() == 2 && section_tangents.size() >= 2,
                                "sweep profile/spine");
                        auto &profile = sections[0];
                        auto &spine = sections[1];
                        auto tan = section_tangents[1];
                        require(tan.size() == spine.size() && !tan.empty(), "sweep tangents");
                        for (auto &t : tan) {
                            auto n = norm(t);
                            require(n >= 1e-12, "degenerate sweep tangent");
                            t = scale(t, 1 / n);
                        }
                        auto rot = identity();
                        auto prev = tan[0];
                        std::vector<std::vector<Point3>> rings;
                        for (std::size_t i = 0; i < spine.size(); ++i) {
                            auto axis = cross(prev, tan[i]);
                            auto sn = norm(axis), cs = dot(prev, tan[i]);
                            if (sn > 1e-12) {
                                axis = scale(axis, 1 / sn);
                                Matrix4 k{};
                                k[0][1] = -axis[2];
                                k[0][2] = axis[1];
                                k[1][0] = axis[2];
                                k[1][2] = -axis[0];
                                k[2][0] = -axis[1];
                                k[2][1] = axis[0];
                                auto kk = multiply(k, k), step = identity();
                                for (unsigned x = 0; x < 3; ++x)
                                    for (unsigned y = 0; y < 3; ++y)
                                        step[x][y] += sn * k[x][y] + (1 - cs) * kk[x][y];
                                rot = multiply(step, rot);
                            }
                            std::vector<Point3> ring;
                            for (auto p : profile)
                                ring.push_back(
                                    add(vector_transform(rot, sub(p, spine[0])), spine[i]));
                            rings.push_back(ring);
                            prev = tan[i];
                        }
                        loft(g, rings, !payload.empty() && payload[0]);
                    } else if (solid == 56) {
                        require(section_native.size() == 2 && section_native_lengths.size() == 2,
                                "guided loft requires two sections");
                        require(section_closed.size() == 2 &&
                                    section_closed[0] == section_closed[1],
                                "guided loft profile boundary types differ");
                        const bool closed = section_closed[0];
                        require(closed || guide_counts.size() == 1,
                                "open guided loft cannot contain inner rings");
                        require(section_native_lengths[0] == section_native_lengths[1] &&
                                    guide_counts.size() == section_native_lengths[0].size(),
                                "guided loft native ring/group correspondence mismatch");
                        std::size_t total_guides = 0;
                        for (auto n : guide_counts) {
                            require(n <= guide_native.size() - total_guides,
                                    "guided loft group count exceeds source guides");
                            total_guides += n;
                        }
                        require(total_guides == guide_native.size(),
                                "unassigned guided loft curves");
                        Geometry result;
                        std::vector<Point3> bottom_cap, top_cap;
                        std::vector<std::size_t> cap_lengths;
                        std::size_t boundary_start = 0, guide_start = 0;
                        const bool capped = !payload.empty() && payload[0];
                        for (std::size_t ring = 0; ring < guide_counts.size(); ++ring) {
                            auto n = section_native_lengths[0][ring], ng = guide_counts[ring];
                            std::vector<GuidedBoundary> a(
                                section_native[0].begin() + boundary_start,
                                section_native[0].begin() + boundary_start + n);
                            std::vector<GuidedBoundary> b(
                                section_native[1].begin() + boundary_start,
                                section_native[1].begin() + boundary_start + n);
                            std::vector<GuidedBoundary> rails(guide_native.begin() + guide_start,
                                                              guide_native.begin() + guide_start +
                                                                  ng);
                            auto mesh = guided_surface(a, b, rails, policy, closed);
                            require(!capped || (mesh.cap_boundaries_closed[0] &&
                                                mesh.cap_boundaries_closed[1]),
                                    "guided loft cap boundary endpoints do not coincide");
                            require(mesh.rings.size() <= policy.max_segments &&
                                        mesh.rings.front().size() <=
                                            (policy.max_segments - result.vertices.size()) /
                                                mesh.rings.size(),
                                    "guided loft total vertex budget");
                            if (guide_counts.size() == 1) {
                                loft(result, mesh.rings, capped);
                            } else {
                                loft(result, mesh.rings, false);
                                cap_lengths.push_back(mesh.rings.front().size());
                                bottom_cap.insert(bottom_cap.end(), mesh.rings.front().begin(),
                                                  mesh.rings.front().end());
                                top_cap.insert(top_cap.end(), mesh.rings.back().begin(),
                                               mesh.rings.back().end());
                                mesh.note["source_ring_index"] = ring;
                                mesh.note["source_guide_group_index"] = ring;
                            }
                            result.notes.push_back(std::move(mesh.note));
                            boundary_start += n;
                            guide_start += ng;
                        }
                        if (guide_counts.size() > 1 && capped) {
                            require(bottom_cap.size() <=
                                        (policy.max_segments - result.vertices.size()) / 2,
                                    "guided loft cap vertex budget");
                            auto faces = polygon_faces(bottom_cap, cap_lengths);
                            for (auto &f : faces)
                                std::reverse(f.begin(), f.end());
                            append(result, bottom_cap, faces);
                            append(result, top_cap, polygon_faces(top_cap, cap_lengths));
                        }
                        append(g, result.vertices, result.faces);
                        for (auto &note : result.notes)
                            g.notes.push_back(std::move(note));
                    } else {
                        auto rings =
                            section_rings.empty() ? std::vector<std::size_t>{} : section_rings[0];
                        for (auto &r : section_rings)
                            require(r == rings, "loft ring topology");
                        loft(g, sections, !payload.empty() && payload[0], rings);
                    }
                    solid = 0;
                    sections.clear();
                    return;
                }
                if (op == 13) {
                    require(body.size() == 96, "matrix size");
                    stack.push_back(matrix);
                    Reader r(body);
                    auto m = identity();
                    for (unsigned i = 0; i < 3; ++i)
                        for (unsigned j = 0; j < 4; ++j)
                            m[i][j] = r.f64();
                    affine(m);
                    matrix = multiply(matrix, m);
                    return;
                }
                if (op == 14) {
                    require(!stack.empty(), "matrix stack underflow");
                    matrix = stack.back();
                    stack.pop_back();
                    return;
                }
                if (op == 34) {
                    matrix = multiply(matrix, instance_transform(body));
                    return;
                }
                g.unknown.push_back(
                    {{"opcode", op}, {"bytes", body.size()}, {"offset", cmd["offset"]}});
            };
            execute();
        } catch (const std::exception &e) {
            g.unknown.push_back({{"opcode", op}, {"offset", cmd["offset"]}, {"reason", e.what()}});
        }
        bool mirrored = reverses_winding(matrix);
        if (mirrored && !mesh_metadata.is_null())
            reverse_mesh_channel_corners(mesh_metadata);
        if (mirrored)
            for (auto i = first_face; i < g.faces.size(); ++i) {
                std::swap(g.faces[i][1], g.faces[i][2]);
                if (g.face_uvs[i])
                    std::swap((*g.face_uvs[i])[1], (*g.face_uvs[i])[2]);
                for (auto *indices : {&g.face_normal_indices, &g.face_uv_indices})
                    if ((*indices)[i])
                        std::swap((*(*indices)[i])[1], (*(*indices)[i])[2]);
            }
        for (auto item : std::vector<std::tuple<std::string, std::size_t, std::size_t>>{
                 {"faces", first_face, g.faces.size()},
                 {"lines", first_line, g.lines.size()},
                 {"texts", first_text, g.texts.size()}}) {
            auto channel = std::get<0>(item);
            auto start = std::get<1>(item), end = std::get<2>(item);
            if (end > start || (channel == "faces" && !mesh_metadata.is_null())) {
                g.primitive_ranges.push_back(
                    {{"channel", channel},
                     {"start", start},
                     {"count", end - start},
                     {"command_offset", cmd["offset"]},
                     {"opcode", op},
                     {"style", style},
                     {"winding_reversed", mirrored && channel == "faces"}});
                if (channel == "faces" && !mesh_metadata.is_null())
                    g.primitive_ranges.back()["mesh_channels"] = std::move(mesh_metadata);
            }
        }
    }
    if (solid || in_path || !stack.empty())
        g.unknown.push_back({{"reason", "unclosed geometry block"}});
    return g;
}
Geometry reconstruct_native(const Json &n, const Tessellation &policy) {
    Geometry g;
    auto b = bytesof(n["data"]);
    Reader r(b);
    unsigned kind = n["element_type"];
    try {
        if (kind == 37) {
            require(b.size() == 156, "native line length");
            r.p = 108;
            std::vector<Point3> p;
            while (r.left())
                p.push_back(r.doubles(3).get<Point3>());
            g.lines.push_back(p);
        } else if (kind == 40 || kind == 44) {
            r.p = 108;
            auto count = r.u32();
            require(b.size() == 116 + 24ull * count, "native point count");
            r.p = 116;
            std::vector<Point3> p;
            while (r.left())
                p.push_back(r.doubles(3).get<Point3>());
            g.lines.push_back(p);
        } else if (kind == 52) {
            require(b.size() == 196, "native arc length");
            r.p = 108;
            double start = r.f64(), sweep = r.f64(), rx = r.f64(), ry = r.f64();
            auto q = r.doubles(4), origin = r.doubles(3);
            g.lines.push_back(curve(5,
                                    {{"origin", origin},
                                     {"quaternion", q},
                                     {"radii", {rx, ry}},
                                     {"start_angle", start},
                                     {"sweep_angle", sweep}},
                                    policy));
        } else if (kind == 54) {
            require(b.size() >= 206, "native text length");
            r.p = 108;
            auto style = r.uints(4, 2);
            auto size = style[3].get<unsigned>();
            auto values = r.doubles(11);
            require(size <= b.size() - 206, "native text extent");
            auto text = native_string(slice(b, 206, size));
            g.texts.push_back({{"text", text},
                               {"origin", {values[8], values[9], values[10]}},
                               {"quaternion", {values[4], values[5], values[6], values[7]}},
                               {"width", values[2]},
                               {"height", values[3]},
                               {"font_scale", {values[0], values[1]}},
                               {"style_words", style},
                               {"source_encoding_marker", hex(slice(b, 206, 2))},
                               {"trailing_hex", hex(slice(b, 206 + size, b.size() - 206 - size))}});
        } else if (kind == 19 || kind == 20 || kind == 21)
            g.notes.push_back(
                "native composite header; constituent records are retained separately");
        else if (kind != 97)
            g.unknown.push_back({{"native_type", kind},
                                 {"reason", "direct native primitive not implemented"},
                                 {"bytes", b.size()}});
    } catch (const std::exception &e) {
        g.unknown.push_back({{"native_type", kind}, {"reason", e.what()}});
    }
    for (auto entry : std::vector<std::pair<std::string, std::size_t>>{
             {"faces", g.faces.size()}, {"lines", g.lines.size()}, {"texts", g.texts.size()}})
        if (entry.second) {
            Json links = Json::array();
            for (auto &l : n["links"])
                links.push_back({{"app", l["app"]},
                                 {"header", l["header"]},
                                 {"payload_hex", hex(bytesof(l["payload"]))}});
            g.primitive_ranges.push_back({{"channel", entry.first},
                                          {"start", 0},
                                          {"count", entry.second},
                                          {"style", Json::object()},
                                          {"source", "native"},
                                          {"native_type", kind},
                                          {"native_element_id", n["id"]},
                                          {"native_header_hex", hex(b)},
                                          {"native_links", links},
                                          {"style_status", "native_style_semantics_unassigned"}});
        }
    for (auto &t : g.texts) {
        t["source_origin"] = t["origin"];
        t["placement_matrix"] = identity();
    }
    return g;
}
void merge_mesh_channels(Geometry &target, const Geometry &source, std::size_t first_face) {
    require(first_face <= target.faces.size() &&
                source.faces.size() <= target.faces.size() - first_face,
            "mesh channel face range");
    require(source.normals.size() == source.source_normals.size(), "mesh normal source pool size");
    const auto normal_base = target.normals.size(), uv_base = target.uvs.size();
    require(normal_base + source.normals.size() <= UINT32_MAX &&
                uv_base + source.uvs.size() <= UINT32_MAX,
            "mesh channel index capacity");
    auto validate_indices = [&](const auto &indices, std::size_t count) {
        require(indices.empty() || indices.size() == source.faces.size(),
                "mesh channel corner count");
        for (const auto &tri : indices)
            if (tri)
                for (auto index : *tri)
                    require(index < count, "mesh channel index range");
    };
    validate_indices(source.face_normal_indices, source.normals.size());
    validate_indices(source.face_uv_indices, source.uvs.size());
    auto append_indices = [&](auto &dest, const auto &indices, std::size_t base) {
        dest.resize(target.faces.size());
        for (std::size_t i = 0; i < source.faces.size(); ++i) {
            auto value = indices.empty() ? std::optional<Triangle>() : indices[i];
            if (value)
                for (auto &index : *value) {
                    index += static_cast<std::uint32_t>(base);
                }
            dest[first_face + i] = value;
        }
    };
    append_indices(target.face_normal_indices, source.face_normal_indices, normal_base);
    append_indices(target.face_uv_indices, source.face_uv_indices, uv_base);
    target.normals.insert(target.normals.end(), source.normals.begin(), source.normals.end());
    target.source_normals.insert(target.source_normals.end(), source.source_normals.begin(),
                                 source.source_normals.end());
    target.uvs.insert(target.uvs.end(), source.uvs.begin(), source.uvs.end());
}
void merge_geometry(Geometry &target, const Geometry &source, const Matrix4 &m, bool parent) {
    affine(m);
    std::map<std::string, std::size_t> offsets = {{"faces", target.faces.size()},
                                                  {"lines", target.lines.size()},
                                                  {"texts", target.texts.size()}};
    bool mirror = reverses_winding(m);
    std::vector<Point3> points;
    for (auto p : source.vertices)
        points.push_back(transform(m, p));
    auto faces = source.faces;
    auto uv = source.face_uvs;
    if (mirror)
        for (std::size_t i = 0; i < faces.size(); ++i) {
            std::swap(faces[i][1], faces[i][2]);
            if (uv[i])
                std::swap((*uv[i])[1], (*uv[i])[2]);
        }
    append(target, points, faces, &uv, &source.face_source_polygons);
    const auto normal_base = target.normals.size();
    merge_mesh_channels(target, source, offsets.at("faces"));
    transform_normals(target, m, normal_base);
    if (mirror)
        for (auto *indices : {&target.face_normal_indices, &target.face_uv_indices})
            for (auto i = offsets.at("faces"); i < target.faces.size(); ++i)
                if ((*indices)[i])
                    std::swap((*(*indices)[i])[1], (*(*indices)[i])[2]);
    for (auto &line : source.lines) {
        std::vector<Point3> p;
        for (auto v : line)
            p.push_back(transform(m, v));
        target.lines.push_back(p);
    }
    for (auto t : source.texts) {
        t["source_origin"] = t.value("source_origin", t["origin"]);
        t["origin"] = transform(m, t["origin"].get<Point3>());
        auto previous =
            t.contains("placement_matrix") ? t["placement_matrix"].get<Matrix4>() : identity();
        t["placement_matrix"] = multiply(m, previous);
        if (parent) {
            t["parent_matrix"] = m;
            t["frame_policy"] = "quaternion, width and height remain source-local; "
                                "placement_matrix maps the source text frame to scene coordinates";
        }
        target.texts.push_back(t);
    }
    for (auto &u : source.unknown)
        target.unknown.push_back(u);
    for (auto &n : source.notes)
        target.notes.push_back(n);
    for (auto range : source.primitive_ranges) {
        auto channel = range["channel"].get<std::string>();
        range["start"] = range["start"].get<std::size_t>() + offsets.at(channel);
        range["winding_reversed"] =
            bool(range.value("winding_reversed", false) ^ (mirror && channel == "faces"));
        if (mirror && range.contains("mesh_channels"))
            reverse_mesh_channel_corners(range["mesh_channels"]);
        target.primitive_ranges.push_back(range);
    }
}
Json geometry_json(const Geometry &g) {
    Json uv = Json::array(), src = Json::array();
    for (auto &v : g.face_uvs)
        uv.push_back(v ? Json(*v) : Json());
    for (auto &v : g.face_source_polygons)
        src.push_back(v ? Json(*v) : Json());
    auto optional_values = [](const auto &values) {
        Json result = Json::array();
        for (const auto &value : values)
            result.push_back(value ? Json(*value) : Json());
        return result;
    };
    return {{"vertices", g.vertices},
            {"faces", g.faces},
            {"lines", g.lines},
            {"texts", g.texts},
            {"unknown", g.unknown},
            {"notes", g.notes},
            {"primitive_ranges", g.primitive_ranges},
            {"face_uvs", uv},
            {"source_normals", g.source_normals},
            {"normals", optional_values(g.normals)},
            {"uvs", g.uvs},
            {"face_normal_indices", optional_values(g.face_normal_indices)},
            {"face_uv_indices", optional_values(g.face_uv_indices)},
            {"face_source_polygons", src}};
}
} // namespace p3d
