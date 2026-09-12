#include "geometry.hpp"
#include "glu/sk_glu.h"
#include <deque>

namespace p3d {
namespace {
struct Corner {
    Point3 point;
    std::array<float, 2> uv;
    std::optional<std::size_t> source_corner;
};
struct Context {
    std::deque<Corner> vertices;
    std::vector<Corner> output;
    std::vector<unsigned> errors;
    std::exception_ptr failure;
    std::size_t limit;
};
template <class F> void callback(Context &c, F fn) noexcept {
    if (c.failure)
        return;
    try {
        fn();
    } catch (...) {
        c.failure = std::current_exception();
    }
}
void vertex(void *data, void *context) noexcept {
    auto &c = *static_cast<Context *>(context);
    callback(c, [&] {
        require(c.output.size() < c.limit, "native tessellation output limit");
        c.output.push_back(*static_cast<Corner *>(data));
    });
}
void combine(double point[3], void *[4], float[4], void **data, void *context) noexcept {
    auto &c = *static_cast<Context *>(context);
    *data = nullptr;
    callback(c, [&] {
        require(c.vertices.size() < c.limit, "native tessellation vertex limit");
        // The native callback ignores the contributing vertices and weights.
        c.vertices.push_back({{point[0], point[1], point[2]}, {0, 0}, {}});
        *data = &c.vertices.back();
    });
}
void edge(unsigned char, void *) noexcept {}
void error(unsigned code, void *context) noexcept {
    auto &c = *static_cast<Context *>(context);
    callback(c, [&] { c.errors.push_back(code); });
}
void begin(unsigned mode, void *context) noexcept {
    auto &c = *static_cast<Context *>(context);
    callback(c, [&] { require(mode == GL_TRIANGLES, "native tessellation primitive mode"); });
}
std::vector<Corner> tessellate(const std::vector<Corner> &input, std::size_t limit) {
    Context c;
    c.limit = limit;
    require(input.size() <= limit, "native tessellation input limit");
    for (const auto &v : input)
        c.vertices.push_back(v);
    std::unique_ptr<GLUtesselator, decltype(&gluDeleteTess)> tess(gluNewTess(), gluDeleteTess);
    require(bool(tess), "native tessellation allocation");
    gluTessCallback(tess.get(), GLU_TESS_BEGIN_DATA, reinterpret_cast<void (*)()>(begin));
    gluTessCallback(tess.get(), GLU_TESS_VERTEX_DATA, reinterpret_cast<void (*)()>(vertex));
    gluTessCallback(tess.get(), GLU_TESS_EDGE_FLAG_DATA, reinterpret_cast<void (*)()>(edge));
    gluTessCallback(tess.get(), GLU_TESS_COMBINE_DATA, reinterpret_cast<void (*)()>(combine));
    gluTessCallback(tess.get(), GLU_TESS_ERROR_DATA, reinterpret_cast<void (*)()>(error));
    gluTessProperty(tess.get(), GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_ODD);
    gluTessProperty(tess.get(), GLU_TESS_BOUNDARY_ONLY, 0);
    gluTessProperty(tess.get(), GLU_TESS_TOLERANCE, 1e-6);
    gluTessBeginPolygon(tess.get(), &c);
    gluTessBeginContour(tess.get());
    // Input pointers remain stable when the combine callback appends vertices.
    for (std::size_t i = 0; i < input.size(); ++i)
        gluTessVertex(tess.get(), c.vertices[i].point.data(), &c.vertices[i]);
    gluTessEndContour(tess.get());
    gluTessEndPolygon(tess.get());
    if (c.failure)
        std::rethrow_exception(c.failure);
    require(c.errors.empty(), "GLU tessellation reported an error");
    require(c.output.size() % 3 == 0, "native tessellation incomplete triangle");
    return c.output;
}
} // namespace

Json triangulate_native_mesh(const Json &decoded, std::size_t max_output_corners) {
    Json out = {{"reader_profile", "bimbase_2025_triangulate2_glu_input"},
                {"status", "invalid"},
                {"tessellator", "sgi_glu"},
                {"winding_rule", "odd"},
                {"tolerance", 1e-6},
                {"scope", "local_indexed_mesh_with_source_corners"},
                {"triangles", Json::array()}};
    try {
        const auto &channels = decoded.at("mesh_channels");
        const auto &routing = channels.at("native_triangulation");
        require(routing.at("status") == "mapped" &&
                    routing.at("source_polygon_layout_matches") == true,
                "native mesh input routing is unavailable");
        const auto mode = routing.at("normal_mode").get<std::string>();
        require(mode == "smoothing_groups" || mode == "flat_triangles", "native mesh normal mode");
        auto points = decoded.at("points").get<std::vector<Point3>>();
        const auto &uvs = channels.at("face_uv_points");
        std::size_t uv_index = 0, corner_count = 0;
        Json triangles = Json::array();
        for (const auto &face : routing.at("polygons")) {
            auto index = face.at("source_polygon").get<std::size_t>();
            const auto &indices = decoded.at("polygons").at(index).at("point_indices");
            std::vector<Corner> input;
            for (std::size_t k = 0; k < indices.size(); ++k) {
                const auto id = indices[k].get<std::int64_t>();
                require(id > 0 && std::uint64_t(id) <= points.size(), "native mesh point index");
                const auto p = points[std::size_t(id - 1)];
                const auto uv = uvs.at(uv_index++).get<Point2>();
                std::array<float, 2> value{float(uv[0]), float(uv[1])};
                for (double x : p)
                    require(std::isfinite(x), "nonfinite native mesh point");
                for (float x : value)
                    require(std::isfinite(x), "nonfinite native mesh UV");
                input.push_back({p, value, k});
            }
            const auto result = tessellate(input, max_output_corners - corner_count);
            corner_count += result.size();
            for (std::size_t i = 0; i < result.size(); i += 3) {
                Json triangle = {{"source_polygon", index},
                                 {"material_id", face.at("material_id")},
                                 {"normal_group", face.at("normal_group")},
                                 {"corners", Json::array()}};
                for (unsigned j = 0; j < 3; ++j) {
                    const auto &v = result[i + j];
                    triangle["corners"].push_back(
                        {{"point", v.point},
                         {"uv", v.uv},
                         {"source_corner", v.source_corner ? Json(*v.source_corner) : Json()},
                         {"generated", !v.source_corner.has_value()}});
                }
                triangles.push_back(std::move(triangle));
            }
        }
        out["buffers"] = assemble_native_mesh_buffers(triangles, mode == "smoothing_groups");
        out["triangles"] = std::move(triangles);
        out["status"] = "triangulated";
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
