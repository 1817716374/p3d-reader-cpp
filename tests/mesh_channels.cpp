#include "geometry.hpp"
#include <cstring>
#include <limits>
using namespace p3d;
namespace {
template <class T> void put(Bytes &b, T value) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&value);
    b.insert(b.end(), p, p + sizeof(value));
}
Json command(unsigned op, const Bytes &body) {
    return {
        {"op", op}, {"offset", 2}, {"body", rawbytes(body)}, {"decoded", command_fields(op, body)}};
}
Json mesh(const std::vector<std::int32_t> &pi, const std::vector<std::int32_t> &ni,
          const std::vector<std::int32_t> &ui, const std::vector<Point3> &points,
          const std::vector<Point3> &normals, const std::vector<Point2> &uvs) {
    Bytes b;
    put(b, std::uint32_t(0));
    put(b, std::uint32_t(pi.size()));
    for (const auto *indices : {&pi, &ni, &ui}) {
        put(b, std::uint32_t(indices->size()));
        for (auto i : *indices)
            put(b, i);
    }
    auto pool = [&](const auto &values) {
        put(b, std::uint32_t(values.size()));
        for (auto point : values)
            for (double value : point)
                put(b, value);
    };
    pool(points);
    pool(normals);
    pool(uvs);
    b.resize(b.size() + 24);
    return command(25, b);
}
Json placement(const Matrix4 &matrix) {
    Bytes b;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 4; ++j)
            put(b, matrix[i][j]);
    return command(13, b);
}
bool near(Point3 a, Point3 b) {
    for (unsigned i = 0; i < 3; ++i)
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
            std::abs(a[i] - b[i]) > 1e-12 * std::max(1., std::abs(b[i])))
            return false;
    return true;
}
} // namespace
unsigned mesh_channel_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    const std::vector<Point3> points = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    const std::vector<Point3> normals = {{1, 2, 4}, {0, 0, 2}, {0, 0, 2}, {0, 0, 0}};
    const std::vector<Point2> uvs = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {3, 3}, {0, 0}};
    auto cmd = mesh({1, -2, 3, 0, 1, 3, 4, 0}, {2, -2, 1, 0, 3, 1, 3, 0}, {1, -2, 3, 0, 5, 3, 4, 0},
                    points, normals, uvs);
    auto g = reconstruct(Json::array({cmd}), {});
    check(g.unknown.empty() && g.faces.size() == 2, "explicit mesh decoded");
    check(g.source_normals == normals && g.uvs == uvs,
          "unused and duplicate source entries retained");
    check(g.normals.size() == normals.size() && *g.normals[0] == normals[0] &&
              *g.normals[1] == normals[1] && *g.normals[3] == normals[3],
          "source normal magnitudes retained");
    check(*g.face_normal_indices[0] == Triangle{1, 1, 0} &&
              *g.face_normal_indices[1] == Triangle{2, 0, 2},
          "independent signed normal lookup");
    check(*g.face_uv_indices[0] == Triangle{0, 1, 2} && *g.face_uv_indices[1] == Triangle{4, 2, 3},
          "UV seams preserve independent source indices");
    check(cmd["decoded"]["polygons"][0]["normal_indices"][1] == -2,
          "signed source indices remain available");
    for (std::size_t f = 0; f < g.faces.size(); ++f)
        for (unsigned c = 0; c < 3; ++c)
            check((*g.face_uvs[f])[c] == g.uvs[(*g.face_uv_indices[f])[c]],
                  "UV expansion agrees with source pool");

    auto concave = mesh({1, 2, 3, 4, 5, 0}, {-2, 1, 3, 2, 1, 0}, {5, 2, 4, 1, 3, 0},
                        {{0, 0, 0}, {2, 0, 0}, {1, 1, 0}, {2, 2, 0}, {0, 2, 0}}, normals, uvs);
    auto poly = reconstruct(Json::array({concave}), {});
    check(poly.unknown.empty() && poly.faces.size() == 3, "concave polygon triangulated");
    const unsigned normal_by_point[] = {1, 0, 2, 1, 0}, uv_by_point[] = {4, 1, 3, 0, 2};
    for (std::size_t f = 0; f < poly.faces.size(); ++f)
        for (unsigned c = 0; c < 3; ++c) {
            const auto p = poly.faces[f][c];
            check((*poly.face_normal_indices[f])[c] == normal_by_point[p] &&
                      (*poly.face_uv_indices[f])[c] == uv_by_point[p],
                  "triangulation preserves corner channels");
        }
    auto no_indices = mesh({1, 2, 3, 0}, {}, {}, points, normals, uvs);
    const std::vector<std::int32_t> two_faces = {1, 2, 3, 0, 1, 3, 4, 0};
    auto bad_normal =
        mesh(two_faces, {9, 2, 1, 0, 3, 1, 3, 0}, {1, 2, 3, 0, 5, 3, 4, 0}, points, normals, uvs);
    auto partial_normal = reconstruct(Json::array({bad_normal}), {});
    check(!bad_normal["decoded"].contains("field_decode_error") &&
              bad_normal["decoded"]["index_bindings"]["normal_indices"]
                        ["unavailable_read_positions"] == Json({0}) &&
              bad_normal["decoded"]["polygons"][0]["normal_indices"][0] == 9,
          "invalid optional normal index retains its exact source position and value");
    check(partial_normal.faces.size() == 2 && partial_normal.unknown.size() == 1 &&
              !partial_normal.face_normal_indices[0] && partial_normal.face_normal_indices[1] &&
              partial_normal.face_uv_indices[0] && partial_normal.face_uv_indices[1] &&
              partial_normal.source_normals == normals,
          "bad normal reference does not discard topology, UVs, pool values or another face");
    auto bad_uv =
        mesh(two_faces, {2, 2, 1, 0, 3, 1, 3, 0}, {1, 2, 3, 0, 99, 3, 4, 0}, points, normals, uvs);
    auto partial_uv = reconstruct(Json::array({bad_uv}), {});
    check(partial_uv.faces.size() == 2 && partial_uv.unknown.size() == 1 &&
              partial_uv.face_uv_indices[0] && !partial_uv.face_uv_indices[1] &&
              partial_uv.face_uvs[0] && !partial_uv.face_uvs[1] &&
              partial_uv.face_normal_indices[0] && partial_uv.face_normal_indices[1] &&
              partial_uv.uvs == uvs &&
              bad_uv["decoded"]["index_bindings"]["uv_indices"]["unavailable_read_positions"] ==
                  Json({4}),
          "bad UV reference disables only affected triangle UV and preserves normals");
    auto absent_normal_pool = mesh(two_faces, {2, 2, 1, 0, 3, 1, 3, 0}, {}, points, {}, {});
    auto absent_normal_geo = reconstruct(Json::array({absent_normal_pool}), {});
    check(absent_normal_pool["decoded"]["index_bindings"]["normal_indices"]["status"] ==
                  "pool_absent" &&
              absent_normal_geo.unknown.empty() && absent_normal_geo.faces.size() == 2 &&
              absent_normal_geo.source_normals.empty() &&
              !absent_normal_geo.face_normal_indices[0] &&
              !absent_normal_geo.face_normal_indices[1],
          "indices with no normal pool are preserved without accessing a missing pool");
    auto extreme = mesh(two_faces, {INT32_MIN, 1, 2, 0, INT32_MAX, 1, 2, 0},
                        {INT32_MAX, 1, 2, 0, INT32_MIN, 1, 2, 0}, points, normals, uvs);
    auto extreme_geo = reconstruct(Json::array({extreme}), {});
    check(extreme_geo.faces.size() == 2 && extreme_geo.unknown.size() == 2 &&
              !extreme_geo.face_normal_indices[0] && !extreme_geo.face_uv_indices[1] &&
              extreme["decoded"]["index_arrays"][1][0] == INT32_MIN &&
              extreme["decoded"]["index_bindings"]["uv_indices"]["unavailable_read_positions"] ==
                  Json({0, 4}),
          "signed int32 extremes are diagnosed without overflow or geometry loss");
    auto invalid_points = mesh({99, 2, 3, 0}, {}, {}, points, normals, uvs);
    auto invalid_count = mesh({1, 2, 3, 0}, {1, 2}, {}, points, normals, uvs);
    auto invalid_separator = mesh({1, 2, 3, 0}, {1, 0, 3, 0}, {}, points, normals, uvs);
    check(invalid_points["decoded"].contains("field_decode_error") &&
              invalid_count["decoded"].contains("field_decode_error") &&
              invalid_separator["decoded"].contains("field_decode_error"),
          "unsafe point topology and inconsistent index layout remain rejected");
    auto missing = reconstruct(Json::array({no_indices}), {});
    check(missing.normals.size() == normals.size() && missing.uvs == uvs &&
              !missing.face_normal_indices[0] && !missing.face_uv_indices[0] &&
              !missing.face_uvs[0],
          "unindexed pools retained without inferred point-index fallback");
    auto empty = reconstruct(Json::array({mesh({}, {}, {}, {}, normals, uvs)}), {});
    check(empty.unknown.empty() && empty.faces.empty() && empty.source_normals == normals &&
              empty.uvs == uvs,
          "pools without faces retained");
    auto both = reconstruct(Json::array({cmd, no_indices, cmd}), {});
    check(both.unknown.empty() && both.faces.size() == 5 && both.normals.size() == 12 &&
              both.uvs.size() == 18,
          "separate command pools concatenated without deduplication");
    check(!both.face_normal_indices[2] && *both.face_normal_indices[3] == Triangle{9, 9, 8} &&
              *both.face_uv_indices[3] == Triangle{12, 13, 14},
          "pool offsets and absent channels aligned");

    auto m = identity();
    m[0] = {2, 1, 0, 19};
    m[1] = {0, 3, 0, -7};
    m[2] = {0, 0, 4, 5};
    Geometry placed;
    Geometry placed_partial;
    auto reflection = identity();
    reflection[0][0] = -1;
    merge_geometry(placed_partial, partial_uv, reflection);
    check(placed_partial.faces.size() == 2 && placed_partial.face_uv_indices[0] &&
              !placed_partial.face_uv_indices[1] && placed_partial.unknown == partial_uv.unknown &&
              *placed_partial.face_uv_indices[0] == Triangle{0, 2, 1},
          "partial optional channel and source diagnostics survive a mirrored instance");
    merge_geometry(placed, g, m);
    check(placed.source_normals == normals && near(*placed.normals[0], {.5, .5, 1}),
          "inverse-transpose handles shear and unequal scales without normalizing");
    check(near(*placed.normals[1], {0, 0, .5}), "normal unaffected by translation");
    auto direct = reconstruct(Json::array({placement(m), cmd, command(14, {})}), {});
    check(direct.unknown.empty() && direct.vertices == placed.vertices &&
              direct.normals == placed.normals &&
              direct.face_normal_indices == placed.face_normal_indices,
          "command and instance placement agree");
    auto inverse = identity();
    inverse[0] = {.5, -1. / 6, 0, 0};
    inverse[1][1] = 1. / 3;
    inverse[2][2] = .25;
    Geometry restored;
    merge_geometry(restored, placed, inverse);
    for (std::size_t i = 0; i < normals.size(); ++i)
        check(near(*restored.normals[i], normals[i]), "inverse placement restores normals");
    auto rotation = identity();
    rotation[0] = {0, -1, 0, 0};
    rotation[1] = {1, 0, 0, 0};
    Geometry rotated;
    merge_geometry(rotated, g, rotation);
    check(near(*rotated.normals[0], {-2, 1, 4}), "normal rotation requires pivot exchange");
    auto mirror = identity();
    mirror[0][0] = -1;
    Geometry mirrored, twice;
    merge_geometry(mirrored, g, mirror);
    check(mirrored.faces[0] == Triangle{0, 2, 1} &&
              *mirrored.face_normal_indices[0] == Triangle{1, 0, 1} &&
              *mirrored.face_uv_indices[0] == Triangle{0, 2, 1} &&
              *mirrored.normals[0] == Point3{-1, 2, 4},
          "mirror swaps all corner channels once");
    merge_geometry(twice, mirrored, mirror);
    check(twice.faces == g.faces && twice.normals == g.normals && twice.face_uvs == g.face_uvs &&
              twice.face_normal_indices == g.face_normal_indices &&
              twice.face_uv_indices == g.face_uv_indices,
          "double mirror restores independent channels");
    auto direct_mirror =
        reconstruct(Json::array({placement(mirror), cmd, command(14, {}), cmd}), {});
    check(direct_mirror.faces[0] == mirrored.faces[0] &&
              direct_mirror.normals[0] == mirrored.normals[0] &&
              direct_mirror.face_uv_indices[0] == mirrored.face_uv_indices[0] &&
              direct_mirror.normals[4] == g.normals[0],
          "command mirror and matrix pop affect only current pools");

    auto singular = identity();
    singular[1][1] = 0;
    Geometry collapsed;
    merge_geometry(collapsed, g, singular);
    check(collapsed.source_normals == normals && collapsed.faces == g.faces &&
              collapsed.face_normal_indices == g.face_normal_indices && !collapsed.normals[0] &&
              !collapsed.normals[3],
          "singular placement preserves source data and unresolved normal slots");
    for (double scale : {1e-150, 1e150}) {
        auto uniform = identity();
        uniform[0][0] = uniform[1][1] = uniform[2][2] = scale;
        Geometry scaled;
        merge_geometry(scaled, g, uniform);
        check(scaled.normals[0].has_value(), "extreme invertible normal remains available");
        auto restored_normal = *scaled.normals[0];
        for (auto &value : restored_normal)
            value *= scale;
        check(near(restored_normal, normals[0]),
              "extreme invertible scale does not use determinant threshold");
        uniform[0][0] = -scale;
        Geometry negative;
        merge_geometry(negative, g, uniform);
        check(negative.normals[0].has_value(), "extreme mirrored normal remains available");
        restored_normal = *negative.normals[0];
        for (auto &value : restored_normal)
            value *= scale;
        check(negative.faces == mirrored.faces &&
                  negative.face_normal_indices == mirrored.face_normal_indices &&
                  negative.face_uv_indices == mirrored.face_uv_indices &&
                  near(restored_normal, {-1, 2, 4}),
              "extreme mirror scale preserves orientation and corner channels");
    }
    auto bad = g;
    bad.normals[0] = Point3{std::numeric_limits<double>::infinity(), 0, 0};
    bad.source_normals[0] = *bad.normals[0];
    Geometry nonfinite;
    merge_geometry(nonfinite, bad, identity());
    check(!nonfinite.normals[0] && std::isinf(nonfinite.source_normals[0][0]) &&
              nonfinite.normals[1],
          "nonfinite normal does not discard other pool entries");
    auto json = geometry_json(collapsed);
    check(json["normals"][0].is_null() && json["source_normals"] == Json(normals) &&
              json["face_normal_indices"][0] == Json({1, 1, 0}),
          "JSON retains unavailable-normal index and source");
    Geometry legacy;
    legacy.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    legacy.faces = {{0, 1, 2}};
    legacy.face_uvs.resize(1);
    legacy.face_source_polygons.resize(1);
    Geometry mixed;
    merge_geometry(mixed, legacy, identity());
    merge_geometry(mixed, g, identity());
    check(mixed.face_normal_indices.size() == 3 && !mixed.face_normal_indices[0] &&
              mixed.face_normal_indices[1] == g.face_normal_indices[0],
          "non-mesh faces align with mesh channels");
    Geometry destination;
    destination.faces.resize(2);
    bad = g;
    bad.face_uv_indices[1] = Triangle{0, 0, 100};
    bool threw = false;
    try {
        merge_mesh_channels(destination, bad, 0);
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw && destination.normals.empty() && destination.face_normal_indices.empty(),
          "invalid independent indices rejected before copying channels");

    NativeScene scene;
    auto def = std::make_shared<GeometryDefinition>();
    def->geometry = g;
    def->source_key = "mesh";
    scene.definitions.push_back(def);
    scene.metadata = {{"color_tables", Json::array()},
                      {"materials", {{"definitions", Json::array()}}},
                      {"document_graph", Json::object()}};
    SceneElement element;
    element.metadata = {{"unknown", Json::array()},
                        {"notes", Json::array()},
                        {"model_id", 1},
                        {"binding_status", "missing"}};
    GeometryInstance instance;
    instance.definition = 0;
    instance.matrix = mirror;
    element.instances.push_back(instance);
    instance.apply_placement = false;
    element.instances.push_back(instance);
    scene.elements.push_back(element);
    scene.for_each_primitive([&](const PrimitiveView &view) {
        check(view.winding_reversed == (view.instance_index == 0),
              "borrowed primitive orientation matches expansion policy");
    });
    auto extreme_mirror = mirror;
    extreme_mirror[0][0] = -1e-150;
    extreme_mirror[1][1] = extreme_mirror[2][2] = 1e-150;
    scene.elements[0].instances[0].matrix = extreme_mirror;
    scene.for_each_primitive([&](const PrimitiveView &view) {
        check(view.winding_reversed == (view.instance_index == 0),
              "borrowed primitive orientation handles tiny mirror determinant");
    });
    scene.elements[0].instances[0].matrix = mirror;
    const auto expanded = scene.expanded()["elements"][0];
    check(expanded["normals"].size() == 8 && expanded["uvs"].size() == 12 &&
              expanded["face_normal_indices"][0] == Json({1, 0, 1}) &&
              expanded["face_normal_indices"][2] == Json({5, 5, 4}) &&
              expanded["face_uv_indices"][2] == Json({6, 7, 8}),
          "expanded scene rebases independent instance pools");
    check(def->geometry.source_normals == normals && def->geometry.normals == g.normals &&
              expanded["normals"][0] == Json({-1, 2, 4}) &&
              expanded["normals"][4] == Json({1, 2, 4}),
          "expansion does not mutate shared definitions and respects disabled placement");
    return checks;
}
