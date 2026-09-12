#include "internal.hpp"
#include <future>

unsigned mesh_tessellation_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto input = [](const std::vector<Point3> &points) {
        Json indices = Json::array(), uv = Json::array();
        for (std::size_t i = 0; i < points.size(); ++i) {
            indices.push_back(i + 1);
            uv.push_back({.1 + double(i), .3});
        }
        return Json{{"points", points},
                    {"polygons", Json::array({{{"point_indices", indices}}})},
                    {"mesh_channels",
                     {{"face_uv_points", uv},
                      {"native_triangulation",
                       {{"status", "mapped"},
                        {"normal_mode", "smoothing_groups"},
                        {"source_polygon_layout_matches", true},
                        {"polygons", Json::array({{{"source_polygon", 0},
                                                   {"material_id", UINT64_MAX},
                                                   {"normal_group", -3}}})}}}}}};
    };
    auto area = [](const Json &result) {
        double sum = 0;
        for (const auto &triangle : result["triangles"]) {
            auto a = triangle["corners"][0]["point"].get<Point3>();
            auto b = triangle["corners"][1]["point"].get<Point3>();
            auto c = triangle["corners"][2]["point"].get<Point3>();
            sum += std::abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
        }
        return sum;
    };
    auto source = input({{0, 0, 0}, {3, 0, 0}, {3, 3, 0}, {1, 1, 0}, {0, 3, 0}});
    auto result = triangulate_native_mesh(source);
    check(result["status"] == "triangulated" && result["triangles"].size() == 3 &&
              area(result) == 6,
          "concave contour is tessellated without fan overlap");
    for (const auto &triangle : result["triangles"]) {
        check(triangle["material_id"] == UINT64_MAX && triangle["normal_group"] == -3,
              "full material and signed normal group references survive tessellation");
        for (const auto &corner : triangle["corners"]) {
            auto i = corner["source_corner"].get<std::size_t>();
            check(corner["point"] == source["points"][i] &&
                      corner["uv"][0] ==
                          double(
                              float(source["mesh_channels"]["face_uv_points"][i][0].get<double>())),
                  "original corners retain source indices with native float UV conversion");
        }
    }
    auto crossing = input({{0, 0, 0}, {2, 2, 0}, {0, 2, 0}, {2, 0, 0}});
    auto crossed = triangulate_native_mesh(crossing);
    check(crossed["status"] == "triangulated" && crossed["triangles"].size() == 2 &&
              area(crossed) == 2,
          "self crossing contour creates both odd winding regions");
    unsigned generated = 0;
    for (const auto &triangle : crossed["triangles"])
        for (const auto &corner : triangle["corners"])
            if (corner["generated"] == true) {
                ++generated;
                check(corner["source_corner"].is_null() && corner["point"] == Json({1., 1., 0.}) &&
                          corner["uv"] == Json({0., 0.}),
                      "combine callback generates coordinates and zero UV, not weighted UV");
            }
    check(generated == 2, "intersection appears in each output triangle that uses it");
    for (const auto &mesh : {result, crossed}) {
        const auto &buffers = mesh.at("buffers");
        check(buffers.at("faces").size() == mesh.at("triangles").size() &&
                  buffers.at("points").size() == buffers.at("normals").size() &&
                  buffers.at("points").size() == buffers.at("uvs").size(),
              "GLU output is assembled into aligned indexed buffers");
        for (std::size_t i = 0; i < mesh.at("triangles").size(); ++i) {
            const auto &triangle = mesh.at("triangles")[i];
            check(buffers.at("face_material_ids")[i] == triangle.at("material_id"),
                  "indexed faces retain GLU callback order and per-face materials");
            for (unsigned j = 0; j < 3; ++j) {
                auto id = buffers.at("faces")[i][j].get<std::size_t>();
                const auto &corner = triangle.at("corners")[j];
                check(buffers.at("uvs")[id] == corner.at("uv") &&
                          buffers.at("points")[id] == corner.at("point"),
                      "indexed buffer corner follows the actual GLU original or generated vertex");
            }
        }
    }
    auto flat_crossing = crossing;
    flat_crossing["mesh_channels"]["native_triangulation"]["normal_mode"] = "flat_triangles";
    auto flat_crossed = triangulate_native_mesh(flat_crossing);
    check(flat_crossed["status"] == "triangulated" &&
              flat_crossed["buffers"]["vertex_key"] == "float32_position_normal_uv",
          "normal routing selects the flat postprocessing branch after GLU");
    auto hole = input({{0, 0, 0},
                       {4, 0, 0},
                       {4, 4, 0},
                       {0, 4, 0},
                       {0, 0, 0},
                       {1, 1, 0},
                       {1, 3, 0},
                       {3, 3, 0},
                       {3, 1, 0},
                       {1, 1, 0},
                       {0, 0, 0}});
    auto holed = triangulate_native_mesh(hole);
    check(holed["status"] == "triangulated" && area(holed) == 12,
          "bridged contour preserves the inner odd-winding hole");
    auto limited = triangulate_native_mesh(source, 3);
    check(limited["status"] == "invalid" && limited["triangles"].empty() &&
              !limited.contains("buffers"),
          "capacity failure does not publish partial triangles");
    auto bad = source;
    bad["polygons"][0]["point_indices"][0] = 0;
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "native point indices remain positive and bounded");
    bad = source;
    bad["mesh_channels"]["native_triangulation"].erase("normal_mode");
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "missing native normal routing is not guessed");
    bad = source;
    bad["mesh_channels"]["native_triangulation"]["status"] = "unsafe_material_copy";
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "unsafe native input routing is not bypassed");
    bad = source;
    bad["mesh_channels"]["face_uv_points"][0][0] = 1e100;
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "unrepresentable float UV is reported");
    auto huge = input({{1e40, 0, 0}, {1e40, 1, 0}, {1e40, 0, 1}});
    for (const auto *mode : {"flat_triangles", "smoothing_groups"}) {
        huge["mesh_channels"]["native_triangulation"]["normal_mode"] = mode;
        const auto failed = triangulate_native_mesh(huge);
        check(failed["status"] == "invalid" && failed["triangles"].empty() &&
                  !failed.contains("buffers") &&
                  failed["error"] == "native mesh float export overflow",
              "export failure after successful GLU does not publish partial results");
    }
    std::vector<std::future<Json>> futures;
    for (unsigned i = 0; i < 12; ++i)
        futures.push_back(
            std::async(std::launch::async, [=] { return triangulate_native_mesh(crossing); }));
    for (auto &future : futures)
        check(future.get() == crossed,
              "independent tessellators are safe across concurrent parsing calls");
    return checks;
}
