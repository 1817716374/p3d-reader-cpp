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
    check(limited["status"] == "invalid" && limited["triangles"].empty(),
          "capacity failure does not publish partial triangles");
    auto bad = source;
    bad["polygons"][0]["point_indices"][0] = 0;
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "native point indices remain positive and bounded");
    bad = source;
    bad["mesh_channels"]["native_triangulation"]["status"] = "unsafe_material_copy";
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "unsafe native input routing is not bypassed");
    bad = source;
    bad["mesh_channels"]["face_uv_points"][0][0] = 1e100;
    check(triangulate_native_mesh(bad)["status"] == "invalid",
          "unrepresentable float UV is reported");
    std::vector<std::future<Json>> futures;
    for (unsigned i = 0; i < 12; ++i)
        futures.push_back(
            std::async(std::launch::async, [=] { return triangulate_native_mesh(crossing); }));
    for (auto &future : futures)
        check(future.get() == crossed,
              "independent tessellators are safe across concurrent parsing calls");
    return checks;
}
